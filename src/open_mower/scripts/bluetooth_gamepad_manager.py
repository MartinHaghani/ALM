#!/usr/bin/env python3
import json
import threading

import rospy
from mower_msgs.srv import BluetoothDeviceCommand, BluetoothDeviceCommandResponse
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

try:
    import dbus
except ImportError:
    dbus = None

try:
    import dbus.service
    from dbus.mainloop.glib import DBusGMainLoop
    from gi.repository import GLib
except ImportError:
    DBusGMainLoop = None
    GLib = None


BLUEZ_SERVICE = "org.bluez"
OBJECT_MANAGER_IFACE = "org.freedesktop.DBus.ObjectManager"
PROPERTIES_IFACE = "org.freedesktop.DBus.Properties"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"
AGENT_IFACE = "org.bluez.Agent1"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
AGENT_PATH = "/com/openmower/BluetoothGamepadAgent"
PAIRING_AUTHORIZATION_SECONDS = 75.0

SUPPORTED_PROFILES = [
    "ps3",
    "shield",
    "steam_stick",
    "steam_touch",
    "switch_pro",
    "xbox360",
]


def dbus_bool(value):
    return dbus.Boolean(bool(value), variant_level=1)


class PairingRejected(dbus.DBusException if dbus else Exception):
    _dbus_error_name = "org.bluez.Error.Rejected"


if dbus is not None:
    class BluetoothPairingAgent(dbus.service.Object):
        def __init__(self, bus, manager):
            super().__init__(bus, AGENT_PATH)
            self.manager = manager

        def authorize(self, device):
            if not self.manager.agent_allows(str(device)):
                raise PairingRejected("Pairing was not requested from the WebUI")

        @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
        def Release(self):
            rospy.loginfo("Bluetooth pairing agent released by BlueZ")

        @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="s")
        def RequestPinCode(self, device):
            self.authorize(device)
            return "0000"

        @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="u")
        def RequestPasskey(self, device):
            self.authorize(device)
            return dbus.UInt32(0)

        @dbus.service.method(AGENT_IFACE, in_signature="ouq", out_signature="")
        def DisplayPasskey(self, device, _passkey, _entered):
            self.authorize(device)

        @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
        def DisplayPinCode(self, device, _pincode):
            self.authorize(device)

        @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
        def RequestConfirmation(self, device, _passkey):
            self.authorize(device)

        @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="")
        def RequestAuthorization(self, device):
            self.authorize(device)

        @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
        def AuthorizeService(self, device, _uuid):
            self.authorize(device)

        @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
        def Cancel(self):
            rospy.loginfo("Bluetooth pairing agent request canceled")
else:
    BluetoothPairingAgent = None


class BluetoothGamepadManager:
    def __init__(self):
        self.profile_param = rospy.get_param("~profile_param", "/direct_gamepad/profile")
        self.scan_requested = False
        self.last_error = ""
        self.bus = None
        self.agent = None
        self.agent_loop = None
        self.agent_allowed_path = ""
        self.agent_allowed_until = rospy.Time(0)
        self.status_pub = rospy.Publisher("/bluetooth_gamepad/status", String, queue_size=1, latch=True)

        rospy.Service("/bluetooth_gamepad/set_powered", SetBool, self.set_powered)
        rospy.Service("/bluetooth_gamepad/set_scan_enabled", SetBool, self.set_scan_enabled)
        rospy.Service("/bluetooth_gamepad/pair", BluetoothDeviceCommand, self.pair_device)
        rospy.Service("/bluetooth_gamepad/connect", BluetoothDeviceCommand, self.connect_device)
        rospy.Service("/bluetooth_gamepad/disconnect", BluetoothDeviceCommand, self.disconnect_device)
        rospy.Service("/bluetooth_gamepad/forget", BluetoothDeviceCommand, self.forget_device)

        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        self.publish_status()

    def system_bus(self):
        if dbus is None:
            raise RuntimeError("python3-dbus is not installed in the runtime container")
        if self.bus is None:
            if DBusGMainLoop is not None:
                DBusGMainLoop(set_as_default=True)
            self.bus = dbus.SystemBus()
        self.ensure_agent()
        return self.bus

    def ensure_agent(self):
        if dbus is None or BluetoothPairingAgent is None or GLib is None:
            raise RuntimeError("python3-dbus and python3-gi are required for Bluetooth pairing")

        if self.agent is None:
            self.agent = BluetoothPairingAgent(self.bus, self)
            self.agent_loop = GLib.MainLoop()
            threading.Thread(target=self.agent_loop.run, name="bluetooth-pairing-agent", daemon=True).start()

        agent_manager = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE, "/org/bluez"), AGENT_MANAGER_IFACE)
        try:
            agent_manager.RegisterAgent(dbus.ObjectPath(AGENT_PATH), "NoInputNoOutput")
        except Exception as exc:
            if "AlreadyExists" not in str(exc):
                raise
        agent_manager.RequestDefaultAgent(dbus.ObjectPath(AGENT_PATH))

    def authorize_pairing_path(self, path):
        self.agent_allowed_path = str(path)
        self.agent_allowed_until = rospy.Time.now() + rospy.Duration(PAIRING_AUTHORIZATION_SECONDS)

    def agent_allows(self, path):
        if path == self.agent_allowed_path and rospy.Time.now() <= self.agent_allowed_until:
            return True
        try:
            props = self.properties(path)
            return bool(props.Get(DEVICE_IFACE, "Paired")) or bool(props.Get(DEVICE_IFACE, "Trusted"))
        except Exception:
            return False

    def managed_objects(self):
        bus = self.system_bus()
        manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE, "/"), OBJECT_MANAGER_IFACE)
        return manager.GetManagedObjects()

    def adapter_path(self, objects=None):
        objects = objects if objects is not None else self.managed_objects()
        for path, interfaces in objects.items():
            if ADAPTER_IFACE in interfaces:
                return str(path)
        return None

    def adapter(self, objects=None):
        path = self.adapter_path(objects)
        if not path:
            raise RuntimeError("No Bluetooth adapter found")
        return path, dbus.Interface(self.system_bus().get_object(BLUEZ_SERVICE, path), ADAPTER_IFACE)

    def properties(self, path):
        return dbus.Interface(self.system_bus().get_object(BLUEZ_SERVICE, path), PROPERTIES_IFACE)

    def device_path_by_address(self, address, objects=None):
        normalized = address.strip().upper()
        objects = objects if objects is not None else self.managed_objects()
        for path, interfaces in objects.items():
            props = interfaces.get(DEVICE_IFACE)
            if props and str(props.get("Address", "")).upper() == normalized:
                return str(path)
        return None

    def sanitize_profile(self, controller_type):
        profile = controller_type.strip() if controller_type else rospy.get_param(self.profile_param, "xbox360")
        if profile not in SUPPORTED_PROFILES:
            return "xbox360"
        return profile

    def set_controller_profile(self, controller_type):
        profile = self.sanitize_profile(controller_type)
        rospy.set_param(self.profile_param, profile)
        return profile

    @staticmethod
    def device_payload(path, props):
        return {
            "path": str(path),
            "address": str(props.get("Address", "")),
            "name": str(props.get("Name", props.get("Alias", "Unknown device"))),
            "alias": str(props.get("Alias", props.get("Name", "Unknown device"))),
            "paired": bool(props.get("Paired", False)),
            "trusted": bool(props.get("Trusted", False)),
            "connected": bool(props.get("Connected", False)),
            "services_resolved": bool(props.get("ServicesResolved", False)),
            "rssi": int(props["RSSI"]) if "RSSI" in props else None,
            "icon": str(props.get("Icon", "")),
        }

    def set_powered(self, req):
        try:
            path, _adapter = self.adapter()
            self.properties(path).Set(ADAPTER_IFACE, "Powered", dbus_bool(req.data))
            self.last_error = ""
            self.publish_status()
            return SetBoolResponse(success=True, message="Bluetooth powered {}".format("on" if req.data else "off"))
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return SetBoolResponse(success=False, message=str(exc))

    def set_scan_enabled(self, req):
        try:
            path, adapter = self.adapter()
            props = self.properties(path)
            if req.data:
                props.Set(ADAPTER_IFACE, "Powered", dbus_bool(True))
                adapter.StartDiscovery()
                self.scan_requested = True
                message = "Bluetooth scan started"
            else:
                try:
                    adapter.StopDiscovery()
                except Exception as exc:
                    rospy.logwarn("StopDiscovery failed: %s", exc)
                self.scan_requested = False
                message = "Bluetooth scan stopped"
            self.last_error = ""
            self.publish_status()
            return SetBoolResponse(success=True, message=message)
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return SetBoolResponse(success=False, message=str(exc))

    def pair_device(self, req):
        try:
            profile = self.set_controller_profile(req.controller_type)
            path = self.device_path_by_address(req.address)
            if not path:
                return BluetoothDeviceCommandResponse(False, "Device is not visible; start scan and try again")
            self.authorize_pairing_path(path)
            device = dbus.Interface(self.system_bus().get_object(BLUEZ_SERVICE, path), DEVICE_IFACE)
            props = self.properties(path)
            if not bool(props.Get(DEVICE_IFACE, "Paired")):
                device.Pair(timeout=30)
            props.Set(DEVICE_IFACE, "Trusted", dbus_bool(True))
            self.last_error = ""
            self.publish_status()
            return BluetoothDeviceCommandResponse(True, "Paired and trusted as {}".format(profile))
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return BluetoothDeviceCommandResponse(False, str(exc))

    def connect_device(self, req):
        try:
            profile = self.set_controller_profile(req.controller_type)
            path = self.device_path_by_address(req.address)
            if not path:
                return BluetoothDeviceCommandResponse(False, "Device is not visible; start scan and try again")
            props = self.properties(path)
            if not bool(props.Get(DEVICE_IFACE, "Paired")):
                return BluetoothDeviceCommandResponse(False, "Pair the controller before connecting")
            props.Set(DEVICE_IFACE, "Trusted", dbus_bool(True))
            if self.scan_requested:
                try:
                    _adapter_path, adapter = self.adapter()
                    adapter.StopDiscovery()
                except Exception as exc:
                    rospy.logwarn("StopDiscovery before Connect failed: %s", exc)
                self.scan_requested = False
            device = dbus.Interface(self.system_bus().get_object(BLUEZ_SERVICE, path), DEVICE_IFACE)
            device.Connect(timeout=15)
            self.last_error = ""
            self.publish_status()
            return BluetoothDeviceCommandResponse(True, "Connected as {}".format(profile))
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return BluetoothDeviceCommandResponse(False, str(exc))

    def disconnect_device(self, req):
        try:
            path = self.device_path_by_address(req.address)
            if not path:
                return BluetoothDeviceCommandResponse(False, "Device is not known")
            device = dbus.Interface(self.system_bus().get_object(BLUEZ_SERVICE, path), DEVICE_IFACE)
            device.Disconnect(timeout=10)
            self.last_error = ""
            self.publish_status()
            return BluetoothDeviceCommandResponse(True, "Disconnected")
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return BluetoothDeviceCommandResponse(False, str(exc))

    def forget_device(self, req):
        try:
            objects = self.managed_objects()
            device_path = self.device_path_by_address(req.address, objects)
            if not device_path:
                return BluetoothDeviceCommandResponse(False, "Device is not known")
            _adapter_path, adapter = self.adapter(objects)
            adapter.RemoveDevice(dbus.ObjectPath(device_path), timeout=10)
            self.last_error = ""
            self.publish_status()
            return BluetoothDeviceCommandResponse(True, "Forgot device")
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status()
            return BluetoothDeviceCommandResponse(False, str(exc))

    def status_payload(self):
        payload = {
            "available": False,
            "message": "",
            "adapter": None,
            "agent_available": self.agent is not None,
            "devices": [],
            "scan_requested": self.scan_requested,
            "controller_type": rospy.get_param(self.profile_param, "xbox360"),
            "supported_profiles": SUPPORTED_PROFILES,
        }
        if dbus is None:
            payload["message"] = "python3-dbus is not installed in the runtime container"
            return payload

        try:
            objects = self.managed_objects()
            payload["agent_available"] = self.agent is not None
            adapter_path = self.adapter_path(objects)
            if not adapter_path:
                payload["message"] = "No Bluetooth adapter found"
                return payload
            adapter_props = objects[adapter_path][ADAPTER_IFACE]
            payload["available"] = True
            payload["adapter"] = {
                "path": adapter_path,
                "address": str(adapter_props.get("Address", "")),
                "name": str(adapter_props.get("Name", "Bluetooth adapter")),
                "alias": str(adapter_props.get("Alias", "Bluetooth adapter")),
                "powered": bool(adapter_props.get("Powered", False)),
                "discovering": bool(adapter_props.get("Discovering", False)),
                "pairable": bool(adapter_props.get("Pairable", False)),
            }
            payload["devices"] = sorted(
                [
                    self.device_payload(path, interfaces[DEVICE_IFACE])
                    for path, interfaces in objects.items()
                    if DEVICE_IFACE in interfaces
                ],
                key=lambda item: (not item["connected"], not item["paired"], item["name"].lower(), item["address"]),
            )
            payload["message"] = self.last_error
            return payload
        except Exception as exc:
            self.last_error = str(exc)
            payload["message"] = str(exc)
            return payload

    def publish_status(self, _event=None):
        self.status_pub.publish(String(data=json.dumps(self.status_payload(), sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("bluetooth_gamepad_manager")
    BluetoothGamepadManager()
    rospy.spin()
