#!/usr/bin/env python3

import socket

import rospy
from std_msgs.msg import Header
from rtcm_msgs.msg import Message

from ntrip_client.rtcm_parser import RTCMParser


class RTCMTCPBridge:
    def __init__(self):
        rospy.init_node("rtcm_tcp_bridge")

        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 5016))
        self.buffer_size = int(rospy.get_param("~buffer_size", 4096))
        self.socket_timeout_seconds = float(rospy.get_param("~socket_timeout_seconds", 5.0))
        self.reconnect_wait_seconds = float(rospy.get_param("~reconnect_wait_seconds", 5.0))
        self.frame_id = rospy.get_param("~frame_id", "odom")

        self.publisher = rospy.Publisher("rtcm", Message, queue_size=10)
        self.parser = RTCMParser(
            logerr=rospy.logerr,
            logwarn=rospy.logwarn,
            loginfo=rospy.loginfo,
            logdebug=rospy.logdebug,
        )

    def run(self):
        while not rospy.is_shutdown():
            stream = None
            try:
                rospy.loginfo("Connecting to RTCM TCP source %s:%s", self.host, self.port)
                stream = socket.create_connection((self.host, self.port), timeout=self.socket_timeout_seconds)
                stream.settimeout(self.socket_timeout_seconds)
                rospy.loginfo("Connected to RTCM TCP source %s:%s", self.host, self.port)

                while not rospy.is_shutdown():
                    try:
                        payload = stream.recv(self.buffer_size)
                    except socket.timeout:
                        continue

                    if not payload:
                        raise ConnectionError("RTCM TCP source closed the connection")

                    for packet in self.parser.parse(payload):
                        self.publisher.publish(
                            Message(
                                header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
                                message=packet,
                            )
                        )
            except Exception as exc:
                if rospy.is_shutdown():
                    break
                rospy.logwarn(
                    "RTCM TCP bridge disconnected from %s:%s: %s. Reconnecting in %.1f seconds.",
                    self.host,
                    self.port,
                    exc,
                    self.reconnect_wait_seconds,
                )
                rospy.sleep(self.reconnect_wait_seconds)
            finally:
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass


if __name__ == "__main__":
    RTCMTCPBridge().run()
