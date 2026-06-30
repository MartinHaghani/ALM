import { useCallback, useEffect, useRef, useState } from "react";
import { Ros, Topic } from "roslib";

import type { Twist } from "./types";

const START_MANUAL_MOWING_ACTION = "mower_logic:area_recording/start_manual_mowing";
const STOP_MANUAL_MOWING_ACTION = "mower_logic:area_recording/stop_manual_mowing";
const BLADE_RELEASE_GRACE_MS = 200;
const DRIVE_INTERVAL_MS = 20;
const GAMEPAD_DEADZONE = 0.14;
const GAMEPAD_ANGULAR_SCALE = 1.6;
const ZERO_COMMAND: JoystickCommand = { x: 0, z: 0 };

type RemoteInputSource = "gamepad" | "none" | "touch";

export interface JoystickCommand {
  x: number;
  z: number;
}

export interface GamepadControlState {
  angular: number;
  bladeComboPressed: boolean;
  connected: boolean;
  deviceName: string;
  driving: boolean;
  linear: number;
  mapping: string;
  pressedButtons: number[];
}

interface UseAreaRecordingControlOptions {
  active: boolean;
  connected: boolean;
  joyTopic: string;
  publishAction: (action: string, source?: string) => void;
  ros: Ros | null;
}

interface UseAreaRecordingControlResult {
  bladeHoldActive: boolean;
  clearTouchCommand: () => void;
  gamepad: GamepadControlState;
  joystickCommand: JoystickCommand;
  publishZeroNow: () => void;
  setTouchBladePressed: (pressed: boolean) => void;
  touchBladePressed: boolean;
  updateTouchCommand: (command: JoystickCommand) => void;
}

const disconnectedGamepad: GamepadControlState = {
  angular: 0,
  bladeComboPressed: false,
  connected: false,
  deviceName: "",
  driving: false,
  linear: 0,
  mapping: "",
  pressedButtons: [],
};

function hasDriveCommand(command: JoystickCommand): boolean {
  return Math.abs(command.x) > 0.01 || Math.abs(command.z) > 0.01;
}

function twistFromCommand(command: JoystickCommand): Twist {
  return {
    angular: { x: 0, y: 0, z: command.z },
    linear: { x: command.x, y: 0, z: 0 },
  };
}

function normalizeAxis(value: number): number {
  if (Math.abs(value) < GAMEPAD_DEADZONE) {
    return 0;
  }
  const normalized = (Math.abs(value) - GAMEPAD_DEADZONE) / (1 - GAMEPAD_DEADZONE);
  const clamped = Math.max(0, Math.min(1, normalized));
  return value < 0 ? -clamped : clamped;
}

function firstConnectedGamepad(): Gamepad | null {
  for (const gamepad of navigator.getGamepads()) {
    if (gamepad?.connected) {
      return gamepad;
    }
  }
  return null;
}

function gamepadAxis(gamepad: Gamepad, index: number): number {
  return index < gamepad.axes.length ? Number(gamepad.axes[index]) : 0;
}

function gamepadButtonPressed(gamepad: Gamepad, index: number): boolean {
  if (index >= gamepad.buttons.length) {
    return false;
  }
  const button = gamepad.buttons[index];
  return button.pressed || button.value > 0.5;
}

function pressedButtons(gamepad: Gamepad): number[] {
  const pressed: number[] = [];
  for (let index = 0; index < gamepad.buttons.length; index += 1) {
    if (gamepadButtonPressed(gamepad, index)) {
      pressed.push(index);
    }
  }
  return pressed;
}

function readGamepad(): GamepadControlState {
  if (document.hidden) {
    return disconnectedGamepad;
  }

  const gamepad = firstConnectedGamepad();
  if (!gamepad) {
    return disconnectedGamepad;
  }

  const linear = normalizeAxis(-gamepadAxis(gamepad, 1));
  const angular = normalizeAxis(-gamepadAxis(gamepad, 0)) * GAMEPAD_ANGULAR_SCALE;
  const leftShoulderPressed = gamepadButtonPressed(gamepad, 4);
  const rightShoulderPressed = gamepadButtonPressed(gamepad, 5);

  return {
    angular,
    bladeComboPressed: leftShoulderPressed && rightShoulderPressed,
    connected: true,
    deviceName: gamepad.id.trim() || "Gamepad",
    driving: hasDriveCommand({ x: linear, z: angular }),
    linear,
    mapping: gamepad.mapping.trim() || "unknown",
    pressedButtons: pressedButtons(gamepad),
  };
}

function sameGamepadState(a: GamepadControlState, b: GamepadControlState): boolean {
  return (
    a.connected === b.connected &&
    a.bladeComboPressed === b.bladeComboPressed &&
    a.deviceName === b.deviceName &&
    a.mapping === b.mapping &&
    Math.abs(a.linear - b.linear) < 0.001 &&
    Math.abs(a.angular - b.angular) < 0.001 &&
    a.pressedButtons.join(",") === b.pressedButtons.join(",")
  );
}

export function useAreaRecordingControl({
  active,
  connected,
  joyTopic,
  publishAction,
  ros,
}: UseAreaRecordingControlOptions): UseAreaRecordingControlResult {
  const [joystickCommand, setJoystickCommand] = useState<JoystickCommand>(ZERO_COMMAND);
  const [gamepad, setGamepad] = useState<GamepadControlState>(disconnectedGamepad);
  const [bladeHoldActive, setBladeHoldActive] = useState(false);
  const [touchBladePressed, setTouchBladePressedState] = useState(false);

  const activeInputSourceRef = useRef<RemoteInputSource>("none");
  const activeRef = useRef(active);
  const bladeHoldCommandedRef = useRef(false);
  const commandRef = useRef<JoystickCommand>(ZERO_COMMAND);
  const gamepadBladeHoldPressedRef = useRef(false);
  const gamepadRef = useRef<GamepadControlState>(disconnectedGamepad);
  const lastDriveNonZeroRef = useRef(false);
  const lastGamepadBladePressRef = useRef<number | null>(null);
  const publishActionRef = useRef(publishAction);
  const rosConnectedRef = useRef(connected);
  const touchBladePressedRef = useRef(false);
  const topicRef = useRef<Topic<Twist> | null>(null);

  useEffect(() => {
    publishActionRef.current = publishAction;
  }, [publishAction]);

  useEffect(() => {
    rosConnectedRef.current = connected;
  }, [connected]);

  const publishDrive = useCallback((command: JoystickCommand) => {
    const topic = topicRef.current;
    if (!topic || !rosConnectedRef.current) {
      return;
    }
    topic.publish(twistFromCommand(command));
  }, []);

  const publishZeroNow = useCallback(() => {
    publishDrive(ZERO_COMMAND);
    lastDriveNonZeroRef.current = false;
  }, [publishDrive]);

  const setCommand = useCallback((command: JoystickCommand, source: RemoteInputSource) => {
    commandRef.current = command;
    activeInputSourceRef.current = source;
    setJoystickCommand(command);
  }, []);

  const releaseBladeHold = useCallback((force = false) => {
    const shouldStop = bladeHoldCommandedRef.current || force;
    if (!shouldStop) {
      setBladeHoldActive(false);
      return;
    }

    publishActionRef.current(STOP_MANUAL_MOWING_ACTION, force ? "blade_hold_safety_release" : "blade_hold_release");
    bladeHoldCommandedRef.current = false;
    setBladeHoldActive(false);
  }, []);

  const syncBladeHold = useCallback(() => {
    if (!activeRef.current) {
      releaseBladeHold(true);
      return;
    }

    if (touchBladePressedRef.current || gamepadBladeHoldPressedRef.current) {
      if (!bladeHoldCommandedRef.current) {
        publishActionRef.current(
          START_MANUAL_MOWING_ACTION,
          gamepadBladeHoldPressedRef.current ? "gamepad_blade_hold" : "touch_blade_hold",
        );
        bladeHoldCommandedRef.current = true;
        setBladeHoldActive(true);
      }
      return;
    }

    releaseBladeHold();
  }, [releaseBladeHold]);

  useEffect(() => {
    if (!ros) {
      topicRef.current = null;
      return undefined;
    }

    const topic = new Topic<Twist>({
      messageType: "geometry_msgs/Twist",
      name: joyTopic,
      queue_length: 1,
      ros,
    });
    topic.advertise();
    topicRef.current = topic;

    return () => {
      publishZeroNow();
      topic.unadvertise();
      if (topicRef.current === topic) {
        topicRef.current = null;
      }
    };
  }, [joyTopic, publishZeroNow, ros]);

  useEffect(() => {
    activeRef.current = active;
    if (!active) {
      setCommand(ZERO_COMMAND, "none");
      gamepadBladeHoldPressedRef.current = false;
      lastGamepadBladePressRef.current = null;
      touchBladePressedRef.current = false;
      setTouchBladePressedState(false);
      setGamepad(disconnectedGamepad);
      gamepadRef.current = disconnectedGamepad;
      publishZeroNow();
      releaseBladeHold(true);
    }
  }, [active, publishZeroNow, releaseBladeHold, setCommand]);

  useEffect(() => {
    if (!active || !ros) {
      return undefined;
    }

    const driveTimer = window.setInterval(() => {
      const command = commandRef.current;
      if (hasDriveCommand(command)) {
        publishDrive(command);
        lastDriveNonZeroRef.current = true;
      } else if (lastDriveNonZeroRef.current) {
        publishZeroNow();
      }
    }, DRIVE_INTERVAL_MS);

    return () => {
      window.clearInterval(driveTimer);
      publishZeroNow();
    };
  }, [active, publishDrive, publishZeroNow, ros]);

  useEffect(() => {
    if (!active) {
      return undefined;
    }

    function emitDisconnected(): void {
      const nextState = disconnectedGamepad;
      if (!sameGamepadState(gamepadRef.current, nextState)) {
        gamepadRef.current = nextState;
        setGamepad(nextState);
      }
      gamepadBladeHoldPressedRef.current = false;
      lastGamepadBladePressRef.current = null;
      if (activeInputSourceRef.current === "gamepad") {
        setCommand(ZERO_COMMAND, "none");
        publishZeroNow();
      }
      syncBladeHold();
    }

    const pollTimer = window.setInterval(() => {
      const nextState = readGamepad();
      if (!sameGamepadState(gamepadRef.current, nextState)) {
        gamepadRef.current = nextState;
        setGamepad(nextState);
      }

      if (!nextState.connected) {
        emitDisconnected();
        return;
      }

      if (nextState.bladeComboPressed) {
        lastGamepadBladePressRef.current = Date.now();
        gamepadBladeHoldPressedRef.current = true;
      } else {
        const lastSeen = lastGamepadBladePressRef.current;
        gamepadBladeHoldPressedRef.current =
          lastSeen !== null && Date.now() - lastSeen < BLADE_RELEASE_GRACE_MS;
      }
      syncBladeHold();

      setCommand({ x: nextState.linear, z: nextState.angular }, "gamepad");
    }, DRIVE_INTERVAL_MS);

    window.addEventListener("blur", emitDisconnected);
    document.addEventListener("visibilitychange", emitDisconnected);

    return () => {
      window.clearInterval(pollTimer);
      window.removeEventListener("blur", emitDisconnected);
      document.removeEventListener("visibilitychange", emitDisconnected);
      emitDisconnected();
    };
  }, [active, publishZeroNow, setCommand, syncBladeHold]);

  useEffect(() => {
    const bladeTimer = window.setInterval(syncBladeHold, 100);
    return () => window.clearInterval(bladeTimer);
  }, [syncBladeHold]);

  const updateTouchCommand = useCallback(
    (command: JoystickCommand) => {
      setCommand(command, "touch");
    },
    [setCommand],
  );

  const clearTouchCommand = useCallback(() => {
    if (activeInputSourceRef.current === "touch") {
      setCommand(ZERO_COMMAND, "none");
      publishZeroNow();
    }
  }, [publishZeroNow, setCommand]);

  const setTouchBladePressed = useCallback(
    (pressed: boolean) => {
      touchBladePressedRef.current = pressed;
      setTouchBladePressedState(pressed);
      syncBladeHold();
    },
    [syncBladeHold],
  );

  return {
    bladeHoldActive,
    clearTouchCommand,
    gamepad,
    joystickCommand,
    publishZeroNow,
    setTouchBladePressed,
    touchBladePressed,
    updateTouchCommand,
  };
}
