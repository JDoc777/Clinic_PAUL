import time
import threading

import BT_control
import Payload
import Melodies


LOOP_DELAY = 0.05

MAX_MOTOR_CMD = 80
TRIGGER_THRESHOLD = 0.5


class BTStateMachine:
    def __init__(self, shared_data, running_event, arm_controller=None):
        self.shared_data = shared_data
        self.running_event = running_event
        self.arm_controller = arm_controller

        self.controller_mode = False
        self.mode = "NORMAL"   # NORMAL, DRIVE, ARM

        self.last_buttons = {}

    def pressed_once(self, state, name):
        now = state.get(name, False)
        before = self.last_buttons.get(name, False)
        return now and not before

    def update_last_buttons(self, state):
        for key, value in state.items():
            if isinstance(value, bool):
                self.last_buttons[key] = value

    def stop_drive(self):
        Payload.set_motors(self.shared_data, 0, 0, 0, 0)

    # -----------------------------
    # DRIVE MODE
    # -----------------------------
    def drive_mode(self, state):
        vx = state["right_stick_y"]
        vy = state["right_stick_x"]
        omega = state["left_stick_x"]

        # diagonal overrides
        if state["right_bumper"]:
            vx = 1.0
            vy = 1.0
            omega = 0.0

        elif state["left_bumper"]:
            vx = 1.0
            vy = -1.0
            omega = 0.0

        elif state["right_trigger"] > TRIGGER_THRESHOLD:
            vx = -1.0
            vy = 1.0
            omega = 0.0

        elif state["left_trigger"] > TRIGGER_THRESHOLD:
            vx = -1.0
            vy = -1.0
            omega = 0.0

        fl = vx + vy + omega
        fr = vx - vy - omega
        rl = vx - vy + omega
        rr = vx + vy - omega

        max_mag = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)

        FL = int((fl / max_mag) * MAX_MOTOR_CMD)
        FR = int((fr / max_mag) * MAX_MOTOR_CMD)
        RL = int((rl / max_mag) * MAX_MOTOR_CMD)
        RR = int((rr / max_mag) * MAX_MOTOR_CMD)

        Payload.set_motors(self.shared_data, FL, FR, RL, RR)

    # -----------------------------
    # ARM MODE
    # -----------------------------
    def arm_mode(self, state):
        # Read current servo values if available
        params = self.shared_data.get_command_params()
        current_servos = params.get("servos", (90, 90, 90, 90, 90))

        yaw = current_servos[0]
        shoulder = current_servos[1]
        elbow = current_servos[2]
        wrist = current_servos[3]
        claw = current_servos[4]

        SERVO_STEP = 2
        CLAW_STEP = 3

        # Joystick control
        yaw += int(state["left_stick_x"] * SERVO_STEP)
        shoulder += int(state["left_stick_y"] * SERVO_STEP)
        elbow += int(state["right_stick_y"] * SERVO_STEP)
        wrist += int(state["right_stick_x"] * SERVO_STEP)

        # Trigger control
        if state["right_trigger"] > TRIGGER_THRESHOLD:
            claw += CLAW_STEP      # close

        if state["left_trigger"] > TRIGGER_THRESHOLD:
            claw -= CLAW_STEP      # open

        # Clamp 0–180
        yaw = max(0, min(180, yaw))
        shoulder = max(0, min(180, shoulder))
        elbow = max(0, min(180, elbow))
        wrist = max(0, min(180, wrist))
        claw = max(0, min(180, claw))

        Payload.set_servos(
            self.shared_data,
            yaw,
            shoulder,
            elbow,
            wrist,
            claw
        )

        print(
            f"[ARM] yaw={yaw}, shoulder={shoulder}, "
            f"elbow={elbow}, wrist={wrist}, claw={claw}"
        )

    # -----------------------------
    # BUZZER / DPAD
    # -----------------------------
    def dpad_songs(self, state):
        x = state["dpad_x"]
        y = state["dpad_y"]

        if x == 0 and y == 1:
            print("[BT] D-pad up: play song X")
            Melodies.play("mario")

        elif x == 0 and y == -1:
            print("[BT] D-pad down: play song Y")
            Melodies.play("birthday")

        elif x == -1 and y == 0:
            print("[BT] D-pad left: play song Z")
            Melodies.play("coin")

        elif x == 1 and y == 0:
            print("[BT] D-pad right: play song ZED")
            Melodies.play("game_over")

    # -----------------------------
    # MAIN UPDATE
    # -----------------------------
    def update(self):
        state = BT_control.get_controller_state()

        # START enters controller mode from anywhere
        if self.pressed_once(state, "start"):
            self.controller_mode = True
            self.mode = "CONTROLLER_IDLE"
            self.stop_drive()
            print("[BT] START pressed: controller mode ON")

        # BACK exits controller mode from anywhere
        if self.pressed_once(state, "back"):
            self.controller_mode = False
            self.mode = "NORMAL"
            self.stop_drive()
            print("[BT] BACK pressed: controller mode OFF")

        # D-pad songs can work whenever controller is connected
        if state["dpad_x"] != 0 or state["dpad_y"] != 0:
            self.dpad_songs(state)

        if not self.controller_mode:
            # Normal PAUL running. Do nothing here.
            self.update_last_buttons(state)
            return

        # In controller mode, A/B pick sub-modes
        if self.pressed_once(state, "button_a"):
            self.mode = "DRIVE"
            print("[BT] A pressed: DRIVE mode")

        if self.pressed_once(state, "button_b"):
            self.mode = "ARM"
            self.stop_drive()
            print("[BT] B pressed: ARM mode")

        if self.mode == "CONTROLLER_IDLE":
            self.stop_drive()

        elif self.mode == "DRIVE":
            self.drive_mode(state)

        elif self.mode == "ARM":
            self.stop_drive()
            self.arm_mode(state)

        self.update_last_buttons(state)

    def loop(self):
        print("[BT] State machine started.")

        while self.running_event.is_set():
            self.update()
            time.sleep(LOOP_DELAY)

        self.stop_drive()
        print("[BT] State machine stopped.")


def create_and_run(shared_data, running_event, arm_controller=None):
    sm = BTStateMachine(shared_data, running_event, arm_controller)

    t = threading.Thread(
        target=sm.loop,
        daemon=True
    )

    t.start()
    return sm, t