import time
import threading

import BT_control
import Payload
import Melodies


LOOP_DELAY = 0.005

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

        self.last_dpad = (0, 0)

        self.servos = [120.0, 0.0, 156.0, 73.0, 0.0]  # <-- ADD THIS


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

        yaw = self.servos[0]
        shoulder = self.servos[1]
        elbow = self.servos[2]
        wrist = self.servos[3]
        claw = self.servos[4]
        

        DEADZONE = 0.08

        MAX_SERVO_STEP = 0.3   # bigger = faster arm movement
        MAX_CLAW_STEP = 0.5    # bigger = faster claw movement

        def scaled_axis(value, max_step):
            if abs(value) < DEADZONE:
                return 0.0
            return (value) * max_step

        # joystick scaled movement
        yaw += scaled_axis(state["left_stick_x"], MAX_SERVO_STEP)
        shoulder += scaled_axis(state["left_stick_y"], MAX_SERVO_STEP)
        elbow += scaled_axis(state["right_stick_y"], MAX_SERVO_STEP)
        wrist += scaled_axis(state["right_stick_x"], MAX_SERVO_STEP)

        # trigger scaled movement
        claw += state["right_trigger"] * MAX_CLAW_STEP
        claw -= state["left_trigger"] * MAX_CLAW_STEP

        yaw = max(0, min(180, yaw))
        shoulder = max(0, min(180, shoulder))
        elbow = max(0, min(180, elbow))
        wrist = max(0, min(180, wrist))
        claw = max(0, min(180, claw))
        self.servos = [yaw, shoulder, elbow, wrist, claw]  # <-- save back

        Payload.set_servos(
            self.shared_data,
            int(round(yaw)),
            int(round(shoulder)),
            int(round(elbow)),
            int(round(wrist)),
            int(round(claw))
        )

        #print(
            #f"[ARM] yaw={yaw:.1f}, shoulder={shoulder:.1f}, "
            #f"elbow={elbow:.1f}, wrist={wrist:.1f}, claw={claw:.1f}"
        #)

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
            Melodies.play("power_up")
            print("[BT] START pressed: controller mode ON")

        # BACK exits controller mode from anywhere
        if self.pressed_once(state, "back"):
            self.controller_mode = False
            self.mode = "NORMAL"
            self.stop_drive()
            Melodies.play("power_down")
            print("[BT] BACK pressed: controller mode OFF")

        # D-pad songs can work whenever controller is connected
        current_dpad = (state["dpad_x"], state["dpad_y"])

        if current_dpad != (0, 0) and current_dpad != self.last_dpad:
            self.dpad_songs(state)

        self.last_dpad = current_dpad

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