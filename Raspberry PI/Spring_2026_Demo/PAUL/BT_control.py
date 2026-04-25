import pygame
import time
import threading

# ============================
# SETTINGS
# ============================
DEADZONE = 0.12
CONTROLLER_LOOP_DELAY = 0.01
MAIN_LOOP_DELAY = 0.10

controller_running = True
state_lock = threading.Lock()


# ============================
# CHANGE THESE IF MAPPING IS FLIPPED
# ============================
INVERT_LEFT_X = False
INVERT_LEFT_Y = True
INVERT_RIGHT_X = False
INVERT_RIGHT_Y = True
INVERT_LEFT_TRIGGER = False
INVERT_RIGHT_TRIGGER = False


# ============================
# SHARED STATE
# ============================
controller_state = {
    # named axes
    "left_stick_x": 0.0,
    "left_stick_y": 0.0,
    "right_stick_x": 0.0,
    "right_stick_y": 0.0,
    "left_trigger": 0.0,
    "right_trigger": 0.0,

    # raw axes
    "axis_0": 0.0,
    "axis_1": 0.0,
    "axis_2": 0.0,
    "axis_3": 0.0,
    "axis_4": 0.0,
    "axis_5": 0.0,
    "axis_6": 0.0,
    "axis_7": 0.0,

    # named buttons
    "button_a": False,
    "button_b": False,
    "button_x": False,
    "button_y": False,
    "left_bumper": False,
    "right_bumper": False,
    "back": False,
    "start": False,
    "left_stick_button": False,
    "right_stick_button": False,
    "home": False,

    # raw buttons
    "button_0": False,
    "button_1": False,
    "button_2": False,
    "button_3": False,
    "button_4": False,
    "button_5": False,
    "button_6": False,
    "button_7": False,
    "button_8": False,
    "button_9": False,
    "button_10": False,
    "button_11": False,
    "button_12": False,
    "button_13": False,
    "button_14": False,
    "button_15": False,

    # dpad / hat
    "dpad_x": 0,
    "dpad_y": 0,
}


# ============================
# HELPER FUNCTIONS
# ============================
def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return round(value, 3)


def invert_if_needed(value, invert):
    return -value if invert else value


def set_state(key, value):
    with state_lock:
        controller_state[key] = value


def get_controller_state():
    with state_lock:
        return controller_state.copy()


# ============================
# AXIS HANDLER
# ============================
def handle_axis(axis_num, value):
    value = apply_deadzone(value)

    raw_key = f"axis_{axis_num}"
    set_state(raw_key, value)

    #print(f"RAW AXIS {axis_num}: {value}")

    match axis_num:
        case 0:
            set_state("left_stick_x", invert_if_needed(value, INVERT_LEFT_X))

        case 1:
            set_state("left_stick_y", invert_if_needed(value, INVERT_LEFT_Y))

        case 2:
            # LEFT TRIGGER
            norm = round((value + 1) / 2, 3)
            set_state("left_trigger", norm)

        case 3:
            # RIGHT STICK X
            set_state("right_stick_x", invert_if_needed(value, INVERT_RIGHT_X))

        case 4:
            # RIGHT STICK Y
            set_state("right_stick_y", invert_if_needed(value, INVERT_RIGHT_Y))

        case 5:
            # RIGHT TRIGGER
            norm = round((value + 1) / 2, 3)
            set_state("right_trigger", norm)

        case _:
            print(f"Unknown axis {axis_num}: {value}")

# ============================
# BUTTON HANDLER
# ============================
def handle_button(button_num, pressed):
    raw_key = f"button_{button_num}"
    set_state(raw_key, pressed)

    print(f"RAW BUTTON {button_num}: {pressed}")

    match button_num:
        case 0:
            set_state("button_a", pressed)

        case 1:
            set_state("button_b", pressed)

        case 2:
            set_state("button_x", pressed)

        case 3:
            set_state("button_y", pressed)

        case 4:
            set_state("left_bumper", pressed)

        case 5:
            set_state("right_bumper", pressed)

        case 6:
            set_state("back", pressed)

        case 7:
            set_state("start", pressed)

        case 8:
            set_state("back", pressed)

        case 9:
            set_state("start", pressed)

        case 10:
            set_state("home", pressed)

        case 11:
            print("left_stick_button", pressed)

        case 12:
            print("left_stick_button", pressed)

        case 13:
            print("Button 13:", pressed)

        case 14:
            print("Button 14:", pressed)

        case 15:
            print("Button 15:", pressed)

        case _:
            print(f"Unknown button {button_num}: {pressed}")


# ============================
# DPAD / HAT HANDLER
# ============================
def handle_hat(hat_num, value):
    x, y = value

    set_state("dpad_x", x)
    set_state("dpad_y", y)

    print(f"RAW HAT {hat_num}: x={x}, y={y}")

    match (x, y):
        case (0, 1):
            print("D-Pad Up")

        case (0, -1):
            print("D-Pad Down")

        case (-1, 0):
            print("D-Pad Left")

        case (1, 0):
            print("D-Pad Right")

        case (0, 0):
            print("D-Pad Released")

        case _:
            print("D-Pad diagonal or unknown:", value)


# ============================
# CONTROLLER THREAD
# ============================
def controller_thread():
    global controller_running

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller found.")
        controller_running = False
        return

    controller = pygame.joystick.Joystick(0)
    controller.init()

    print("Controller connected!")
    print("Name:", controller.get_name())
    print("Axes:", controller.get_numaxes())
    print("Buttons:", controller.get_numbuttons())
    print("Hats:", controller.get_numhats())
    print("--------------------------------")

    while controller_running:
        for event in pygame.event.get():

            if event.type == pygame.JOYAXISMOTION:
                handle_axis(event.axis, event.value)

            elif event.type == pygame.JOYBUTTONDOWN:
                handle_button(event.button, True)

            elif event.type == pygame.JOYBUTTONUP:
                handle_button(event.button, False)

            elif event.type == pygame.JOYHATMOTION:
                handle_hat(event.hat, event.value)

        time.sleep(CONTROLLER_LOOP_DELAY)

    pygame.quit()

def create_and_run(shared_data=None, running_event=None):
    t = threading.Thread(target=controller_thread, daemon=True)
    t.start()
    return t


# ============================
# EXAMPLE MAIN ROBOT LOOP
# Replace this with PAUL code later
# ============================
def robot_main_loop():
    while controller_running:
        state = get_controller_state()

        print(
            f"LX={state['left_stick_x']} | "
            f"LY={state['left_stick_y']} | "
            f"RX={state['right_stick_x']} | "
            f"RY={state['right_stick_y']} | "
            f"LT={state['left_trigger']} | "
            f"RT={state['right_trigger']} | "
            f"A={state['button_a']} | "
            f"B={state['button_b']} | "
            f"X={state['button_x']} | "
            f"Y={state['button_y']} | "
            f"LB={state['left_bumper']} | "
            f"RB={state['right_bumper']} | "
            f"BACK={state['back']} | "
            f"START={state['start']} | "
            f"L3={state['left_stick_button']} | "
            f"R3={state['right_stick_button']} | "
            f"DPAD=({state['dpad_x']},{state['dpad_y']})"
        )

        # Example PAUL mapping later:
        # vx = state["left_stick_y"]
        # vy = state["left_stick_x"]
        # omega = state["right_stick_x"]

        time.sleep(MAIN_LOOP_DELAY)


# ============================
# MAIN
# ============================
if __name__ == "__main__":
    try:
        t = threading.Thread(target=controller_thread, daemon=True)
        t.start()

        robot_main_loop()

    except KeyboardInterrupt:
        print("\nStopping controller...")
        controller_running = False
        time.sleep(0.3)
        print("Stopped.")