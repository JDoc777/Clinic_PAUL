import time
import Payload
import testingUART

class ServoController:
    def __init__(self, step_sizes=None, transition_delay=0.01, shared_data=None):
        """Initialize the servo controller."""
        self.current_positions = [0, 0, 0, 0, 0]  # Initial positions
        self.target_positions = [0, 0, 0, 0, 0]   # Target positions
        self.state = "resting"  # Initial state
        self.grabbed_object = False  # Flag for detecting object grab
        self.last_update_time = time.time()  # For non-blocking delay
        self.locked_close_position = None  # Store locked close position
        self.transition_delay = transition_delay  # Delay between position updates
        self.step_sizes = step_sizes if step_sizes else [1, 1, 1, 1, 1]  # Step sizes for each servo
        # shared_data is the UARTSharedData (or compatible) object used by Payload
        # If None, Payload calls will be skipped.
        self.shared = shared_data
        # debug helper to print once when run() starts
        self._run_started = False
        self.last_grab_flag = False
        self.BACKOFF = 20  # degrees to reopen after detecting grab


    # delete this function later
    def set_servos(self, s1, s2, s3, s4, s5):
        """Simulate setting servo positions by printing them."""
        self.current_positions = [s1, s2, s3, s4, s5]
        print(f"Servo positions: s1={s1}, s2={s2}, s3={s3}, s4={s4}, s5={s5}")

    def smooth_transition(self):
        """Smoothly transition to target positions."""
        current_time = time.time()
        if current_time - self.last_update_time >= self.transition_delay:
            for i in range(5):
                # Stop the last servo (s5) if an object is detected
                if i == 4 and self.grabbed_object:
                    # Smoothly approach backed-off locked position
                    if self.current_positions[i] < self.locked_close_position:
                        self.current_positions[i] += 1
                    elif self.current_positions[i] > self.locked_close_position:
                        self.current_positions[i] -= 1
                    continue
                if self.current_positions[i] < self.target_positions[i]:
                    self.current_positions[i] += min(self.step_sizes[i], self.target_positions[i] - self.current_positions[i])
                elif self.current_positions[i] > self.target_positions[i]:
                    self.current_positions[i] -= min(self.step_sizes[i], self.current_positions[i] - self.target_positions[i])
            # Print the live servo positions during the transition (local simulation)
            self.set_servos(*self.current_positions)  # Unpack the list into arguments
            # update servos in payload/shared_data if provided
            if self.shared is not None:
                Payload.set_servos(self.shared, *self.current_positions)
            else:
                # no shared_data provided; skip writing to payload
                pass
            self.last_update_time = current_time

    def set_state(self, state):
        """Set the target state and positions."""
        if state == "resting" and self.state in ["extended", "open"]:  # Allow "resting" from "open"
            self.target_positions = [0, 0, 0, 0, 0]
            self.state = "resting"
        elif state == "extended" and self.state in ["resting", "pickup"]:
            self.target_positions = [45, 90, 45, 45, self.current_positions[4] if self.state == "pickup" else 10]
            self.state = "extended"
        elif state == "open" and self.state in ["extended", "close"]:
            self.target_positions = [45, 90, 45, 45, 180]
            self.state = "open"
            self.grabbed_object = False  # Reset the grabbed_object flag
        elif state == "close" and self.state in ["extended", "open"]:  # Allow "close" from "open"
            self.target_positions = [45, 90, 45, 45, 0]  # Ensure last servo is set to 0
            self.state = "close"
        elif state == "pickup" and self.state == "close" and self.grabbed_object:
            self.target_positions = [180, 180, 180, 180, self.locked_close_position]
            self.state = "pickup"
        else:
            print(f"Invalid state transition: {self.state} -> {state}")
            return
        print(f"Transitioning to state: {self.state}")

    def detect_object(self):
        """Simulate object detection."""
        self.grabbed_object = True
        self.locked_close_position = self.current_positions[4]
        print("Object detected and grabbed.")

    def run(self):
        """Main loop to simulate the servo controller."""
        if not self._run_started:
            print("[claw] ServoController.run thread started")
            self._run_started = True
        while True:
            # --- UPDATE grabbed_object FROM Arduino flag (bit 6 example) ---
            #'''
            if self.shared is not None:
                data = self.shared.get_received_data()
                if data and "relay_bits" in data:
                    grab_flag = bool((data["relay_bits"] >> 0) & 0x01)

                    # edge detect: only trigger once
                    if grab_flag and not self.last_grab_flag:
                        print(f"[claw] GRAB FLAG TRIGGERED at pos={self.current_positions[4]}")

                        self.grabbed_object = True

                        # apply BACKOFF: servo 5 closes by decreasing angle
                        backoff_pos = self.current_positions[4] + self.BACKOFF
                        backoff_pos = max(0, min(180, backoff_pos))

                        self.locked_close_position = backoff_pos
                        print(f"[claw] locking claw at backed-off pos={self.locked_close_position}")

                    self.last_grab_flag = grab_flag
            #'''
            self.smooth_transition()
            time.sleep(0.01)  # Small delay to prevent high CPU usage

# ---- NEW HELPERS (moved from run_all.py) ----
def wait_or_exit(stop_event, timeout):
	"""Wait up to timeout seconds, return False if stop_event cleared early."""
	end = time.time() + timeout
	while time.time() < end:
		if not stop_event.is_set():
			return False
		time.sleep(0.05)
	return True

def start_claw_controller(step_sizes=None, transition_delay=None, shared_data=None):
	"""Create and start the ServoController.run loop in a daemon thread and return controller.

	Pass a `shared_data` (e.g. an instance of testingUART.UARTSharedData or the object
	returned by testingUART.start_reader) so `Payload.set_servos(shared_data, ...)`
	updates the real outgoing packet state.
	"""
	ctrl = ServoController(step_sizes=step_sizes, transition_delay=transition_delay if transition_delay is not None else 0.01, shared_data=shared_data)
	import threading
	threading.Thread(target=ctrl.run, daemon=True).start()
	return ctrl

def start_claw_demo_thread(stop_event, controller, whenDetect=1):
	"""Start the demo sequence in a daemon thread; sequence exits early if stop_event cleared."""
	import threading
	def seq():
		# small initial delay
		if not wait_or_exit(stop_event, 1.0):
			return
		if not stop_event.is_set(): return
		controller.set_state("extended")
		if not wait_or_exit(stop_event, 2.0): return
		controller.set_state("open")
		if not wait_or_exit(stop_event, 2.3): return
		controller.set_state("close")
		if not wait_or_exit(stop_event, whenDetect): return
		#controller.detect_object()
		if not wait_or_exit(stop_event, max(0, 2 - whenDetect)): return
		controller.set_state("pickup")
		if not wait_or_exit(stop_event, 1.5): return
		controller.set_state("extended")
		if not wait_or_exit(stop_event, 2.0): return
		controller.set_state("open")
		if not wait_or_exit(stop_event, 1.0): return
		controller.set_state("resting")
	t = threading.Thread(target=seq, daemon=True)
	t.start()
	return t
# ---- END NEW HELPERS ----

# Example usage
if __name__ == "__main__":
    whenDetect= .5
    # Create the controller with custom step sizes for each servo
    # create a local shared_data (doesn't open serial) and pass it to the controller
    shared_data = testingUART.UARTSharedData()

    # create and start controller in background thread
    controller = start_claw_controller(step_sizes=[2, 3, 1, 4, 5], transition_delay=0.02, shared_data=shared_data)

    # Start the main loop to continuously process transitions   
    time.sleep(1)  # Wait for transition
    controller.set_state("extended")  # resting -> extended
    time.sleep(2)  # Wait for transition
    controller.set_state("open")  # resting -> extended
    time.sleep(2.3)  # Wait for transition
    controller.set_state("close")  # extended -> close
    time.sleep(whenDetect)  # Wait for transition
    controller.detect_object()  # Simulate grabbing an object
    time.sleep(2 - whenDetect)  # Wait for transition
    controller.set_state("pickup")  # close -> pickup
    time.sleep(1.5)  # Wait for transition
    controller.set_state("extended")  # pickup -> extended
    time.sleep(2)  # Wait for transition
    controller.set_state("open")  # resting -> extended
    time.sleep(1)  # Wait for transition
    controller.set_state("resting")  # extended -> resting
    time.sleep(1)  # Wait for transition
