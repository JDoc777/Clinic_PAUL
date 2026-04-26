from PyQt5 import QtWidgets
from claw_test import IKWindow, current_angles_to_servos

class ClawSimWindow(IKWindow):
    def __init__(self, controller=None):
        super().__init__()
        self.external_controller = controller
        self.setWindowTitle("PAUL Claw Simulation")

    def _send_sim_to_arm(self):
        """
        Override original claw_test serial sending.
        This prevents the sim window from opening its own serial connection.
        Instead it sends targets to the controller from run_all.py.
        """
        if self.external_controller is None:
            return

        servos = current_angles_to_servos(
            self.current_angles,
            grip_value=self.grip
        )

        self.external_controller.set_targets(servos)