# core/state_machine.py
from enum import Enum, auto

class RobotState(Enum):
    IDLE                  = auto()
    VERIFYING_SENDER_QR   = auto()
    FORM_INPUT            = auto()
    PACKAGE_ACCEPTED      = auto()
    COUNTDOWN             = auto()
    NAVIGATING            = auto()
    AT_DESTINATION        = auto()
    VERIFYING_RECEIVER_QR = auto()
    DELIVERING            = auto()
    RETURNING             = auto()
    ERROR                 = auto()

class StateMachine:
    def __init__(self):
        self.state = RobotState.IDLE

    def transition(self, new_state: RobotState):
        # define valid transitions here
        valid = {
            RobotState.IDLE:                  [RobotState.VERIFYING_SENDER_QR],
            RobotState.VERIFYING_SENDER_QR:   [RobotState.FORM_INPUT, RobotState.IDLE],
            RobotState.FORM_INPUT:            [RobotState.PACKAGE_ACCEPTED, RobotState.IDLE],
            RobotState.PACKAGE_ACCEPTED:      [RobotState.COUNTDOWN],
            RobotState.COUNTDOWN:             [RobotState.NAVIGATING],
            RobotState.NAVIGATING:            [RobotState.AT_DESTINATION, RobotState.ERROR],
            RobotState.AT_DESTINATION:        [RobotState.VERIFYING_RECEIVER_QR],
            RobotState.VERIFYING_RECEIVER_QR: [RobotState.DELIVERING, RobotState.AT_DESTINATION],
            RobotState.DELIVERING:            [RobotState.RETURNING],
            RobotState.RETURNING:             [RobotState.IDLE],
            RobotState.ERROR:                 [RobotState.IDLE],
        }
        if new_state in valid[self.state]:
            self.state = new_state
        else:
            raise ValueError(f"Invalid transition: {self.state} → {new_state}")