# delivery/delivery.py

import time

from config.settings import (
    COUNTDOWN_SECONDS,
    CAMPUS_ROOMS,
)

from delivery.validator import validate_package
from delivery.qr import (
    verify_sender_qr,
    verify_receiver_qr,
    generate_delivery_id,
)

from core.state_machine import (
    StateMachine,
    RobotState,
)


class DeliveryManager:
    """
    Handles the complete delivery workflow.

    This module contains business logic only.
    Navigation and UI are handled elsewhere.
    """

    def __init__(self, state_machine=None):

        self.state_machine = state_machine or StateMachine()

        self.delivery_id = None
        self.current_delivery = None

        self.sender_verified = False
        self.receiver_verified = False

    # =====================================================
    # DELIVERY CREATION
    # =====================================================

    def start_delivery(
        self,
        sender_qr,
        room,
        length,
        width,
        height,
        weight,
    ):
        """
        Start a new delivery request.

        Returns:
            (success, message)
        """

        # -------------------------------------------------
        # Verify sender
        # -------------------------------------------------

        self.state_machine.transition(
            RobotState.VERIFYING_SENDER_QR
        )

        if not verify_sender_qr(sender_qr):

            self.state_machine.transition(
                RobotState.IDLE
            )

            return False, "Invalid sender QR"

        self.sender_verified = True

        # -------------------------------------------------
        # Form input
        # -------------------------------------------------

        self.state_machine.transition(
            RobotState.FORM_INPUT
        )

        # -------------------------------------------------
        # Validate room
        # -------------------------------------------------

        if room not in CAMPUS_ROOMS:
            return False, f"Unknown room: {room}"

        # -------------------------------------------------
        # Validate package
        # -------------------------------------------------

        valid, errors = validate_package(
            length,
            width,
            height,
            weight,
        )

        if not valid:
            return False, "\n".join(errors)

        # -------------------------------------------------
        # Package accepted
        # -------------------------------------------------

        self.state_machine.transition(
            RobotState.PACKAGE_ACCEPTED
        )

        room_info = CAMPUS_ROOMS[room]

        self.delivery_id = generate_delivery_id()

        self.current_delivery = {
            "delivery_id": self.delivery_id,
            "room": room,
            "floor": room_info["floor"],
            "grid_pos": room_info["grid_pos"],
            "label": room_info["label"],
            "length": length,
            "width": width,
            "height": height,
            "weight": weight,
        }

        return True, "Package accepted"

    # =====================================================
    # COUNTDOWN
    # =====================================================

    def begin_countdown(self):
        """
        Countdown before navigation starts.
        """

        self.state_machine.transition(
            RobotState.COUNTDOWN
        )

        for remaining in range(
            COUNTDOWN_SECONDS,
            0,
            -1,
        ):
            print(
                f"[Delivery] Starting in {remaining}s..."
            )
            time.sleep(1)

        self.state_machine.transition(
            RobotState.NAVIGATING
        )

        return True

    # =====================================================
    # DESTINATION
    # =====================================================

    def destination_reached(self):
        """
        Called by navigation subsystem.
        """

        self.state_machine.transition(
            RobotState.AT_DESTINATION
        )

    # =====================================================
    # RECEIVER QR
    # =====================================================

    def verify_receiver(self, receiver_qr):
        """
        Verify receiver identity.
        """

        self.state_machine.transition(
            RobotState.VERIFYING_RECEIVER_QR
        )

        if not verify_receiver_qr(receiver_qr):

            self.state_machine.transition(
                RobotState.AT_DESTINATION
            )

            return False

        self.receiver_verified = True

        self.state_machine.transition(
            RobotState.DELIVERING
        )

        return True

    # =====================================================
    # DELIVERY COMPLETE
    # =====================================================

    def complete_delivery(self):
        """
        Package handed over.
        """

        if not self.receiver_verified:
            raise RuntimeError(
                "Receiver has not been verified."
            )

        self.state_machine.transition(
            RobotState.RETURNING
        )

        return True

    # =====================================================
    # RETURN COMPLETE
    # =====================================================

    def returned_to_base(self):
        """
        Called when robot reaches base station.
        """

        self.state_machine.transition(
            RobotState.IDLE
        )

        self.delivery_id = None
        self.current_delivery = None

        self.sender_verified = False
        self.receiver_verified = False

    # =====================================================
    # ERROR HANDLING
    # =====================================================

    def navigation_error(self):
        """
        Called by navigation subsystem.
        """

        self.state_machine.transition(
            RobotState.ERROR
        )

    def reset_error(self):
        """
        Reset robot after an error.
        """

        self.state_machine.transition(
            RobotState.IDLE
        )

    # =====================================================
    # STATUS
    # =====================================================

    @property
    def current_state(self):
        return self.state_machine.state

    def get_delivery_info(self):
        return self.current_delivery

    def has_active_delivery(self):
        return self.current_delivery is not None