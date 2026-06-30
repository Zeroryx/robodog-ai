from delivery.delivery import DeliveryManager


def main():

    dm = DeliveryManager()

    print("STATE:", dm.current_state)

    success, message = dm.start_delivery(
        sender_qr="VALID_SENDER",
        room="101",
        length=10,
        width=10,
        height=10,
        weight=1.5,
    )

    print(success)
    print(message)

    print("STATE:", dm.current_state)

    dm.begin_countdown()

    print("STATE:", dm.current_state)

    dm.destination_reached()

    print("STATE:", dm.current_state)

    verified = dm.verify_receiver(
        "ROOM_101_RECEIVER"
    )

    print("Receiver verified:", verified)

    print("STATE:", dm.current_state)

    dm.complete_delivery()

    print("STATE:", dm.current_state)

    dm.returned_to_base()

    print("STATE:", dm.current_state)


if __name__ == "__main__":
    main()