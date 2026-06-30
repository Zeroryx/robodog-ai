# delivery/qr.py

import uuid
import qrcode


VALID_SENDERS = {
    "VALID_SENDER"
}

VALID_RECEIVERS = {
    "ROOM_101_RECEIVER",
    "ROOM_102_RECEIVER",
    "ROOM_201_RECEIVER",
    "LAB1_RECEIVER",
}


def verify_sender_qr(token: str) -> bool:
    """
    Verify sender QR token.
    """
    return token in VALID_SENDERS


def verify_receiver_qr(token: str) -> bool:
    """
    Verify receiver QR token.
    """
    return token in VALID_RECEIVERS


def generate_sender_token():
    """
    Generate mock sender token.
    """
    return "VALID_SENDER"


def generate_receiver_token(room: str):
    """
    Generate room-specific receiver token.
    """

    return f"{room.upper()}_RECEIVER"


def generate_qr_image(data: str, output_path: str):
    """
    Save QR image to file.
    """

    qr = qrcode.QRCode(
        version=1,
        box_size=10,
        border=4
    )

    qr.add_data(data)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")

    img.save(output_path)

    return output_path


def generate_delivery_id():
    """
    Generate unique delivery id.
    """

    return str(uuid.uuid4())