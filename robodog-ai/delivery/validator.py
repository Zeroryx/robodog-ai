# delivery/validator.py

from config.settings import (
    MAX_PACKAGE_LENGTH_CM,
    MAX_PACKAGE_WIDTH_CM,
    MAX_PACKAGE_HEIGHT_CM,
    MAX_PACKAGE_WEIGHT_KG,
)


def validate_package(length, width, height, weight):
    """
    Validate package dimensions and weight.

    Returns:
        (bool, list[str])
        True if valid, False otherwise.
    """

    errors = []

    if length > MAX_PACKAGE_LENGTH_CM:
        errors.append(
            f"Length exceeds limit ({MAX_PACKAGE_LENGTH_CM} cm)"
        )

    if width > MAX_PACKAGE_WIDTH_CM:
        errors.append(
            f"Width exceeds limit ({MAX_PACKAGE_WIDTH_CM} cm)"
        )

    if height > MAX_PACKAGE_HEIGHT_CM:
        errors.append(
            f"Height exceeds limit ({MAX_PACKAGE_HEIGHT_CM} cm)"
        )

    if weight > MAX_PACKAGE_WEIGHT_KG:
        errors.append(
            f"Weight exceeds limit ({MAX_PACKAGE_WEIGHT_KG} kg)"
        )

    return len(errors) == 0, errors