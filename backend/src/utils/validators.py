"""
Validation utilities for user input (email, password, background data)
"""

import re
from typing import List, Optional
from email_validator import validate_email as email_validate, EmailNotValidError
from passlib.context import CryptContext


class ValidationError(Exception):
    """Custom validation error for user input validation"""
    pass


def validate_email(email: str) -> str:
    """
    Validate email format according to RFC 5322

    Args:
        email: Email address to validate

    Returns:
        str: Normalized email address (lowercase)

    Raises:
        ValidationError: If email format is invalid
    """
    try:
        # email-validator library validates RFC 5322 format
        validation = email_validate(email, check_deliverability=False)
        return validation.normalized
    except EmailNotValidError as e:
        raise ValidationError(f"Invalid email format: {str(e)}")


def validate_password_strength(password: str) -> None:
    """
    Validate password strength
    Requirements:
    - Minimum 8 characters
    - At least 1 uppercase letter
    - At least 1 lowercase letter
    - At least 1 digit

    Args:
        password: Password to validate

    Raises:
        ValidationError: If password doesn't meet strength requirements
    """
    if len(password) < 8:
        raise ValidationError("Password must be at least 8 characters long")

    if not re.search(r'[A-Z]', password):
        raise ValidationError("Password must contain at least one uppercase letter")

    if not re.search(r'[a-z]', password):
        raise ValidationError("Password must contain at least one lowercase letter")

    if not re.search(r'\d', password):
        raise ValidationError("Password must contain at least one digit")


def validate_experience_level(level: str) -> None:
    """
    Validate experience level value

    Args:
        level: Experience level to validate

    Raises:
        ValidationError: If level is not valid
    """
    valid_levels = ['beginner', 'intermediate', 'advanced']
    if level not in valid_levels:
        raise ValidationError(
            f"Invalid experience level '{level}'. Must be one of: {', '.join(valid_levels)}"
        )


def validate_array_not_empty(arr: List[str], field_name: str) -> None:
    """
    Validate that array is not empty and contains valid values

    Args:
        arr: Array to validate
        field_name: Field name for error message

    Raises:
        ValidationError: If array is empty or contains empty strings
    """
    if not arr or len(arr) == 0:
        raise ValidationError(f"{field_name} must contain at least one value")

    if any(not item or not item.strip() for item in arr):
        raise ValidationError(f"{field_name} cannot contain empty values")


# Predefined lists for validation
VALID_LANGUAGES = [
    'Python', 'JavaScript', 'TypeScript', 'Java', 'C++', 'C#', 'Go', 'Rust',
    'Ruby', 'PHP', 'Swift', 'Kotlin', 'R', 'MATLAB', 'SQL', 'Shell', 'Other'
]

VALID_FRAMEWORKS = [
    'React', 'Vue', 'Angular', 'FastAPI', 'Django', 'Flask', 'Express', 'NestJS',
    'Spring', 'ASP.NET', 'Ruby on Rails', 'Laravel', 'Fiber', 'Gin', 'Other'
]

VALID_PLATFORMS = [
    'desktop', 'mobile', 'web', 'embedded', 'cloud', 'iot', 'other'
]

VALID_DEVICE_TYPES = [
    'laptop', 'desktop', 'smartphone', 'tablet', 'raspberry-pi', 'arduino',
    'microcontroller', 'fpga', 'gpu', 'server', 'other'
]


def validate_background_languages(languages: List[str]) -> None:
    """
    Validate programming languages list

    Args:
        languages: List of programming languages

    Raises:
        ValidationError: If invalid language is provided
    """
    validate_array_not_empty(languages, "Programming languages")

    # Allow any language (don't enforce strict validation for extensibility)
    # This allows users to add custom languages not in predefined list


def validate_background_frameworks(frameworks: List[str]) -> None:
    """
    Validate frameworks list

    Args:
        frameworks: List of frameworks

    Raises:
        ValidationError: If validation fails
    """
    # Frameworks are optional but if provided, must not be empty strings
    if frameworks:
        for framework in frameworks:
            if not framework or not framework.strip():
                raise ValidationError("Frameworks list cannot contain empty values")


def validate_background_platforms(platforms: List[str]) -> None:
    """
    Validate development platforms list

    Args:
        platforms: List of development platforms

    Raises:
        ValidationError: If validation fails
    """
    validate_array_not_empty(platforms, "Development platforms")


def validate_background_devices(devices: List[str]) -> None:
    """
    Validate device types list

    Args:
        devices: List of device types

    Raises:
        ValidationError: If validation fails
    """
    # Devices are optional but if provided, must not be empty strings
    if devices:
        for device in devices:
            if not device or not device.strip():
                raise ValidationError("Device types list cannot contain empty values")


# Password hashing context for bcrypt
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain text password against a bcrypt hash

    Args:
        plain_password: Plain text password to verify
        hashed_password: Stored bcrypt hash

    Returns:
        bool: True if password matches the hash, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)
