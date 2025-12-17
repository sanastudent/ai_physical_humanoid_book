"""
Background Service - Business logic for user background data management
"""

from datetime import datetime
from typing import Optional
from uuid import UUID

from ..database.connection import get_db_pool
from ..models.background import BackgroundInput, BackgroundResponse
from ..models.software_background import SoftwareBackground, SoftwareBackgroundCreate
from ..models.hardware_background import HardwareBackground, HardwareBackgroundCreate
from ..utils.validators import (
    validate_experience_level,
    validate_background_languages,
    validate_background_frameworks,
    validate_background_platforms,
    validate_background_devices,
    ValidationError
)


class BackgroundService:
    """Service for user background data creation and management"""

    @staticmethod
    def validate_background_data(background_data: BackgroundInput) -> None:
        """
        Validate background data before database operations

        Args:
            background_data: Background data to validate

        Raises:
            ValidationError: If validation fails
        """
        # Validate software background
        validate_experience_level(background_data.software.experience_level.value)
        validate_background_languages(background_data.software.preferred_languages)
        validate_background_frameworks(background_data.software.preferred_frameworks)

        # Validate hardware background
        validate_experience_level(background_data.hardware.experience_level.value)
        validate_background_platforms(background_data.hardware.preferred_platforms)
        validate_background_devices(background_data.hardware.device_types)

    @staticmethod
    def create_background(user_id: UUID, background_data: BackgroundInput) -> BackgroundResponse:
        """
        Create background data for a user (both software and hardware)

        Args:
            user_id: User UUID
            background_data: Combined software and hardware background data

        Returns:
            BackgroundResponse: Created background data

        Raises:
            ValidationError: If validation fails or background already exists
            Exception: If database operation fails
        """
        # Validate background data
        BackgroundService.validate_background_data(background_data)

        # Check if background already exists
        if BackgroundService.check_background_exists(user_id):
            raise ValidationError("User background already exists. Use update endpoint instead.")

        db_pool = get_db_pool()

        # Create software background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                INSERT INTO software_background
                (user_id, experience_level, preferred_languages, preferred_frameworks, created_at, updated_at)
                VALUES (%s, %s, %s, %s, NOW(), NOW())
                RETURNING id, user_id, experience_level, preferred_languages, preferred_frameworks, created_at, updated_at
                """,
                (
                    str(user_id),
                    background_data.software.experience_level.value,
                    background_data.software.preferred_languages,
                    background_data.software.preferred_frameworks
                )
            )
            software_record = cursor.fetchone()

        if not software_record:
            raise Exception("Failed to create software background")

        # Create hardware background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                INSERT INTO hardware_background
                (user_id, experience_level, preferred_platforms, device_types, created_at, updated_at)
                VALUES (%s, %s, %s, %s, NOW(), NOW())
                RETURNING id, user_id, experience_level, preferred_platforms, device_types, created_at, updated_at
                """,
                (
                    str(user_id),
                    background_data.hardware.experience_level.value,
                    background_data.hardware.preferred_platforms,
                    background_data.hardware.device_types
                )
            )
            hardware_record = cursor.fetchone()

        if not hardware_record:
            raise Exception("Failed to create hardware background")

        # Construct response
        return BackgroundResponse(
            user_id=user_id,
            software=SoftwareBackground(
                id=software_record['id'],
                user_id=software_record['user_id'],
                experience_level=software_record['experience_level'],
                preferred_languages=software_record['preferred_languages'],
                preferred_frameworks=software_record['preferred_frameworks'],
                created_at=software_record['created_at'],
                updated_at=software_record['updated_at']
            ),
            hardware=HardwareBackground(
                id=hardware_record['id'],
                user_id=hardware_record['user_id'],
                experience_level=hardware_record['experience_level'],
                preferred_platforms=hardware_record['preferred_platforms'],
                device_types=hardware_record['device_types'],
                created_at=hardware_record['created_at'],
                updated_at=hardware_record['updated_at']
            )
        )

    @staticmethod
    def get_user_background(user_id: UUID) -> Optional[BackgroundResponse]:
        """
        Get background data for a user

        Args:
            user_id: User UUID

        Returns:
            BackgroundResponse: User's background data if exists, None otherwise
        """
        db_pool = get_db_pool()

        # Get software background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, user_id, experience_level, preferred_languages, preferred_frameworks, created_at, updated_at
                FROM software_background
                WHERE user_id = %s
                """,
                (str(user_id),)
            )
            software_record = cursor.fetchone()

        if not software_record:
            return None

        # Get hardware background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, user_id, experience_level, preferred_platforms, device_types, created_at, updated_at
                FROM hardware_background
                WHERE user_id = %s
                """,
                (str(user_id),)
            )
            hardware_record = cursor.fetchone()

        if not hardware_record:
            return None

        # Construct response
        return BackgroundResponse(
            user_id=user_id,
            software=SoftwareBackground(
                id=software_record['id'],
                user_id=software_record['user_id'],
                experience_level=software_record['experience_level'],
                preferred_languages=software_record['preferred_languages'],
                preferred_frameworks=software_record['preferred_frameworks'],
                created_at=software_record['created_at'],
                updated_at=software_record['updated_at']
            ),
            hardware=HardwareBackground(
                id=hardware_record['id'],
                user_id=hardware_record['user_id'],
                experience_level=hardware_record['experience_level'],
                preferred_platforms=hardware_record['preferred_platforms'],
                device_types=hardware_record['device_types'],
                created_at=hardware_record['created_at'],
                updated_at=hardware_record['updated_at']
            )
        )

    @staticmethod
    def update_background(user_id: UUID, background_data: BackgroundInput) -> BackgroundResponse:
        """
        Update existing background data for a user

        Args:
            user_id: User UUID
            background_data: Updated background data

        Returns:
            BackgroundResponse: Updated background data

        Raises:
            ValidationError: If validation fails or background doesn't exist
            Exception: If database operation fails
        """
        # Validate background data
        BackgroundService.validate_background_data(background_data)

        # Check if background exists
        if not BackgroundService.check_background_exists(user_id):
            raise ValidationError("User background does not exist. Use create endpoint instead.")

        db_pool = get_db_pool()

        # Update software background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                UPDATE software_background
                SET experience_level = %s,
                    preferred_languages = %s,
                    preferred_frameworks = %s,
                    updated_at = NOW()
                WHERE user_id = %s
                RETURNING id, user_id, experience_level, preferred_languages, preferred_frameworks, created_at, updated_at
                """,
                (
                    background_data.software.experience_level.value,
                    background_data.software.preferred_languages,
                    background_data.software.preferred_frameworks,
                    str(user_id)
                )
            )
            software_record = cursor.fetchone()

        if not software_record:
            raise Exception("Failed to update software background")

        # Update hardware background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                UPDATE hardware_background
                SET experience_level = %s,
                    preferred_platforms = %s,
                    device_types = %s,
                    updated_at = NOW()
                WHERE user_id = %s
                RETURNING id, user_id, experience_level, preferred_platforms, device_types, created_at, updated_at
                """,
                (
                    background_data.hardware.experience_level.value,
                    background_data.hardware.preferred_platforms,
                    background_data.hardware.device_types,
                    str(user_id)
                )
            )
            hardware_record = cursor.fetchone()

        if not hardware_record:
            raise Exception("Failed to update hardware background")

        # Construct response
        return BackgroundResponse(
            user_id=user_id,
            software=SoftwareBackground(
                id=software_record['id'],
                user_id=software_record['user_id'],
                experience_level=software_record['experience_level'],
                preferred_languages=software_record['preferred_languages'],
                preferred_frameworks=software_record['preferred_frameworks'],
                created_at=software_record['created_at'],
                updated_at=software_record['updated_at']
            ),
            hardware=HardwareBackground(
                id=hardware_record['id'],
                user_id=hardware_record['user_id'],
                experience_level=hardware_record['experience_level'],
                preferred_platforms=hardware_record['preferred_platforms'],
                device_types=hardware_record['device_types'],
                created_at=hardware_record['created_at'],
                updated_at=hardware_record['updated_at']
            )
        )

    @staticmethod
    def check_background_exists(user_id: UUID) -> bool:
        """
        Check if background data exists for a user

        Args:
            user_id: User UUID

        Returns:
            bool: True if background exists, False otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT EXISTS(
                    SELECT 1 FROM software_background WHERE user_id = %s
                ) AND EXISTS(
                    SELECT 1 FROM hardware_background WHERE user_id = %s
                )
                """,
                (str(user_id), str(user_id))
            )
            result = cursor.fetchone()
            # Result will be a dict with 'bool_and' key (PostgreSQL AND operator result)
            return list(result.values())[0] if result else False

    @staticmethod
    def delete_background(user_id: UUID) -> bool:
        """
        Delete background data for a user (useful for testing or account deletion)

        Args:
            user_id: User UUID

        Returns:
            bool: True if background was deleted, False if didn't exist
        """
        if not BackgroundService.check_background_exists(user_id):
            return False

        db_pool = get_db_pool()

        # Delete software background (hardware will cascade if FK is set)
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "DELETE FROM software_background WHERE user_id = %s",
                (str(user_id),)
            )

        # Delete hardware background
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "DELETE FROM hardware_background WHERE user_id = %s",
                (str(user_id),)
            )

        return True
