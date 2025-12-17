"""
User Background API Routes - Create, Get, Update Background Data
"""

from fastapi import APIRouter, HTTPException, Depends
from uuid import UUID

from ..models.background import BackgroundInput, BackgroundResponse
from ..services.background_service import BackgroundService
from ..middleware.auth_middleware import require_authentication
from ..utils.validators import ValidationError

router = APIRouter(prefix="/background", tags=["background"])


@router.post("", response_model=BackgroundResponse, status_code=201)
async def create_background(
    background_data: BackgroundInput,
    session_data: dict = Depends(require_authentication)
):
    """
    Create background data for authenticated user (Step 2 of signup flow)

    **Authentication Required**

    **Flow:**
    1. User completes signup (email/password) → receives session cookie
    2. User redirects to background questions page
    3. User submits background data (software + hardware)
    4. Backend creates background records and associates with user

    **Request Body:**
    ```json
    {
      "software": {
        "experience_level": "intermediate",
        "preferred_languages": ["Python", "JavaScript"],
        "preferred_frameworks": ["FastAPI", "React"]
      },
      "hardware": {
        "experience_level": "beginner",
        "preferred_platforms": ["web", "desktop"],
        "device_types": ["laptop", "smartphone"]
      }
    }
    ```

    **Validation Rules:**
    - experience_level: Must be "beginner", "intermediate", or "advanced"
    - preferred_languages: At least 1 language required
    - preferred_frameworks: Optional, can be empty array
    - preferred_platforms: At least 1 platform required
    - device_types: Optional, can be empty array

    **Response:**
    - 201: Background created successfully
    - 400: Validation error or background already exists
    - 401: Not authenticated
    - 500: Server error
    """
    try:
        user_id = session_data["user_id"]

        # Create background data
        result = BackgroundService.create_background(user_id, background_data)

        return result

    except ValidationError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Create background error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to create background data")


@router.get("", response_model=BackgroundResponse)
async def get_background(session_data: dict = Depends(require_authentication)):
    """
    Get background data for authenticated user

    **Authentication Required**

    **Use Cases:**
    - Load user preferences for personalization
    - Display user profile information
    - Check if user has completed background questions

    **Response:**
    - 200: Background data found
    - 404: No background data exists for user
    - 401: Not authenticated
    """
    try:
        user_id = session_data["user_id"]

        # Get background data
        result = BackgroundService.get_user_background(user_id)

        if not result:
            raise HTTPException(status_code=404, detail="Background data not found")

        return result

    except HTTPException:
        raise
    except Exception as e:
        print(f"Get background error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve background data")


@router.put("", response_model=BackgroundResponse)
async def update_background(
    background_data: BackgroundInput,
    session_data: dict = Depends(require_authentication)
):
    """
    Update background data for authenticated user

    **Authentication Required**

    **Use Cases:**
    - User updates their preferences
    - User changes experience level
    - User adds/removes languages or frameworks

    **Request Body:** Same format as create endpoint

    **Response:**
    - 200: Background updated successfully
    - 400: Validation error or background doesn't exist
    - 401: Not authenticated
    - 500: Server error
    """
    try:
        user_id = session_data["user_id"]

        # Update background data
        result = BackgroundService.update_background(user_id, background_data)

        return result

    except ValidationError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Update background error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update background data")


@router.delete("", status_code=204)
async def delete_background(session_data: dict = Depends(require_authentication)):
    """
    Delete background data for authenticated user

    **Authentication Required**

    **Use Cases:**
    - User wants to reset their preferences
    - Testing and development
    - Account cleanup

    **Response:**
    - 204: Background deleted successfully
    - 404: No background data exists
    - 401: Not authenticated
    """
    try:
        user_id = session_data["user_id"]

        # Delete background data
        deleted = BackgroundService.delete_background(user_id)

        if not deleted:
            raise HTTPException(status_code=404, detail="Background data not found")

        return None  # 204 No Content

    except HTTPException:
        raise
    except Exception as e:
        print(f"Delete background error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to delete background data")
