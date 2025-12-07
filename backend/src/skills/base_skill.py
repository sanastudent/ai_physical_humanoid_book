"""
Base skill class for reusable AI agent skills

Implements FR-019: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
import logging


class BaseSkill(ABC):
    """
    Base class for all reusable skills in the AI agent system.

    Skills are designed to be reusable components that can be called by different agents.
    Each skill should be self-contained and focused on a specific task.
    """

    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def execute(self, **kwargs) -> Dict[str, Any]:
        """
        Execute the skill with the given parameters.

        Args:
            **kwargs: Skill-specific parameters

        Returns:
            Dictionary containing the result of the skill execution
        """
        pass

    def validate_inputs(self, **kwargs) -> bool:
        """
        Validate the inputs for the skill execution.
        Override this method in subclasses if input validation is needed.
        """
        return True

    def __str__(self) -> str:
        return f"{self.name}: {self.description}"