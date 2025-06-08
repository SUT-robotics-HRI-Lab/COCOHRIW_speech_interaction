from pydantic import BaseModel
from enum import Enum
from typing import List, Optional

# Enum for task types
class TaskType(str, Enum):
    RELOCATE_OBJECT = "Relocate_Object"
    NAVIGATE = "Navigate"
    INSPECT = "Inspect"
    FIND = "Find"
    MONITOR = "Monitor"
    REPORT = "Report"
    FOLLOW = "Follow"
    PICK = "Pick"
    PLACE = "Place"
    UNKNOWN = "Unknown"
    #ASSIST = "Assist"

# Data model for tasks
class TaskInformation(BaseModel):
    object_of_interest: Optional[List[str]] = None
    task_type: Optional[TaskType] = None
    source_location: Optional[List[str]] = None
    target_location: Optional[List[str]] = None
    context: Optional[str] = None
