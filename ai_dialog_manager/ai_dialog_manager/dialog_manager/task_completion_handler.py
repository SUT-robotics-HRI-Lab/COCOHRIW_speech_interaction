from typing import Dict, List, Tuple
from task_extractor.task_types import TaskInformation, TaskType

from enum import Enum

class TaskField(str, Enum):
    TASK_TYPE = "task_type"
    OBJECT = "object_of_interest"
    SOURCE = "source_location"
    TARGET = "target_location"

class TaskCompletionHandler:
    def __init__(self):
        self.task_buffer: Dict[int, TaskInformation] = {}
        self.missing_info: Dict[int, List[TaskField]] = {}

    def register_task(self, task: TaskInformation, task_index: int) -> Tuple[bool, str, List[TaskField]]:
        self.task_buffer[task_index] = task
        return self._validate_task(task, task_index)

    def _validate_task(self, task: TaskInformation, task_index: int) -> Tuple[bool, str, List[TaskField]]:
        missing: List[TaskField] = []

        if not task.task_type:
            missing.append(TaskField.TASK_TYPE)
            self.missing_info[task_index] = missing
            return False, "The task type is missing and must be specified.", missing

        t = task.task_type

        if t == TaskType.NAVIGATE:
            if not task.target_location:
                missing.append(TaskField.TARGET)

        elif t == TaskType.RELOCATE_OBJECT:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        elif t == TaskType.INSPECT:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        elif t == TaskType.REPORT:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        elif t == TaskType.FIND:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        elif t == TaskType.MONITOR:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        # elif t == TaskType.ASSIST:
        #     if not task.object_of_interest:
        #         missing.append(TaskField.OBJECT)
        #     if not task.source_location:
        #         missing.append(TaskField.SOURCE)
        #     if not task.target_location:
        #         missing.append(TaskField.TARGET)

        elif t == TaskType.FOLLOW:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)

        elif t == TaskType.PICK:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.source_location:
                missing.append(TaskField.SOURCE)
                
        elif t == TaskType.PLACE:
            if not task.object_of_interest:
                missing.append(TaskField.OBJECT)
            if not task.target_location:
                missing.append(TaskField.TARGET)

        else:
            missing.append(TaskField.TASK_TYPE)

        if missing:
            self.missing_info[task_index] = missing
            sentence = self._build_clarification_sentence(t, missing, task)
            return False, sentence, missing

        self.missing_info.pop(task_index, None)
        return True, "", []

    def _build_clarification_sentence(self, task_type: TaskType, missing: List[TaskField], task: TaskInformation = None) -> str:
        # Descriptions for missing fields based on task type
        descriptions = []

        # Ensure task is provided for dynamic field access
        if task is None:
            return "The task is missing required information."

        for field in missing:
            if task_type == TaskType.NAVIGATE:
                if field == TaskField.TARGET:
                    descriptions.append("target location to navigate to")

            elif task_type == TaskType.RELOCATE_OBJECT:
                if field == TaskField.OBJECT:
                    descriptions.append("object to relocate")
                elif field == TaskField.SOURCE:
                    descriptions.append(
                        "current location of the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object")
                    )
                elif field == TaskField.TARGET:
                    descriptions.append(
                        "destination to relocate the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object") +
                        " to"
                    )

            elif task_type == TaskType.INSPECT:
                if field == TaskField.OBJECT:
                    descriptions.append("object to inspect")
                elif field == TaskField.TARGET:
                    descriptions.append(
                        "location of the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object") +
                        " to inspect"
                    )

            elif task_type == TaskType.REPORT:
                if field == TaskField.OBJECT:
                    descriptions.append("object to report on")
                elif field == TaskField.SOURCE:
                    descriptions.append(
                        "location of the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object") +
                        " to report from"
                    )
                elif field == TaskField.TARGET:
                    descriptions.append("recipient or target of the report")

            elif task_type == TaskType.FIND:
                if field == TaskField.OBJECT:
                    descriptions.append("object to find")
                elif field in [TaskField.SOURCE, TaskField.TARGET]:
                    descriptions.append(
                        "location of the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object") +
                        " to find"
                    )

            elif task_type == TaskType.MONITOR:
                if field == TaskField.OBJECT:
                    descriptions.append("object to monitor")
                elif field in [TaskField.SOURCE, TaskField.TARGET]:
                    descriptions.append(
                        "location of the " +
                        (", ".join(task.object_of_interest) if task.object_of_interest else "object") +
                        " to monitor"
                    )

            # elif task_type == TaskType.ASSIST:
            #     if field == TaskField.OBJECT:
            #         descriptions.append("object you need help with")
            #     elif field in [TaskField.SOURCE, TaskField.TARGET]:
            #         descriptions.append("location of the object to assist with")

            elif task_type == TaskType.FOLLOW:
                if field == TaskField.OBJECT:
                    descriptions.append("entity to follow")
                elif field == TaskField.SOURCE:
                    descriptions.append("starting location of the entity to follow")
            
            elif task_type == TaskType.PICK:
                if field == TaskField.OBJECT:
                    descriptions.append("object to pick")
                elif field == TaskField.SOURCE:
                    descriptions.append("From what location to pick the " + (", ".join(task.object_of_interest) if task.object_of_interest else "object") + "?")

            elif task_type == TaskType.PLACE:
                if field == TaskField.OBJECT:
                    descriptions.append("object to place")
                elif field == TaskField.TARGET:
                    descriptions.append("Where to place the " + (", ".join(task.object_of_interest) if task.object_of_interest else "object") + "?")

            elif field == TaskField.TASK_TYPE:
                descriptions.append("type of task")

        if not descriptions:
            return "The task is missing required information."

        if len(descriptions) == 1:
            return f"The task is missing the following information: {descriptions[0]}."
        else:
            combined = ", ".join(descriptions[:-1]) + " and " + descriptions[-1]
            return f"The task is missing the following information: {combined}."

    def get_task(self, task_index: int) -> TaskInformation:
        return self.task_buffer.get(task_index)

    def get_missing_fields(self, task_index: int) -> List[str]:
        # Return the missing fields as string values for compatibility
        return [field.value for field in self.missing_info.get(task_index, [])]

    def update_task(self, task_index: int, new_data: TaskInformation):
        existing = self.task_buffer.get(task_index)
        if not existing:
            self.task_buffer[task_index] = new_data
            return
        if new_data.task_type:
            existing.task_type = new_data.task_type
        if new_data.object_of_interest:
            existing.object_of_interest = new_data.object_of_interest
        if new_data.source_location:
            existing.source_location = new_data.source_location
        if new_data.target_location:
            existing.target_location = new_data.target_location
        if new_data.context:
            existing.context = new_data.context
