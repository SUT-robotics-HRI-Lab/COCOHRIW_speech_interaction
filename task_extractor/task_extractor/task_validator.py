from typing import Dict, List
from task_extractor.task_types import TaskInformation, TaskType

class TaskValidator:
    def __init__(self):
        self.missing_info_questions: Dict[int, List[str]] = {}

    def validate_task(self, task: TaskInformation, task_index: int) -> bool:
        missing_info = []
        
        if task.task_type == TaskType.NAVIGATE:
            if not task.target_location:
                missing_info.append("Where would you like to navigate to?")
        elif task.task_type == TaskType.RELOCATE_OBJECT:
            if not task.object_of_interest:
                missing_info.append("What object should I relocate?")
            if not task.source_location:
                missing_info.append("Where is the object currently located?")
            if not task.target_location:
                missing_info.append("Where should I move the object?")
        elif task.task_type == TaskType.INSPECT:
            if not task.object_of_interest:
                missing_info.append("What object should I inspect?")
            if not task.target_location:
                missing_info.append("Where is the object located?")
        elif task.task_type == TaskType.REPORT:
            if not task.object_of_interest:
                missing_info.append("What object should I report on?")
            if not task.source_location:
                missing_info.append("Where is the object located?")
            if not task.target_location:
                missing_info.append("Who should I report to?")
        elif task.task_type == TaskType.IDENTIFY:
            if not task.object_of_interest:
                missing_info.append("What object should I identify?")
            if not task.source_location or not task.target_location:
                missing_info.append("Where is the object located?")
        elif task.task_type == TaskType.MONITOR:
            if not task.object_of_interest:
                missing_info.append("What object should I monitor?")
            if not task.source_location or not task.target_location:
                missing_info.append("Where is the object located?")
        elif task.task_type == TaskType.ASSIST:
            if not task.object_of_interest:
                missing_info.append("What object should I assist with?")
            if not task.source_location or not task.target_location:
                missing_info.append("Where is the object located?")
        elif task.task_type == TaskType.FOLLOW:
            if not task.object_of_interest:
                missing_info.append("What object should I follow?")
            if not task.source_location:
                missing_info.append("Where is the object located?")
        else:
            missing_info.append("Task type not recognized")
        
        if missing_info:
            self.missing_info_questions[task_index] = missing_info
            return False
        return True

    def get_missing_info_questions(self, task_index: int) -> List[str]:
        return self.missing_info_questions.get(task_index, [])
