import json
import re
from typing import List

from langchain_ollama import OllamaLLM
from langchain.prompts import PromptTemplate
from pydantic import TypeAdapter
from task_types import TaskInformation

class TaskExtractor:
    def __init__(self):
        self.llm = OllamaLLM(model="gemma2:2b", temperature=0.1)
        self.prompt = PromptTemplate(
    input_variables=["user_input"],
    template="""
        As an intelligent assistant, break down each part of the user's request into distinct tasks. Each task should be identified as one of the following types:
        - Relocate_Object, Navigate, Inspect, Identify, Monitor, Assist, Report, Follow.

        **Important**: Always treat each command as a separate task. For instance, if the user requests the robot to "go to" or "navigate to" a location, treat this as a distinct "Navigate" task before any other actions at that location. Additionally, if a task involves multiple objects or multiple locations, include all relevant objects and locations as lists.


        Each task should be strictly outputed only in the following JSON format:
        {{
            "object_of_interest": ["<object1>", "<object2>", ...],  # Include multiple objects if applicable
            "task_type": "<task from list above>",
            "source_location": ["<source1>", "<source2>", ...],    # Include multiple source locations if applicable
            "target_location": ["<target1>", "<target2>", ...],    # Include multiple target locations if applicable
            "context": "<context>"
        }}

        The "context" field should capture the specific part of the user request describing each task.

        For multiple tasks, return a list of JSON objects. Here are some examples:

        Example 1:
        User Request: "Go to the pantry, pick up a bottle of water and a snack, take them to the gym, and place them on the shelf and bench."
        Output:
        [
            {{
                "object_of_interest": null,
                "task_type": "Navigate",
                "source_location": null,
                "target_location": ["pantry"],
                "context": "Go to the pantry"
            }},
            {{
                "object_of_interest": ["bottle of water", "snack"],
                "task_type": "Relocate_Object",
                "source_location": ["pantry"],
                "target_location": ["gym"],
                "context": "pick up a bottle of water and a snack and take them to the gym"
            }},
            {{
                "object_of_interest": ["bottle of water", "snack"],
                "task_type": "Relocate_Object",
                "source_location": ["gym"],
                "target_location": ["shelf", "bench"],
                "context": "place them on the shelf and bench"
            }}
        ]

        Example 2:
        User Request: "Go to the office and garage, inspect the printer and toolbox, and report back to me."
        Output:
        [
            {{
                "object_of_interest": null,
                "task_type": "Navigate",
                "source_location": null,
                "target_location": ["office", "garage"],
                "context": "Go to the office and garage"
            }},
            {{
                "object_of_interest": ["printer", "toolbox"],
                "task_type": "Inspect",
                "source_location": ["office", "garage"],
                "target_location": null,
                "context": "inspect the printer and toolbox"
            }},
            {{
                "object_of_interest": ["printer", "toolbox"],
                "task_type": "Report",
                "source_location": ["office", "garage"],
                "target_location": ["me"],
                "context": "report back to me"
            }}
        ]

        Do not include the examples in the output. Only provide the JSON-formatted tasks based on the user's request.

        User Request: "{user_input}"
        Output:
        """
        )

    def extract_tasks(self, user_input) -> List[TaskInformation]:
        formatted_prompt = self.prompt.format(user_input=user_input)
        response = self.llm.invoke(formatted_prompt, temperature=0.1)
        json_match = re.search(r'\[.*\]', response, re.DOTALL)
        
        if not json_match:
            print("Error: Could not find JSON array in the response.")
            return []

        cleaned_response = json_match.group(0).replace('"null"', 'null')
        try:
            task_data = json.loads(cleaned_response)
            return TypeAdapter(List[TaskInformation]).validate_python(task_data)
        except Exception as e:
            print("Error parsing response as JSON:", e)
            return []
