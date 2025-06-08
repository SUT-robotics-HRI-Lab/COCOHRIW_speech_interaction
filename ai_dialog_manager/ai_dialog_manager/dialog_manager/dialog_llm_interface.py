# dialog_llm_interface.py

import rclpy
from rclpy.node import Node
from langchain.prompts import PromptTemplate
from ros2_ollama_msgs.srv import RunPrompt
from typing import Callable, Optional
from task_extractor.task_types import TaskInformation
from pydantic import TypeAdapter
import json

class DialogLLMInterface:
    def __init__(self, node: Node, service_name: str):
        self.node = node
        self.cli = node.create_client(RunPrompt, service_name)

        if not self.cli.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"LLM service not available: {service_name}")

    def _build_clarification_prompt(self, clarification_sentence: str) -> str:
        template = PromptTemplate(
            input_variables=["clarification_sentence"],
            template="""
            You are an assistant helping a user complete a task.
            Based on the following missing information:
            "{clarification_sentence}"

            Please generate a polite and clear single-sentence question to ask the user to provide the missing information.
            Your output should sound natural and helpful.
            Do not include any emojis.

            Example:
            Missing info: The task is missing the object to relocate and the target location.
            Output: Could you please tell me which object I should relocate and where to?

            Now respond to this:
            Missing info: {clarification_sentence}
            Output:
            """
        )
        return template.format(clarification_sentence=clarification_sentence)

    def _build_extraction_prompt(self, clarification_input: str, missing_fields: list[str]):
        field_hint = {
            "object_of_interest": "Extract the object(s) mentioned.",
            "source_location": "Find the location(s) where the object currently is.",
            "target_location": "Identify where the object should go.",
            "task_type": "Infer the type of task if mentioned."
        }
        hints = [field_hint.get(f, f) for f in missing_fields]
        hint_text = "\n".join(f"- {hint}" for hint in hints)
        missing_details = ", ".join(missing_fields)

        prompt = PromptTemplate(
            input_variables=["missing_details", "clarification_input", "hint_text"],
            template="""
            You are helping fill in these missing information for a task.
            Missing details: "{missing_details}"
            User clarification: "{clarification_input}"
            Task requirements:
            {hint_text}

            Extract the missing details and return them in the following JSON format:
            {{
            "object_of_interest": ["Object1", "Object2"],
            "source_location": ["source_location1", "source_location2"],
            "target_location": ["target_location1", "target_location2"],
            "task_type": "<task_type>"
            }}
            Only include fields that can be reasonably filled in.
            The task type should be one of the following: Relocate_Object, Pick, Place, Navigate, Inspect, Find, Monitor, Report, Follow.
            Do not include explanations.
            Response:
            """
        )
        return prompt, {
            "missing_details": missing_details,
            "clarification_input": clarification_input,
            "hint_text": hint_text
        }

    def build_banter_while_idle_prompt(self, banter_input: str) -> str:

        prompt = PromptTemplate(
            input_variables=["banter_input"],
            template="""
            The user said: "{banter_input}"

            You are a helpful assistant waiting for a task. Respond politely and naturally in 1–2 sentences.
            Acknowledge the user's message and gently remind them that you are waiting for a task to complete.
            Do not ask follow-up questions. Do not invent tasks. Avoid sarcasm. Be friendly and professional.

            Example outputs:
            - \"That's interesting! Let me know when you're ready to give me a task.\"
            - \"Haha, good one! If you have a task for me, I'm ready.\"


            Do not include any emojis.
            Your response:
            """
        )
        return prompt.format(banter_input=banter_input)

    def request_clarification_text(self, clarification_sentence: str, callback: Callable[[str, Exception], None]):
        prompt_text = self._build_clarification_prompt(clarification_sentence)

        req = RunPrompt.Request()
        req.prompt_template = prompt_text
        req.keys = []
        req.variables = []

        future = self.cli.call_async(req)

        def _internal_callback(fut):
            try:
                result = fut.result().result.strip()
                callback(result, None)
            except Exception as e:
                callback("", e)

        future.add_done_callback(_internal_callback)

    def request_task_field_extraction(self, clarification_input: str, missing_fields: list[str], callback: Callable[[Optional[TaskInformation], Exception], None]):
        prompt, variables = self._build_extraction_prompt(clarification_input, missing_fields)

        req = RunPrompt.Request()
        req.prompt_template = prompt.template
        req.keys = list(variables.keys())
        req.variables = list(variables.values())

        future = self.cli.call_async(req)

        def _internal_callback(fut):
            try:
                result_text = fut.result().result.strip()

                # Try to extract valid JSON object from the text
                start = result_text.find("{")
                end = result_text.rfind("}") + 1
                if start == -1 or end == -1 or end <= start:
                    raise ValueError("No valid JSON object found in response.")

                json_chunk = result_text[start:end]

                # Handle both "null" (string) and null (actual JSON null)
                # Replace "null" (string) with null (JSON null)
                cleaned_json = json_chunk.replace('"null"', 'null')

                # Parse JSON
                parsed = json.loads(cleaned_json)

                # Optionally, convert any actual None values in lists to empty lists
                for key in ["object_of_interest", "source_location", "target_location"]:
                    if key in parsed:
                        if parsed[key] is None:
                            parsed[key] = []
                        elif isinstance(parsed[key], list):
                            parsed[key] = [item for item in parsed[key] if item is not None]

                task_data = TypeAdapter(TaskInformation).validate_python(parsed)
                callback(task_data, None)
            except Exception as e:
                self.node.get_logger().error(f"Failed to extract task fields from LLM response: {e}")
                callback(None, e)

        future.add_done_callback(_internal_callback)
