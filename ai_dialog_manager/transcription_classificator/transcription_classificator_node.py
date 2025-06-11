import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dialog_interfaces.msg import DialogState
import spacy
from langchain.prompts import PromptTemplate
from ros2_ollama_msgs.srv import RunPrompt
import os
import yaml

class TranscriptionClassificatorNode(Node):
    def __init__(self):
        super().__init__('ai_dialog_manager')

        # Load configuration with defaults
        default_prefix = "/transcription_classifier"
        default_config = {
            "llm_service_name": "/ollama_interface/run_prompt",
            "dialog_state_topic": "/dialog_state",
            "transcription_topic": "/transcription",
            "publish_topic_new_task": f"{default_prefix}/new_task",
            "publish_topic_clarification": f"{default_prefix}/clarification",
            "publish_topic_banter": f"{default_prefix}/banter",
            "transcription_limit": 50,
            "llm_client_timeout": 2.0,
        }

        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        self.get_logger().info(f"Looking for config.yaml at: {config_path}")
        user_config = {}
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as file:
                    user_config = yaml.safe_load(file) or {}
            except Exception as e:
                self.get_logger().error(f"Failed to load config.yaml: {e}")
        else:
            self.get_logger().warn(f"config.yaml not found at {config_path}. Using default parameters.")

        self.get_logger().info("Loading configuration...")
        self.get_logger().info(f"User config: {user_config}")
        self.get_logger().info(f"Default config: {default_config}")
        self.get_logger().info("Merging user config with defaults...")
        self.get_logger().info(f"Default prefix: {default_prefix}")
        self.get_logger().info("Final configuration:")
        self.get_logger().info(f"User config keys: {list(user_config.keys())}")
        self.config = {key: user_config.get(key, default) for key, default in default_config.items()}

        self.invalid_phrases = set()
        invalid_input_path = os.path.join(os.path.dirname(__file__), 'invalid_input.yaml')
        if os.path.exists(invalid_input_path):
            try:
                with open(invalid_input_path, 'r') as file:
                    phrases_data = yaml.safe_load(file)
                    self.invalid_phrases = set(p.lower().strip() for p in phrases_data.get('faulty_phrases', []))
                    self.get_logger().info(f"Loaded {len(self.invalid_phrases)} invalid phrases.")
                    if self.invalid_phrases:
                        self.get_logger().info(f"All invalid phrases: {', '.join(self.invalid_phrases)}")
            except Exception as e:
                self.get_logger().error(f"Failed to load invalid_input.yaml: {e}")
        else:
            self.get_logger().warn("invalid_input.yaml not found. No invalid phrases loaded.")

        self.dialog_state = "AWAITING_TASK"
        self.waiting_for_response = False

        self.nlp = spacy.load("en_core_web_sm")
        self.llm_service_name = self.config["llm_service_name"]
        self.llm_client = self.create_client(RunPrompt, self.llm_service_name)

        timeout = float(self.config.get("llm_client_timeout", 2.0))
        while not self.llm_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"Waiting for LLM service at {self.llm_service_name}...")

        self.transcription_sub = self.create_subscription(
            String,
            self.config["transcription_topic"],
            self.transcription_callback,
            10
        )

        self.dialog_state_sub = self.create_subscription(
            DialogState,
            self.config["dialog_state_topic"],
            self.dialog_state_callback,
            10
        )

        self.publisher_new_task = self.create_publisher(String, self.config["publish_topic_new_task"], 10)
        self.publisher_clarification = self.create_publisher(String, self.config["publish_topic_clarification"], 10)
        self.publisher_banter = self.create_publisher(String, self.config["publish_topic_banter"], 10)

        self.get_logger().info('AI Dialog Manager Node is ready.')

    def dialog_state_callback(self, msg: DialogState):
        self.dialog_state = msg.current_state
        self.get_logger().info(f"Dialog state updated: {self.dialog_state}")

    def transcription_callback(self, msg: String):
        if self.waiting_for_response:
            self.get_logger().info("Currently waiting for LLM response. Ignoring new transcription.")
            return

        user_input = msg.data.strip()
        self.get_logger().info(f"Received transcription: {user_input}")

        if not self.is_valid_text(user_input):
            self.get_logger().info("Classified as FAULTY_NOISE (invalid phrase filter)")
            return

        self.waiting_for_response = True

        if self.dialog_state == "AWAITING_TASK":
            doc = self.nlp(user_input)
            has_verb = any(token.pos_ == "VERB" for token in doc)
            if not has_verb:
                self.get_logger().info("Rejected by SpaCy: no verb detected")

                #check if it is more than three words
                if len(user_input.split()) > 3:
                    self.publisher_banter.publish(String(data=user_input))
                self.waiting_for_response = False
                return

            prompt = PromptTemplate(
                input_variables=["user_input"],
                template="""
                Determine if the text is a task request.

                Examples:
                - "Go to the kitchen and get me water" → Yes
                - "Could you open the window please" → Yes
                - "How's the weather today?" → No
                - "Tell me a joke" → No

                Now evaluate:
                Is the following text a request for someone or to do some task?
                '{user_input}'

                Answer only Yes or No.
                """
            )

            formatted_prompt = prompt.format(user_input=user_input)
            request = RunPrompt.Request()
            request.prompt_template = prompt.template
            request.keys = ['user_input']
            request.variables = [user_input]

            future = self.llm_client.call_async(request)
            future.add_done_callback(lambda f: self.publish_based_on_yes_no(f, user_input, target="task"))

        elif self.dialog_state == "AWAITING_CLARIFICATION":
            prompt = PromptTemplate(
                input_variables=["user_input"],
                template="""
                You are helping determine whether a piece of text contains relevant task information.

                Examples:
                - "It is in the garage next to the toolbox" → Yes
                - "The object is the blue crate" → Yes
                - "Hello again" → No
                - "I already told you everything" → No

                Now answer this:
                Does the following text contain anything related to an object, location, or a type of a task?
                '{user_input}'

                Answer only Yes or No.
                """
            )

            formatted_prompt = prompt.format(user_input=user_input)
            request = RunPrompt.Request()
            request.prompt_template = prompt.template
            request.keys = ['user_input']
            request.variables = [user_input]

            future = self.llm_client.call_async(request)
            future.add_done_callback(lambda f: self.publish_based_on_yes_no(f, user_input, target="clarification"))

        else:
            self.get_logger().info(f"Unhandled dialog state: {self.dialog_state}")
            self.waiting_for_response = False

    def is_valid_text(self, text: str) -> bool:
        return text.lower() not in self.invalid_phrases and len(text.strip()) > 0 and len(text.split()) < self.config["transcription_limit"]

    def publish_based_on_yes_no(self, future, original_text, target: str):
        try:
            result = future.result().result.strip().lower()
            self.get_logger().info(f"LLM {target} check response: {result}")
            msg = String()
            msg.data = original_text
            if "yes" in result:
                if target == "task":
                    self.publisher_new_task.publish(msg)
                    self.get_logger().info("Classified as NEW_TASK (yes response)")
                elif target == "clarification":
                    self.get_logger().info("Classified as CLARIFICATION (yes response)")
                    self.publisher_clarification.publish(msg)
            elif "no" in result:
                if self.dialog_state == "AWAITING_TASK":
                    self.get_logger().info(f"Classified as BANTER (no response in {target} mode)")
                    self.publisher_banter.publish(msg)
                elif self.dialog_state == "AWAITING_CLARIFICATION":
                    self.get_logger().info(f"Classified as BANTER (no response in {target} mode)")
                    # self.publisher_clarification.publish(msg)
            else:
                self.get_logger().warn(f"Unrecognized LLM response: '{result}' → sending to (not implemented) banter topic")
                self.publisher_banter.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process LLM response: {e}")
        finally:
            self.waiting_for_response = False


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionClassificatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
