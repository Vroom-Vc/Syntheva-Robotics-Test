# Fine-Tuning Data  Strategy
The goal of this fine-tuning effort is to train a base LLM to act as the "Language Transformer," converting natural language into a highly constrained, valid JSON array of robot actions. In case the prompt engineering method result is not well.

## 1.Method
It looks like the most effective way is Synthetic Generation and Reflexive Validation Loop. Since the knowledge base (the robot functions and constraints) is entirely internal to syntheva system,  must generate all examples based on those known rules.

Step 1:Define Scenario/User Intent: Create a diverse natural language command (e.g., "Tell me a joke and look down.").(again, probably use some llm model)

Step 2: base on the commands form step 1, manually or with custom scirpt(?) to generate the correct JSON array of action. to be safe, we can run the generated json through the validator class (in json_validator file)

example: [{"action_id": "uuid_1", "function_name": "say_text", "arguments": {"text": "..."}}, {"action_id": "uuid_2", "function_name": "move_joint"..}]

Step 3: the (Prompt, Completion) pair is ready, use it to fine tune LLM model

## 2.Training data template
Look like most LLM fine-tuning APIs require data in JSON which is good since the training data is in json. To ensure the model learns the rules and the output format, the full System Instruction should add the user command in the prompt field for every example.

prompt(String (Full Context)): [BEGIN SYSTEM PROMPT]\n<Full Instruction like: the role to convert a user's fuzzy, natural-language command into a structured, machine-readable JSON array of action sequences...>\n[END SYSTEM PROMPT]\n\nUSER COMMAND: Lift your right arm straight up.

completion (String (Raw JSON Array)): [\n  {\n    "action_id": "uuid_1",\n    "function_name": "move_joint",\n    "arguments": {\n      "joint_name": "RIGHT_ARM",\n      "axis": "SHOULDER_LIFT",\n      "degree": 105\n    }\n  }\n] ...
