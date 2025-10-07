import json
from typing import Dict, Any, Tuple, List
import os
from pathlib import Path



class RobotActionValidator:
    """
    A class to load robot function capabilities from a JSON file and validate
    LLM-generated actions against the defined schema.
    """

    def __init__(self):
        """Initializes the validator and loads the robot capabilities."""
        self.capabilities: Dict[str, Dict[str, Any]] = self._load_robot_capabilities()

    def _load_robot_capabilities(self) -> Dict[str, Dict[str, Any]]:
 

        try:
            # 1. Read and Parse the JSON file
            with open('robot_capacity.json', 'r') as file:
                data = json.load(file)
            
            # 2. Extract and transform the list into a dictionary (keyed by function name)
            capabilities_map = {}
            for func in data.get("robot_functions", []):
                if "name" in func:
                    capabilities_map[func["name"]] = func
            
            print(f"Loaded {len(capabilities_map)} robot functions successfully.")
            return capabilities_map
            
        except json.JSONDecodeError as e:
            print(f"FATAL: Could not parse robot capabilities JSON. Error: {e}")
            return {}
        except Exception as e:
            print(f"FATAL: Could not load robot capabilities. Error: {e}")
            return {}

    def validate_action(self, action_input: Dict[str, Dict[str, Any]]) -> Tuple[bool, str]:
        """
        Validates a single action object against the defined robot capabilities.
        The expected input structure is: { "function_name": { "arg1": val1, ... } }
        """
        if not self.capabilities:
            return False, "Validator not initialized: Capabilities not loaded."
        
        if not action_input or not isinstance(action_input, dict):
            return False, "Input must be a non-empty dictionary."
            
        if len(action_input) != 1:
            return False, f"Action must contain exactly one function name key, found {len(action_input)}."

        function_name = list(action_input.keys())[0] # The single key is the function name
        arguments = action_input[function_name]      # The value is the arguments dictionary

        # --- 1. Function Existence Check ---
        if function_name not in self.capabilities:
            return False, f"Unknown function '{function_name}'. Must be one of: {list(self.capabilities.keys())}"
            
        function_spec = self.capabilities[function_name]
        required_args_spec = function_spec.get("arguments", {})
        
        # --- 2. Argument Presence and Unnecessary Argument Check ---
        
        # Check for missing arguments
        for req_arg_name in required_args_spec:
            if req_arg_name not in arguments:
                return False, f"Function '{function_name}' is missing required argument '{req_arg_name}'."

        # Check for unexpected arguments
        for arg_name in arguments.keys():
            if arg_name not in required_args_spec:
                return False, f"Function '{function_name}' received unexpected argument '{arg_name}'."
                
        # --- 3. Argument Validation (Type, Value, Range) ---
        for arg_name, arg_value in arguments.items():
            arg_spec = required_args_spec[arg_name]
            arg_type = arg_spec.get("type")
            
            # Validate Type
            if arg_type == "string":
                if not isinstance(arg_value, str):
                    return False, f"Argument '{arg_name}' must be a string, got {type(arg_value).__name__}."
                
                # Validate Specific Values
                if "valid_values" in arg_spec:
                    valid_values_upper = [v.upper() for v in arg_spec["valid_values"]]
                    if arg_value.upper() not in valid_values_upper:
                        return False, f"Argument '{arg_name}' ('{arg_value}') is invalid. Must be one of: {arg_spec['valid_values']}."
            
            elif arg_type == "integer":
                # Check for integer type
                if not isinstance(arg_value, int):
                    return False, f"Argument '{arg_name}' must be an integer, got {type(arg_value).__name__}."
                
                # Validate Range
                min_val = arg_spec.get("minimum", float('-inf'))
                max_val = arg_spec.get("maximum", float('inf'))
                
                if arg_value < min_val:
                    return False, f"Argument '{arg_name}' ({arg_value}) is below minimum of {min_val}."
                if arg_value > max_val:
                    return False, f"Argument '{arg_name}' ({arg_value}) is above maximum of {max_val}."
                    
        return True, "Valid"

    def validate_kinematic_json(self, raw_json: str) -> Tuple[bool, str, List[Dict[str, Any]]]:
        """
        Parses a JSON string and validates it as a sequence of robot actions.
        NOTE: This function assumes the LLM output is a list of dictionary actions,
        WHERE EACH DICTIONARY CONTAINS: action_id, function_name, and arguments.
        It converts the structure from the LLM format to the simplified format
        expected by `validate_action` for the core validation step.
        """
        
        try:
            # Step 1: Parse the JSON string
            actions = json.loads(raw_json)
        except json.JSONDecodeError as e:
            return False, f"JSON Parsing Error: The output is not valid JSON. Error: {e}", []
        except TypeError:
            return False, "Input must be a JSON string.", []

        # Step 2: Check if the top level is a list (array)
        if not isinstance(actions, list):
            return False, f"JSON structure must be a top-level array ([]), got {type(actions).__name__}.", []
            
        if not actions:
            return True, "Valid: Empty action sequence.", []

        # Step 3: Validate each action sequentially
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                return False, f"Action at index {i} is not an object, got {type(action).__name__}.", []
            
            # A common LLM output format: { "action_id": "uuid", "function_name": "...", "arguments": {...} }
            if not all(k in action for k in ["action_id", "function_name", "arguments"]):
                return False, f"Action at index {i} missing required fields (action_id, function_name, arguments)."
                
            function_name = action["function_name"]
            arguments = action["arguments"]

            # --- Validation Core: Convert LLM format to simple {func_name: args} for internal validation ---
            simple_action_format = { function_name: arguments }
            
            is_valid, message = self.validate_action(simple_action_format)
            
            if not is_valid:
                return False, f"Validation failed for action {i} (ID: {action.get('action_id', 'N/A')}): {message}", []
                
        return True, f"Valid: {len(actions)} action(s) approved.", actions


if __name__ == "__main__":
    # Example usage for self-testing the class
    validator = RobotActionValidator()
    
    # 1. Test the core single action validator
    test_action = {"move_joint": {"joint_name": "RIGHT_ARM", "axis": "YAW", "degree": 45}}
    valid, msg = validator.validate_action(test_action)
    print(f"\nSingle Action Test (Valid): {valid} - {msg}")
    
    # 2. Test the full JSON sequence validator
    valid_json = """
    [
      { "action_id": "id_1", "function_name": "move_joint", "arguments": { "joint_name": "LEFT_ARM", "axis": "PITCH", "degree": 10 } },
      { "action_id": "id_2", "function_name": "express_emotion", "arguments": { "emotion": "SMILE" } }
    ]
    """
    valid, msg, actions = validator.validate_kinematic_json(valid_json)
    print(f"\nFull JSON Sequence Test (Valid): {valid} - {msg}")
    
    invalid_json = """
    [
      { "action_id": "id_1", "function_name": "unknown_func", "arguments": { "arg": 1 } }
    ]
    """
    valid, msg, actions = validator.validate_kinematic_json(invalid_json)
    print(f"\nFull JSON Sequence Test (Invalid): {valid} - {msg}")