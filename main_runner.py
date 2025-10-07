import json
import time
import requests
from typing import List, Dict, Any, Tuple, Optional
from json_validator import RobotActionValidator

# --- CONFIGURATION (Match your environment) ---
CAPABILITIES_FILENAME = 'robot_capacity.json'
PROMPT_TEMPLATE_FILENAME = 'prompt_template.txt' # Assuming your prompt is saved here
MAX_RETRIES = 3

def load_prompt_template() :
    """ Loads the content of the prompt template file. """
    try:
        # Assuming prompt_template.txt is in the same directory
        with open('prompt_template.txt' , 'r') as f:
            return f.read()
    except Exception as e:
        print(f"Error loading prompt template from file: {e}")
        # Fallback instruction if the file is missing
        return f

def build_full_system_prompt(base_template: str, capabilities_json: str) -> str:
    """ Assembles the system instruction from the template and capability JSON. """
    
    system_prompt = (
        f"{base_template}\n\n"
        f"AVAILABLE ROBOT FUNCTIONS (STRICTLY ADHERE TO THIS SCHEMA):\n"
        f"```json\n{capabilities_json}\n```\n\n"
        "Your output MUST be a JSON array of actions based ONLY on these functions."
    )
    return system_prompt

def simulate_gemini_call( 
    full_prompt: str,  
) -> str: 
    api_key = "sth" 
    API_URL = "https://catgpt or whatever"
    headers = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {api_key}"
}
    payload = {
    "model": "gpt-3.5-turbo",
    "messages": [
        {
            "role": "user", #
            "content": full_prompt
        }
        
    ],
   
    "temperature": 0.7   
}
    try:
        response = requests.post(API_URL, headers=headers, json=payload)


        response.raise_for_status()
     
        response_data = response.json()

        if response_data.get("choices"):
            print("response：", response_data["choices"][0]["message"]["content"])
        else:
            print("no choices is found")

    except requests.exceptions.RequestException as e:
        print(f"error{e}")


def call_gemini_api_with_retry( 
    user_query: str,  
    max_retries: int = MAX_RETRIES 
) -> Optional[List[Dict[str, Any]]]: 
    """ 
    Manages the LLM communication, validation, and self-correction loop.
    """ 
    global VALIDATOR 


    # Load prompt template
    base_template = load_prompt_template()
    try:
        base_template=open('prompt_template.txt')
        with open ('robot_capacity.json' , 'r') as file:
            capabilities_json = json.load(file)
    except Exception as e:
        print(f"Error loading prompt template from file: {e}")
        # Fallback instruction if the file is missing
        return False
    
    # Build the full, stable system instruction used for all attempts
    system_instruction_block = build_full_system_prompt(base_template, capabilities_json)
    
    # Initial full prompt: System Instruction + User Query
    # This structure ensures the core instruction and capability definitions are always present.
    current_prompt = f"{system_instruction_block}\n\nUSER COMMAND: {user_query}"
    
    final_actions = None
    
    for attempt in range(max_retries): 
        delay = 5  
        
        # --- API Call (Simulated) --- 
        raw_json_output = simulate_gemini_call(current_prompt) 
        
        # --- Validation Step --- 
        is_valid, validation_msg, validated_actions = VALIDATOR.validate_kinematic_json(raw_json_output) 
        
        if is_valid: 
            print(f"SUCCESS function call") 
            final_actions = validated_actions
            break # Exit the loop on succes
        
        if attempt < max_retries - 1: 
            print(f"  -> Retrying prompt") 
            time.sleep(delay) 

            # Build the self-correction instruction
            correction_instruction = ( 
                f"\n\n-- CORRECTION INSTRUCTION () --\n" 
                "The previous output failed validation. You MUST correct the errors and re-generate the JSON array.\n" 
                f"Previous validation error: {validation_msg}\n" 
                f"Original Command: {user_query}\n" 
                "You must strictly adhere to the capabilities defined in the schema above."
            ) 
            
            # The next prompt is the original system block + correction context
            current_prompt = f"{system_instruction_block}{correction_instruction}\n\nUSER COMMAND: {user_query}"

        else: 
            print("MAX RETRIES REACHED. Command failed validation.") 
            break # Exit the loop after max attempts

    return final_actions 

# --- Main Execution --- 

def main():
    global VALIDATOR
    
    # 1. Initialize Global Assets
    VALIDATOR = RobotActionValidator() 
  

    command_1 = "i want to look to my right" 
    final_actions_1 = call_gemini_api_with_retry(command_1)
    if final_actions_1:
        print("success")
    
    else:
        print("failed.")

if __name__ == "__main__":
    main()