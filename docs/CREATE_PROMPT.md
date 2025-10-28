# How to add prompt?

## 1. In the file `prompts_ros2.py` add class with the name connected to your prompt:
```python
class YourPromptName(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            name="name-of-the-prompt",
            description=(
                "Description for user, what this prompt does "
            ),
            args=[
                prompthandler.ArgSpec(
                    "argument_x", "Argument x description", True, "number" # "name_of_the_argument", "Description of the argument for user", (If argument is required: True or false),  Type of the argument
                ),
                prompthandler.ArgSpec(
                    "argument_y", "Argument y description", True, "string"
                ),
                prompthandler.ArgSpec(
                    "dest_z", "Target Z (meters relative to HOME, MAV_FRAME=3)", True, "number"
                ),
            ],
            messages_template=[
                ("assistant",
                 "Message for AI, what should do"
                ),
                ("user",
                 "Passed data from user: x={argument_x}, y={argument_y}; "
                )
            ],
        )
```

## 2. In the file `server.py` add your prompt to server prompts:
```python
# Place for adding prompts
add_prompt_handler(prompts_ros2.YourPromptName())
```

## 3. [Rebuilt MCP image after adding new changes](../installation/README.md#build-docker-image-locally)


