# drone project
EE478 term project

## Installation
```
git clone https://github.com/twoo-kim/drone_project.git
cd ~/catkin
catkin_make
```

## API test
First, get the OpenAI API key
```
export OPENAI_API_KEY="your-api-key"
```

```
roscore
rosrun gpt_api gpt_server.py
rosservice call /ask_gpt "question: 'your-question'"
```
