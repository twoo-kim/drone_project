# drone project
EE478 term project

## Goal
1. Gate position
    dict key: gate pair순서  item: left, right gate 위치
    현재는 gate pair위치만 찍어놓음
    sdf 참고해서 정확한 Left, Right 위치로 고쳐야 함

2. Goal publish
    현재는 각 QR에 대해 1번만 service call하도록 되어있음
    controller와 잘 맞춰서 수정

3. Openai api key를 github에 올리는게 거부돼서 비워뒀습니다.

4. 수정한 부분이나 개선할 부분이 있으면 source code 또는 Readme.md에 적어주세요 

CMakeLists.txt도 고쳐야 합니다

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
