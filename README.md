# drone project
EE478 term project

## Goal
1. Gate position
    dict key: gate pair순서  item: left, right gate 위치
    현재는 gate pair위치만 찍어놓음
    sdf 참고해서 정확한 Left, Right 위치로 고쳐야 함

   april tag 좌표 값 기준으로 Localization 하는 코드 만들겠습니다.
   
2. Localization
    apriltag에서 pose는 camera link 기준 bundle의 pose를 말합니다
    즉 T_cb 를 의미하기때문에 T_wc (camera in world frame)을 얻기 위해선
    T_wc = T_wt * T_tb * T_bc 를 해줘야합니다

    이때 T_tb는 각 gatepair마다 기준점을 잡도록했기때문에 무시하고 T_wt는
    기준 tag의 world frame 상 pose입니다. 따라서
    T_wc = T_wt * (T_cb)^-1 로 구할 수 있습니다

    추가로 센서의 covariance 정도에 따라 어떤 값을 중요하게 다룰 지 조정하는
    Kalman Filter 노드를 만들었습니다.
    IMU 센서로 위치를 추정하고 ORB, AprilTag가 들어올 때 조정할 수 있게
    만들었습니다. (AprilTag frame 기준을 map으로 보기때문에 수정하거나 무시)

    실행은 되는데 위치가 이상하게 나오네요 transform 생각해서 apriltag_tf.cpp 수정 필요


3. Goal publish
    현재는 각 QR에 대해 1번만 service call하도록 되어있음
    controller와 잘 맞춰서 수정

4. Openai api key를 github에 올리는게 거부돼서 비워뒀습니다.

5. 수정한 부분이나 개선할 부분이 있으면 source code 또는 Readme.md에 적어주세요 

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
