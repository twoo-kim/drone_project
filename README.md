# drone project
EE478 term project

## Quick Start
현재 공용 GPT API key 비활성화 ->  /goal topic에 수동으로 publish
```
rostopic pub -r 1 /goal std_msgs/String "RIGHT"

tmuxp load path/to/ee478.yaml
```
종료
```
tmux kill-server
```

## Goal
1. **Goal selector**  
    Call GPT service, publish LEFT, RIGHT or blink LED  

2. **Path Planner**  
    당장은 ground truth ```/mavros/local_position/pose```따라 가도록 해서 테스트 실행
    1. yaw check는 controller에서 하도록, 중복 확인 시 waypoint publish 너무 제한적임
    2. gate와 지나치게 가까워 짐. 1~1.5 이상에서 멈추는게 좋을 듯 함  
       path 새로 만들어야 할 것 같습니다
    3. 0.2~0.5 정도의 오차가 생길 수 있다는 점을 고려할 때, 그냥 중심을 통과하는게 안정적일 듯함. 현재 실행 시 안쪽 gate 테두리와 충돌

3. **Controller**  
    1. target pose와 current pose의 벡터 차를 통해 yaw를 계산하도록 수정
    2. K<sub>P</sub>, K<sub>I</sub> tuning 필요


3. **Localization**  
    1. EKF error check  
      ```tmuxp load``` 또는 px4, AprilTag, ORB 모두 실행 후
    ```
    rosrun ekf_fusion ekf_error
    ```
    2. EKF  
      covariance 조정 중  
      다만 AprilTag 및 ORB 특성 따라서 조금 더 dynamic하게 covarinace 조정 적용 예정


수정사항 있으면 README.md에 적어주세요.



