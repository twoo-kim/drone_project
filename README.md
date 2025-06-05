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
    1. controller 조정 및 점 개수 조정


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
      IMU bias state 추가  
      covariance 조정 중  
      ORB tracking잃었을 때 이전 pose사용 코드 추가했지만 효과 미미  
      April Tag 가깝지 않으면 값이 좋지 않음, 적당한 거리에서 적용되기 했지만 z방향으로 너무 진동함 적용 정도를 줄이거나 해야할 듯함



수정사항 있으면 README.md에 적어주세요.



