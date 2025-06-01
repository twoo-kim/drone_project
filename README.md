# drone project
EE478 term project

## Goal
1. Goal selector  
    현재 pose 및 gpt답에 따라서 알맞은 gate와 방향 선택
    string으로 LEFT, RIGHT publish  
    GPIO, LED 코드만 수정하면 완료  

    방향 선택 및 planner까지 잘 작동하는 것 확인했습니다  

2. Controller  
    ground truth알고 있으므로 LEFT, RIGHT에 대해 최적화 경로 고려

   현재 최적화 경로 기반 planner, controller (z하고 yaw에 PI 적용) 완성해서 커밋했습니다.  
   Class로 정리하고 일부분 수정했습니다.  
   launch 실행 시 planner, controller 실행됩니다  

   
3. Localization  
    origin에서 기준 tag위치 (-0.075, 1.675, 0) 반영하여 ground truth 수정 완료  
    groun truth orientation도 알맞게 수정함
    
    이전 commit기준 orientation이 이상하게 나와서 다시 수정  
    초기에 나오는 큰 오류는 tag를 제대로 읽지 못해서 운행함에 따라 정확한 값을 보임
    Transformation을 적용하면 제대로 값이 나옴 (오해가 있었음)

    ORB, AprilTag, ekf_launch 실행 후 ekf_error 실행하여 오차 확인 가능합니다  
    covariance 조정 필요  
    
    ORB: 초기에만 좋음 / AprilTag: 초기에만 나쁨 진행하면서 ground truth에 매우 가까워짐  
    현재 ORB가 tracking을 잃으면 error에 비례하여 covariance를 조정하도록 했습니다  
    만약 tracking을 잃지 않더라도 Takeoff + AprilTage detection 이후 AprilTag와 차이가 매우 커지면 covariance를 조정해도 좋을 것 같네요  

    ekf_error 로 확인해본 결과 안정적으로 가려면 속도를 적당히 조절해야할 것 같습니다  
    또한, 아직 직접 조종해서인지 모르겠지만 gate를 통과할 때 AprilTag, ORB 모두 잃어버리는 경우가 생깁니다. IMU로 되긴 하겠지만 천천히 가야할 것 같네요 에러가 많이 커집니다.  
    
    (position error, orientation error)  
    안정적일 때: 0.2~0.6,  0.05 ~ 0.2   
    시작, line 상에서 빠르게 움직일 때: 0.5 ~ 1.5,   
    Gate에서 모두 잃었을 때: 1.0 ~ 6.0,   


현재 localization 조정 및 전체적인 코드 작동 확인을 해야할 것 같습니다  
아직 완전히 돌려본 건 아니라서 수정이 좀 필요합니다.  
시간나시면 covariance 조정 및 controller 수정 부탁드립니다

수정한 부분이나 개선할 부분이 있으면 source code 또는 Readme.md에 적어주세요 

## Run
AprilTag는 추가로 첨부한 tag.yaml 파일 복사해서 실행
ORB는 시뮬레이션 yaml로 조정 후 실행
GPT API의 경우 공용 key가 현재 막혀있는 듯함

Manual run
```
roslaunch ee478_px4_sim tag_spawn.launch

roslaunch apriltag_ros continuous_detection.launch
roslaunch orb_slam3_ros euroc_stereo.launch
roslaunch ekf_fusion ekf_node.launch

roslaunch qr_detector detector.launch
roslaunch gpt_api goal_selector.launch
roslaunch control plan_control.launch
```

Automate
```
tmuxp load ee478.yaml
```
Close each terminal or kill all
```
exit

tmux kill-server
```

