# drone project
EE478 term project

## Goal
1. Goal selector  
    현재 pose 및 gpt답에 따라서 알맞은 gate와 방향 선택
    string으로 LEFT, RIGHT publish할 예정

2. Controller  
    ground truth알고 있으므로 LEFT, RIGHT에 대해 최적화 경로 고려

3. Localization  
    p_wc, R_wc를 frame에 맞게 변환 후 publish  
    EKF; covariance에 따라 imu prediction 수정

    origin에서 기준 tag위치 (-0.075, 1.675, 0) 반영하여 ground truth 수정 완료

    ORB랑 같이 EKF test 예정


수정한 부분이나 개선할 부분이 있으면 source code 또는 Readme.md에 적어주세요 
