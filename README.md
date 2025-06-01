# drone project
EE478 term project

## Goal
1. Goal selector  
    현재 pose 및 gpt답에 따라서 알맞은 gate와 방향 선택
    string으로 LEFT, RIGHT publish할 예정

    web search 다른 종류의 API를 사용해야 해서 구조도 달라지네요 상황보고 적용해야할 것 같습니다

2. Controller  
    ground truth알고 있으므로 LEFT, RIGHT에 대해 최적화 경로 고려

   현재 최적화 경로 기반 planner, controller (z하고 yaw에 PI 적용) 완성해서 커밋했습니다.
   
4. Localization  
    origin에서 기준 tag위치 (-0.075, 1.675, 0) 반영하여 ground truth 수정 완료  
    groun truth orientation도 알맞게 수정함
    
    이전 commit기준 orientation이 이상하게 나와서 다시 수정  
    초기에 나오는 큰 오류는 tag를 제대로 읽지 못해서 운행함에 따라 정확한 값을 보임
    Transformation을 적용하면 제대로 값이 나옴 (오해가 있었음)

    ORB랑 같이 EKF test 예정


Contributor가 되면 권한이 생기는줄 알았어요. push권한 부여했습니다
수정한 부분이나 개선할 부분이 있으면 source code 또는 Readme.md에 적어주세요 
