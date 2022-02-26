# Capstone Design  (On Progress)
2022년 김종형 교수님 캡스톤 A팀 깃허브 레포지토리.

# Abstract
전용 트레이에 올라간 음식을 로봇팔을 이용해 모바일 로봇으로 이송하고, 모바일 로봇이 테이블에 이를 서빙 및 회수하는 시스템.

# Structure
## S/W
ROS환경(Ubuntu 18.04, ROS melodic)에서 진행됨. 노드 구성은 로봇팔을 제어하는 부분과 모바일 로봇을 제어하는 부분으로 크게 2부분으로 나눌 수 있다. 
### Collaborative Robot Node
- dsr_robot_arm : 실제 로봇에 명령을 전달하는 노드. 
- motion_planner : realsense, yolo 노드로부터 가져온 데이터를 처리해서 로봇이 이동할 3차원 좌표를 계산.
- teleop : 테스트를 위한 노드. 방향키로 이동힐 좌표를 전달.
- realsense : Intel Realsense의 2D 이미지 전달.
- yolo : Camera의 2D 이미지에서 Yolo를 통해 음식 인식.
### Mobile Robot Node
- turtlebot : robotis turtlebot3의 노드. 
- cctv_slam : Cctv에서 들어오는 정보를 통해 SLAM map 생성.
- yolo : 실시간으로 사람(장애물)의 위치 파악, map에 반영할 수 있도록 함.
![node구조](https://user-images.githubusercontent.com/77828741/155833741-2c0a7c01-72da-4519-ad6b-cfd32dbffa9a.png)

## H/W
- Doosan Robotics Collaborative Robot
- 자체 제작 End-Effector
- Intel Realsense
- Camera(for yolo on end-effector)
- Robotis Turtlebot3 기반 Mobile Robot
- Camera 2ea(for cctv)

# Reference
ROS Melodic. (http://wiki.ros.org/melodic/Installation/Ubuntu)
DoosanRobotics Collaborative Robot. (https://github.com/doosan-robotics/doosan-robot)
Robotis Turtlebot3. (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
네이버 카페 오로카. (https://cafe.naver.com/openrt)
Yolov3. (https://github.com/pjreddie/darknet)

