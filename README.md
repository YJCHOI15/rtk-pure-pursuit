# Dual RTK-GPS 기반 고정밀 Waypoint 추종 알고리즘

## 프로젝트 개요
두 개의 RTK GPS 모듈을 사용하여 센티미터 수준의 정밀도로 Waypoint를 추종하는 자율주행 차량의 알고리즘과 구현 코드를 다룹니다. 

국토지리정보원의 NTRIP 서비스를 이용한 RTK 보정과 오직 두 GPS 수신기의 위치 정보만으로 로봇의 방위각을 계산하여 경로를 추종하는 것이 목표입니다. 

가장 널리 사용되는 경로 추종 알고리즘 중 하나인 'Pure Pursuit' 알고리즘을 적용하여 부드럽고 안정적인 주행을 구현합니다. 

ROS1을 기반으로 각 기능을 노드 단위로 분리하여 개발 및 유지보수가 용이하도록 했습니다. 

### 개발 기간

2024.08.01 ~ 2024.08.20

---

## 하드웨어 구성

* 로봇 플랫폼: 하네스 1/5 차량
* 메인 컨트롤러: gram 노트북 (8GB, Ubuntu 20.04)
* RTK GPS 모듈 (2개): FST-UEF9P
* RTK 기준국: [대구 기준국 TEGN-RTCM31](https://gnssdata.or.kr/cors/getCorsView.do)
* 모터 드라이버: Arduino Mega 2560

### 안테나 배치

정확한 방위각 계산을 위해 두 개의 GPS 안테나는 로봇의 진행 방향 축과 정확히 일치하도록 앞뒤로 배치해야 합니다. 두 안테나 간의 **거리가 멀수록** 방위각 계산의 정밀도가 향상됩니다. 

--- 

## 1. Waypoint 생성: 

웨이포인트는 보통 UTM 좌표계(X, Y)와 해당 지점에서의 목표 속도로 구성된 목록입니다.

웨이포인트를 생성하는 가장 직관적인 방법은 수동으로 로봇을 주행시키며 경로를 기록하는 것입니다.

데이터 기록: 로봇을 수동으로 조종하면서, 일정한 간격(예: 0.5초 또는 1m)마다 현재 RTK-GPS의 UTM 좌표(x, y)와 속도를 CSV 파일과 같은 텍스트 파일에 저장합니다.

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/8128919f-8c34-4304-89ec-840480ad21e2" />


---

## 2. 듀얼 GPS를 이용한 Heading 계산

IMU의 누적 오차 문제를 해결하기 위해, 차량의 앞쪽과 뒤쪽에 장착된 두 RTK GPS의 절대 좌표를 이용합니다.

1. **좌표 수신:** 전방 GPS(GPS_F)와 후방 GPS(GPS_R)로부터 각각의 위도, 경도 좌표를 수신합니다.

2. **좌표 변환:** 수신된 WGS84 좌표(위도, 경도)를 로컬 평면 좌표계인 UTM-K 좌표(x, y)로 변환합니다.

3. **방위각 계산:** 두 점의 좌표를 이용하여 로봇의 현재 방위각(Yaw, θ)을 계산합니다. atan2 함수를 사용하면 360도 모든 방향에 대해 정확한 각도를 얻을 수 있습니다.

    <img width="606" height="69" alt="image" src="https://github.com/user-attachments/assets/aef11b9c-b83b-4cda-afb4-95b642f5a0eb" />

로봇의 현재 위치는 두 GPS 좌표의 **중간 지점**으로 설정하거나, **후방 GPS의 위치**를 기준으로 사용할 수 있습니다.

---

## 3. Pure Pursuit 알고리즘

계산된 현재 위치와 방위각을 바탕으로 미리 설정된 경로(Waypoints)를 따라가기 위해 Pure Pursuit 알고리즘을 사용합니다.

1. **목표 지점 탐색:** 

    * 로봇의 현재 위치로부터 **'전방 주시 거리(Look-ahead distance)'** 만큼 떨어진 경로 상의 점을 목표 지점으로 설정합니다.

2. **조향각 계산:**

    * 로봇이 목표 지점을 향해 나아가기 위해 필요한 곡률(κ)과 조향각(δ)을 계산합니다. 이는 로봇의 현재 방위각과 목표 지점까지의 각도 차이(α)를 이용하여 계산됩니다.

    * 조향각 공식:
  
      <img width="384" height="60" alt="image" src="https://github.com/user-attachments/assets/3a1fbd38-b75a-4061-93d9-92104e808bcb" />

        * L: 로봇의 축간거리(Wheelbase)
        * α: 로봇의 현재 방위각과 목표 지점을 바라보는 각도 사이의 차이
        * L_d: 전방 주시 거리

3. **모터 제어:**

    * 계산된 조향각에 맞춰 좌우 바퀴의 속도를 제어하여 로봇이 목표 지점으로 부드럽게 회전하도록 합니다.

---

## 사용 방법

**레포지토리 클론 및 빌드:** 
```
cd ~/catkin_ws/src
git clone https://github.com/your_username/rtk_pure_pursuit.git
cd ..
catkin_make
source devel/setup.bash
```

**RTK 기준국을 먼저 활성화하여 보정 데이터가 송출되도록 합니다.**

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/ba5d8697-1f9b-48a8-9e7a-108d0df67936" />


**아래 명령어를 통해 전체 시스템 실행:**
```
roslaunch rtk_pure_pursuit follow_waypoints.launch
```
