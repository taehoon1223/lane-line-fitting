# ROS Lane Line Fitting (HSV + BEV + Polyfit)

HSV 색상 필터링 → Bird Eye View(원근변환) → 3차 다항식 피팅으로 **좌/우 차선의 모델식**(C3~C0), **곡률/반지름**, **각도**, **오프셋**을 실시간 산출하고 **ROS 토픽으로 publish**하는 노드입니다.

## ✨ 주요 기능
- 카메라 원본(`/camera/rgb/image_raw`) 구독
- 노란색(좌), 흰색(우) 차선 **HSV 범위 분리**
- BEV(원근 변환)로 직선/곡선 판단 용이
- `numpy.polyfit` 기반 3차 다항식 피팅 + **적응 차수 선택**
- 좌/우 차선 결과를 각각 `/lane_info/left`, `/lane_info/right` 에 `Float32MultiArray`로 publish
  - `[C3, C2, C1, C0, curvature, radius, angle_deg, offset]`

## 🧱 설치
ROS/`cv_bridge`/`rospy`는 apt로 설치하세요. pip 로는 아래만 설치합니다.
```bash
pip install -r requirements.txt
```
- ROS: Ubuntu/ROS Noetic 기준
- `cv_bridge`: `sudo apt install ros-noetic-cv-bridge`
- OpenCV/NumPy: `requirements.txt`

## ▶ 실행
### 1) roslaunch (catkin 패키지 내에서)
```
roslaunch lane_estimator.launch
```
> `lane_estimator.launch`는 이 레포의 스크립트 이름(`lane_detector_node_2022080.py`)을 그대로 사용합니다. 패키지 이름은 `remocar_lane_fitting`으로 가정했으니, 다른 이름이면 파일에서 `pkg`를 바꿔주세요.

### 2) rosrun (직접 실행)
```
chmod +x lane_detector_node_2022080.py
rosrun remocar_lane_fitting lane_detector_node_2022080.py
```

## 📡 토픽
- **입력**
  - `/camera/rgb/image_raw` (`sensor_msgs/Image`)
- **출력**
  - `/lane_info/left`  (`std_msgs/Float32MultiArray`)
  - `/lane_info/right` (`std_msgs/Float32MultiArray`)
  - 데이터: `[C3, C2, C1, C0, curvature, radius, angle_deg, offset]`

## 🧪 알고리즘 개요
1. **HSV 색상 필터링**
   - 노란색: (10, 80, 80) ~ (30, 255, 255)
   - 흰색:   (0, 0, 200) ~ (180, 50, 255)
   - 모폴로지 + 가우시안 블러로 노이즈 제거
2. **Bird Eye View (원근 변환)**
   - `getPerspectiveTransform` + `warpPerspective`
3. **3차 다항식 피팅**
   - `numpy.polyfit` 기반
   - C3~C0, **곡률(curvature=2*C2)**, **반지름(1/curvature)**, **각도(angle= C1 rad → deg)**, **오프셋(offset=C0)** 계산
4. **ROS Publish**
   - 좌/우 각각의 결과를 `Float32MultiArray`로 송신

## 📁 레포 구조
```
lane-line-fitting/
├── .gitignore
├── README.md
├── requirements.txt
├── images/
│   └── .gitkeep
├── lane_detector_node_2022080.py
├── lane_fitting.launch            # (원본)
└── lane_estimator.launch          # (이 레포용, 스크립트명 정합)
```

## ✔ 체크리스트
- [ ] `cv_bridge` 설치됨
- [ ] 카메라 토픽 이름 확인(`/camera/rgb/image_raw`)
- [ ] 스크립트에 실행 권한 부여: `chmod +x lane_detector_node_2022080.py`
- [ ] 패키지명 확인 후 `lane_estimator.launch`의 `pkg` 수정
