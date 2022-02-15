# Welcome To Doge Driver Project!

## Formatter & Linter
### Install mypy & black for typing and reformatting
- vscode Ctrl+Shift+P > python select linter > mypy 선택 (우측하단에 설치하라고 하면 설치) (타입 지원위해서)
- vscode settings > python formatting provider black 선택 (우측하단에 설치하라고 하면 설치)
- vscode settings > format on save 체크

## 파일 설명

### Vehicle.getLane(angle, findRange)
angle 타입: `Vehicle.LEFT_LANE_ANGLE`, `Vehicle.FRONT_LANE_ANGLE`, `Vehicle.RIGHT_LANE_ANGLE`
findRange: 기본값=0.7 (시뮬레이터 좌표 기준 탐지할 최대거리)
return: LaneType enum 클래스에 해당하는 value
웬만하면 쓸일없음

### Vehicle.getFrontLane()
차량중앙점기준 전방 0.7m 안에 도로선 검출
return: LaneType enum 클래스에 해당하는 value

### Vehicle.getLeftLane()
차량중앙점기준 좌방 0.4m 안에 도로선검출
return: LaneType enum 클래스에 해당하는 value

### Vehicle.getRightLane()
차량중앙점기준 우방 0.4m 안에 도로선검출
return: LaneType enum 클래스에 해당하는 value

#### Example
```python
Vehicle.getLeftLane()
# 만약 좌측 레인이 점선이면 LaneType.DOT.value 반환함
Vehicle.getRig음tLane()
# 만약 우측 레인이 실선이면 LaneType.EDGE.value 반환함
Vehicle.getRightLane()
# 만약 정방향 레인이 정지선이면 LaneType.STOP.value 반환함
```
### Utils (archived)
`src/wecar_ros/scripts/archive/util_recorder.py`: 프레임 단위로 카메라에서 데이터 레코딩 (`src/wecar_ros/scripts/pickles` 폴더에 저장됨)
`src/wecar_ros/scripts/archive/util_recordPlayer.py`: 프레임 단위로 레코딩된 데이터 재생

