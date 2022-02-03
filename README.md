# Welcome To Doge Driver Project!

## Formatter & Linter
### Install mypy & black for typing and reformatting
- vscode Ctrl+Shift+P > python select linter > mypy 선택 (우측하단에 설치하라고 하면 설치) (타입 지원위해서)
- vscode settings > python formatting provider black 선택 (우측하단에 설치하라고 하면 설치)
- vscode settings > format on save 체크

## 파일 설명
### Subscriber
`src/wecar_ros/scripts/Subscriber.py`: Wrapper for Subscriber
- 싱글톤으로 바꿔서 클래스 여러번 선언해도 인스턴스 하나임 (Instantiates only ONCE)

#### Example1
```python
camera = Camera()
camera.retrieve() # 카메라에서 데이터 가져옴 (반환: 현재 인스턴스)
camera.get() # 가져왔었던 데이터 반환 (CompressedImage 타입)
# camera는 편의를 위하여 getImage() retrieveImage() 로 바로 이미지데이터 반환

# 그니까 이런거 할필요없음
np_arr = np.frombuffer(self.get().data, dtype=np.uint8)
cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
```
#### Example2
```python
imu = IMU()
imu.retrieve() # IMU에서 데이터 가져오고 반환 (반환: 현재 인스턴스)
imu.get() # 가져왔었던 데이터 반환(Imu 타입)
imu.get().angular_velocity # 이렇게 바로 접근 가능
imu.retrieve().get() # IMU에서 데이터 새로 가져오고 가져온 데이터 받아옴
```
#### Example3
```python
# 싱글톤이라 아래 둘은 같은 인스턴스임
imu1 = IMU()
imu2 = IMU()
IMU().retrieve()
IMU().get()
# imu1이든 imu2든 IMU()든 인스턴스 주소값은 같음 (즉, 저장된 데이터값도 일치함)
```
### Utils
`src/wecar_ros/scripts/util_recorder.py`: 프레임 단위로 카메라에서 데이터 레코딩 (`src/wecar_ros/scripts/pickles` 폴더에 저장됨)
`src/wecar_ros/scripts/util_recordPlayer.py`: 프레임 단위로 레코딩된 데이터 재생

