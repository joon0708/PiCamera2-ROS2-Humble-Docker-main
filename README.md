# PiCamera2 ROS2 카메라 노드

# PiCamera2 ROS2 카메라 노드

PiCamera2를 수정하여 압축된 이미지를 전송하는 ROS2 카메라 노드입니다.

## 🚀 주요 기능

- **JPEG 압축 전송**: 대역폭 절약을 위한 압축 이미지 지원
- **색상 정확성**: 정확한 RGB 색상 채널 처리
- **유연한 설정**: 해상도, 프레임레이트, 품질 조정 가능
- **색상 검증**: 디버그 모드로 색상 정확성 확인

## 📖 사용법

### JPEG 압축 전송 (권장)
```bash
python3 main.py --resolution 1280x720 --downscale 0.5 --rate 10 --compressed --jpeg_quality 70 --best_effort --topic /camera/image_raw
```

### 색상 확인
색상이 정확한지 확인하려면 `--debug_colors` 옵션을 추가하세요:
```bash
python3 main.py --resolution 1280x720 --downscale 0.5 --rate 10 --compressed --jpeg_quality 70 --best_effort --topic /camera/image_raw --debug_colors
```

**디버그 모드에서 표시되는 색상 박스**:
- **빨간색 박스** (BGR: 0,0,255): 카메라 원본 출력 확인
- **파란색 박스** (BGR: 255,0,0): 색상 채널 순서 확인  
- **녹색 박스** (BGR: 0,255,0): 추가 색상 검증

**정상적인 색상 표시**:
- 빨간색 박스가 정확한 빨간색으로 보이면 색상 채널이 올바르게 처리되고 있습니다
- 만약 빨간색 박스가 파란색으로 보인다면 색상 채널이 뒤바뀌어 있는 것입니다

## ⚙️ 명령행 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--resolution` | `640x480` | 해상도 (WIDTHxHEIGHT) |
| `--rate` | `10.0` | 발행 속도 (Hz) |
| `--topic` | `/camera/image_raw` | 기본 토픽 이름 |
| `--compressed` | `False` | CompressedImage (JPEG) 발행 |
| `--jpeg_quality` | `80` | JPEG 품질 (1-100) |
| `--downscale` | `1.0` | 크기 조정 비율 |
| `--best_effort` | `False` | BEST_EFFORT QoS 사용 |
| `--debug_colors` | `False` | 색상 검증 박스 표시 |

## 📡 ROS2 토픽

### 발행 토픽
- `/camera/image_raw` (sensor_msgs/Image): 원본 RGB 이미지
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage): JPEG 압축 이미지

## 🔧 문제 해결

### 색상 문제
피부색이 파란색으로 보이는 경우:
```bash
# 디버그 모드로 색상 확인
python3 main.py --debug_colors
```

**정상적인 색상 표시**:
- 빨간색 박스: 정확한 빨간색
- 파란색 박스: 정확한 파란색
- 피부색: 자연스러운 살구색/황갈색

## 📁 프로젝트 구조

```
PiCamera2-ROS2-Humble-Docker/
├── camera_test/
│   └── main.py          # 메인 카메라 노드
├── Dockerfile           # Docker 이미지 설정
└── README.md           # 이 파일
```