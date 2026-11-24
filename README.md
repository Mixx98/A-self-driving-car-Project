# 🚗 A Self-Driving Car Project

ESP32로 만든 **라이다(LiDAR) 기반 장애물 회피 + GPS 자율주행 RC카** 프로젝트입니다.
스마트폰 앱(App Inventor)과 블루투스로 연결해 수동 조종하거나, 목적지 방향으로 스스로 주행하게 할 수 있습니다.

## 주요 기능

- **수동 조종 모드** – 앱에서 전진 / 후진 / 좌회전 / 우회전 / 정지 명령을 블루투스로 전송
- **자율 주행 모드** – 전방 라이다 값을 보고 장애물이 있으면 정지하거나 반대 방향으로 회피
- **GPS 위치 전송** – GPS 모듈로 현재 위치를 받아 앱으로 전송
- **방위 측정** – 지자기 센서(HMC5883L, I2C `0x1E`)로 차량이 바라보는 방향을 계산해 앱으로 전송
- **배터리 감시** – 분압 저항으로 배터리 전압을 읽고 저전압이면 부저 경고

## 시스템 구성

```
[YDLidar] ──UART── [ESP32 #1 (lidar.ino)] ──UART── [ESP32 #2 (code.ino)] ──Bluetooth── [스마트폰 앱 (.aia)]
                    360° 거리 → 10° 단위 36칸으로 요약        │
                                                     GPS · 지자기 센서 · 모터 드라이버(74HC595 시프트 레지스터)
```

| 구성 요소 | 역할 |
| --- | --- |
| ESP32 (라이다용) | YDLidar의 360° 스캔을 10° 단위 36개 구간 최솟값으로 압축해 200ms마다 송신 |
| ESP32 (메인) | 모터 4개 PWM 제어, GPS·지자기 센서 읽기, 블루투스 통신 |
| App Inventor 앱 | 조종 버튼, 위치·방향·장애물 거리 표시 |

## 파일 구조

```
├── code.ino                  # 메인 ESP32 – 주행 제어, GPS, 지자기, 블루투스
├── lidar/
│   ├── lidar.ino             # 라이다 전용 ESP32 – 거리 데이터 수집/전송
│   ├── YDLidar.cpp / .h      # YDLidar 드라이버
│   └── inc/v8stdint.h
├── A self-driving car Project.aia   # MIT App Inventor 프로젝트 (조종 앱)
└── PPT2.pptx                 # 발표 자료
```

## 사용 기술

`ESP32` `Arduino` `YDLidar` `TinyGPS++` `HMC5883L` `Bluetooth Serial` `MIT App Inventor`

## 실행 방법

1. Arduino IDE에 ESP32 보드 패키지와 `TinyGPS++` 라이브러리를 설치합니다.
2. `lidar/lidar.ino`를 라이다 쪽 ESP32에 업로드합니다. (파티션: 2MB APP / 2MB SPIFFS)
3. `code.ino`를 메인 ESP32에 업로드합니다.
4. `.aia` 파일을 [MIT App Inventor](https://appinventor.mit.edu/)에서 열어 스마트폰에 설치하고, 블루투스 기기 `Project`에 연결합니다.

## 통신 프로토콜 (블루투스)

| 시작 바이트 | 명령 | 의미 |
| --- | --- | --- |
| `20` | `1` ~ `5` | 수동 모드: 전진 / 우 / 좌 / 후진 / 정지 |
| `20` | `11` ~ `14` | 자율 모드로 전환하며 이동 |

---
📅 2022년 진행
