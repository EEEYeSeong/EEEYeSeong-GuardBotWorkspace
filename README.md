# GuardBot
고1 정보 R&amp;E 산출물 관련 소스코드
![Image1](/assets/image1.png)

<br>

# Requirements
- libcurl4-openssl-dev
- paho.mqtt.cpp

<br>

# Build
```bash
git clone https://github.com/EEEYeSeong/GuardBot.git GuardBot
cd GuardBot
chmod +x patch.sh
chmod +x build.sh
./patch.sh
./build.sh
```

<br>

# Run
## HTTP로 연결
### Server(PC)
```bash
./bin/slam_http ./ORB_SLAM3/Vocabulary/ORBvoc.txt ./config/DevCamera1.yaml (rasp_ip):5000
```
### Client(RaspberryPI)
```bash
python3 ./raspberry_pi/slam_http.py
```
## MQTT로 연결
### Server(PC)
```bash
./bin/slam_mqtt ./ORB_SLAM3/Vocabulary/ORBvoc.txt ./config/DevCamera1.yaml (rasp_ip) 1883
```
### Client(RaspberryPI)
```bash
./bin/slam_mqtt_rasp (pc_ip) 1883 /dev/video0
```

<br>

# LICENSE
GPL v3 라이선스를 따릅니다.