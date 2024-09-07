# GuardBot
고1 정보 R&amp;E 산출물 관련 소스코드

# Requirements
- libcurl4-openssl-dev

# Build
```bash
git clone https://github.com/EEEYeSeong/GuardBot.git GuardBot
cd GuardBot
chmod +x build.sh
./build.sh
```

# Run
디바이스와 연결
```bash
./bin/slam_http ORB_SLAM3/Vocabulary/ORBvoc.txt ./config/DevCamera1.yaml 192.168.1.3:5000
```

# LICENSE
GPL v3 라이선스를 따릅니다.