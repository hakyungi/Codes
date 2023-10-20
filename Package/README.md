# Package

OpenMV H7 / H7 Plus 카메라에 I/F 보드를 장착하여 TOF(적외선 거리 측정센서) 센서 3개와 9 DOF IMU(BNO055) 센서를 연결 한 샘플 코드입니다.

I2C 주소만 틀리다면 선을 병렬로 연결하여 센서 추가가 가능합니다.(예 MLX90614 비접촉식 온도센서)

I/F 보드와 EV3 또는 SPike Prime과 연결한 예입니다.

USB 연결만으로는 동작이 안되며 EV3 또는 Spike Prime에 연결하여 전원이 공급되어야 작동 가능합니다.
TIMEOUT, ENODEV 간은 에러가 뜨면
    - 전원이 연결되지 않았거나
    - 전선이 잘못 연결되었거나
    - 커넥터 접촉이 안좋거나
상기 이유일겁니다.

테스트 후 이상이 없을 경우 커넥터 부위를 글루건으로 고정하시길 추천드립니다.

동영상 참조.
EV3 : https://youtu.be/USE37TX5gu8?si=YUAwIl3PE65uqGaZ
Spike Prime : https://youtube.com/shorts/MC_HzxJB1lU?si=tBtv8v40VfLhSl6N

전류는 아래와 같이 소모되며 OpenMV H7 + TOF 3개 + IMU 시 약 240mA 소모됨.
    - OpenMV H7 : 약 175mA @ 5V
    - TOF : 약 20mA @ 5V
    - IMU : 약 10mA @ 5V

EV3는 모든 포트에 공급되는 전류의 총합이 350mA이며 최대 1A에서 전원을 차단합니다. (모터 구동 전원은 별도)

PDF 문서를 참조하세요.