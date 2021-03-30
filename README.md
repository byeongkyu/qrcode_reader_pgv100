# qrcode_reader_pv100

## Product
-  PGV100-F200A-R4-V19 ([link](https://www.pepperl-fuchs.com/usa/en/classid_3334.htm?view=productdetails&prodid=60355))

## Features
-  QR카메라의 값을 계산하여 qrcode_scan_result ([qrcode_reader_pgv100/QRDetectResult](./msg/QRDetectResult.msg)) 토픽으로 보낸다.
-  qrcode_scan_result 토픽에는 x, y , angle, text, Detection status로 구성

## Install

### Requirements
-  libSerial 1.0.0 : https://github.com/crayzeewulf/libserial

Ubuntu 20.04

```
$ sudo apt install libserial-dev
```

Ubuntu 18.04 - install by building source


## Configuration via using **VisionConfiguration** ([link](https://www.pepperl-fuchs.com/usa/en/classid_3334.htm?view=productdetails&prodid=60355#software)) in Windows
- Head adress: 0x0
-  Baudrate: 115200
-  Terminal Resistance: off
-  Position Resolution: 0.1mm
-  Angle Resolution: 0.1deg

## Usage
```
$ rosrun qrcode_reader_pgv100 qrcode_reader_pgv100_node _port_name:=/dev/ttyS1 _baudrate:=115200 _rate:=50 _angle_offset:=1800
```
- port_name : 제품이 연결 된 PC의 Port 번호
-  baudrate : 통신속도
-  rate : 데이터 송수신을 위한 타이머 주기 입력
- angle_offset : QR카메라 부착 위치를 보정하기 위한 offset