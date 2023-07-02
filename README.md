# person_detect_mqtt
Detect person with Mediapipe and publish with MQTT

## Requirements

- Python 3.7
- OpenCV 4.5.1
- Mediapipe 0.8.3
- Paho-mqtt 1.5.1
- mosquitto 1.6.12

## Install script

1. Update and upgrade

```bash
sudo apt update && sudo apt upgrade -y
```

2. Install MediaPipe dependencies

```bash
pip install mediapipe
```

3. Install Paho-mqtt

```bash
pip install paho-mqtt
```

4. Install mosquitto

```bash
sudo apt install mosquitto mosquitto-clients
mosquitto -v -d -c /etc/mosquitto/mosquitto.conf
```

5. configure mosquitto

```bash
sudo cp /usr/share/doc/mosquitto/examples/mosquitto.conf /etc/mosquitto/conf.d/mosquitto.conf
sudo nano /etc/mosquitto/conf.d/mosquitto.conf
```

Write this lines in the file
Replace your `ip address` to line `bind_address xxx.xxx.xxx.xxx`

```bash
allow_anonymous true
listener 1883
bind_address 192.168.136.116
```

6. Restart mosquitto

```bash
sudo systemctl restart mosquitto
```

7. Install Hikvision SDK

Download the SDK from [Hikvision](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_221208.zip) and choose your architecture file to install.

Show your architecture

```bash
uname -m
```

Download and install the SDK, in example is `x86_64`

```bash
sudo apt install wget
wget https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_221208.zip
unzip MVS_STD_GML_V2.1.2_221208.zip
cd MVS_STD_GML_V2.1.2_221208
sudo dpkg -i MVS-2.1.2_x86_64_20221208.deb
```

8. Clone this repository and enter the folder

```bash
git clone git@github.com:FaterYU/person_detect_mqtt.git
cd person_detect_mqtt
```

9. Download task file

```bash
wget https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task
```

## Run

```bash
python3 person_detect_mqtt.py
```