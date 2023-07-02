# person_detect_mqtt
Detect person with Mediapipe and publish with MQTT

## Description

This project is to detect person with Mediapipe and publish the coordinates of the person's nose to MQTT. The test device is Hikvision MV-CA016-10UC.

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

## Result

The result will be like this

```bash
SDKVersion[0x3020201]
Connected to MQTT Broker!
Find 1 devices!

u3v device: [0]
device model name: MV-CA016-10UC
user serial number: 00K01829492
press a key to stop grabbing.
INFO: Created TensorFlow Lite XNNPACK delegate for CPU.
Send `0.058794498443603516`, `0.5515105128288269`, `-0.041321564465761185` to topic `pose_x`, `pose_y`, `pose_z`
Send `0.7573527097702026`, `0.48874354362487793`, `-1.3059470653533936` to topic `pose_x`, `pose_y`, `pose_z`
Send `0.7348895072937012`, `0.5120759010314941`, `-0.9977606534957886` to topic `pose_x`, `pose_y`, `pose_z`
Send `0.7130177021026611`, `0.436440110206604`, `-1.1693328619003296` to topic `pose_x`, `pose_y`, `pose_z`
```

It will send the `x`, `y`, `z` coordinates of the person's nose to the topic `pose_x`, `pose_y`, `pose_z` respectively.
- x: The horizontal coordinate of the keypoint in the image coordinate system, ranging from [0, 1], where 0 means the leftmost edge of the image and 1 means the rightmost edge. 
- y: The vertical coordinate of the keypoint in the image coordinate system, ranging from [0, 1], where 0 means the topmost edge of the image and 1 means the bottommost edge. 
- z: The depth coordinate of the keypoint in the world coordinate system, measured in meters, where smaller values mean closer to the camera.
