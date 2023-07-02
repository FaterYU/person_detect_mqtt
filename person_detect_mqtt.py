# -- coding: utf-8 --

import sys
import threading
import os
import termios
import cv2
import numpy as np
import time
from paho.mqtt import client as mqtt_client
from ctypes import *
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

sys.path.append('/opt/MVS/Samples/64/Python/MvImport')
from MvCameraControl_class import *

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
 
    client.on_connect = on_connect
    client.connect(broker, port)

def publish(landmark):
    x, y, z = landmark.x, landmark.y, landmark.z
    
    result = client.publish(topic_x, x,qos=2)
    status = result[0]
    result = client.publish(topic_y, y,qos=2)
    status += result[0]
    result = client.publish(topic_z, z,qos=2)
    status += result[0]
    # result: [0, 1]
    if status == 0:
        print(f"Send `{x}`, `{y}`, `{z}` to topic `{topic_x}`, `{topic_y}`, `{topic_z}`")
    else:
        print(f"Failed to send message to topic topic")

def work_thread(cam=0, pData=0, nDataSize=0):
    # set media pipe
    base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True)
    detector = vision.PoseLandmarker.create_from_options(options)
    
    # set detector
    def process_image(image):
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = detector.detect(rgb_frame)
        # print(detection_result.pose_landmarks)
        
        if len(detection_result.pose_landmarks):
            if detection_result.pose_landmarks[0][0].visibility>0.9:
                publish(detection_result.pose_landmarks[0][0])
                # annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
                # cv2.imshow('result',cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
                # cv2.waitKey(0)
        
    
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
    img_buff = None
    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            # print ("get one frame: Width[%d], Height[%d], PixelType[0x%x], nFrameNum[%d]"  % (stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.enPixelType,stFrameInfo.nFrameNum))
            stConvertParam = MV_CC_PIXEL_CONVERT_PARAM()
            memset(byref(stConvertParam), 0, sizeof(stConvertParam))
            if IsImageColor(stFrameInfo.enPixelType) == 'mono':
                # print("mono!")
                stConvertParam.enDstPixelType = PixelType_Gvsp_Mono8
                nConvertSize = stFrameInfo.nWidth * stFrameInfo.nHeight
            elif IsImageColor(stFrameInfo.enPixelType) == 'color':
                # print("color!")
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed  # opecv要用BGR，不能使用RGB
                nConvertSize = stFrameInfo.nWidth * stFrameInfo.nHeight* 3
            else:
                print("not support!!!")
            if img_buff is None:
                img_buff = (c_ubyte * stFrameInfo.nFrameLen)()
            # ---
            stConvertParam.nWidth = stFrameInfo.nWidth
            stConvertParam.nHeight = stFrameInfo.nHeight
            stConvertParam.pSrcData = cast(pData, POINTER(c_ubyte))
            stConvertParam.nSrcDataLen = stFrameInfo.nFrameLen
            stConvertParam.enSrcPixelType = stFrameInfo.enPixelType
            stConvertParam.pDstBuffer = (c_ubyte * nConvertSize)()
            stConvertParam.nDstBufferSize = nConvertSize
            ret = cam.MV_CC_ConvertPixelType(stConvertParam)
            if ret != 0:
                print("convert pixel fail! ret[0x%x]" % ret)
                del stConvertParam.pSrcData
                sys.exit()
            else:
                # print("convert ok!!")
                # 转OpenCV
                # 黑白处理
                if IsImageColor(stFrameInfo.enPixelType) == 'mono':
                    img_buff = (c_ubyte * stConvertParam.nDstLen)()
                    memmove(byref(img_buff), stConvertParam.pDstBuffer, stConvertParam.nDstLen)
                    img_buff = np.frombuffer(img_buff,count=int(stConvertParam.nDstLen), dtype=np.uint8)
                    img_buff = img_buff.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                # 彩色处理
                if IsImageColor(stFrameInfo.enPixelType) == 'color':
                    img_buff = (c_ubyte * stConvertParam.nDstLen)()
                    memmove(byref(img_buff), stConvertParam.pDstBuffer, stConvertParam.nDstLen)
                    img_buff = np.frombuffer(img_buff, count=int(stConvertParam.nDstBufferSize), dtype=np.uint8)
                    img_buff = img_buff.reshape(stFrameInfo.nHeight,stFrameInfo.nWidth,3)
            process_image(img_buff)
        else:
            print ("no data[0x%x]" % ret)
        if g_bExit == True:
                break

def press_any_key_exit():
	fd = sys.stdin.fileno()
	old_ttyinfo = termios.tcgetattr(fd)
	new_ttyinfo = old_ttyinfo[:]
	new_ttyinfo[3] &= ~termios.ICANON
	new_ttyinfo[3] &= ~termios.ECHO
	#sys.stdout.write(msg)
	#sys.stdout.flush()
	termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
	try:
		os.read(fd, 7)
	except:
		pass
	finally:
		termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)

def IsImageColor(enType):
    dates = {
        PixelType_Gvsp_RGB8_Packed: 'color',
        PixelType_Gvsp_BGR8_Packed: 'color',
        PixelType_Gvsp_YUV422_Packed: 'color',
        PixelType_Gvsp_YUV422_YUYV_Packed: 'color',
        PixelType_Gvsp_BayerGR8: 'color',
        PixelType_Gvsp_BayerRG8: 'color',
        PixelType_Gvsp_BayerGB8: 'color',
        PixelType_Gvsp_BayerBG8: 'color',
        PixelType_Gvsp_BayerGB10: 'color',
        PixelType_Gvsp_BayerGB10_Packed: 'color',
        PixelType_Gvsp_BayerBG10: 'color',
        PixelType_Gvsp_BayerBG10_Packed: 'color',
        PixelType_Gvsp_BayerRG10: 'color',
        PixelType_Gvsp_BayerRG10_Packed: 'color',
        PixelType_Gvsp_BayerGR10: 'color',
        PixelType_Gvsp_BayerGR10_Packed: 'color',
        PixelType_Gvsp_BayerGB12: 'color',
        PixelType_Gvsp_BayerGB12_Packed: 'color',
        PixelType_Gvsp_BayerBG12: 'color',
        PixelType_Gvsp_BayerBG12_Packed: 'color',
        PixelType_Gvsp_BayerRG12: 'color',
        PixelType_Gvsp_BayerRG12_Packed: 'color',
        PixelType_Gvsp_BayerGR12: 'color',
        PixelType_Gvsp_BayerGR12_Packed: 'color',
        PixelType_Gvsp_Mono8: 'mono',
        PixelType_Gvsp_Mono10: 'mono',
        PixelType_Gvsp_Mono10_Packed: 'mono',
        PixelType_Gvsp_Mono12: 'mono',
        PixelType_Gvsp_Mono12_Packed: 'mono'}
    return dates.get(enType, '未知')

def hik_cam_init():
	SDKVersion = MvCamera.MV_CC_GetSDKVersion()
	print ("SDKVersion[0x%x]" % SDKVersion)

	deviceList = MV_CC_DEVICE_INFO_LIST()
	tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
	
	# ch:枚举设备 | en:Enum device
	ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
	if ret != 0:
		print ("enum devices fail! ret[0x%x]" % ret)
		sys.exit()

	if deviceList.nDeviceNum == 0:
		print ("find no device!")
		sys.exit()

	print ("Find %d devices!" % deviceList.nDeviceNum)

	for i in range(0, deviceList.nDeviceNum):
		mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
		if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
			print ("\ngige device: [%d]" % i)
			strModeName = ""
			for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
				strModeName = strModeName + chr(per)
			print ("device model name: %s" % strModeName)

			nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
			nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
			nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
			nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
			print ("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
		elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
			print ("\nu3v device: [%d]" % i)
			strModeName = ""
			for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
				if per == 0:
					break
				strModeName = strModeName + chr(per)
			print ("device model name: %s" % strModeName)

			strSerialNumber = ""
			for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
				if per == 0:
					break
				strSerialNumber = strSerialNumber + chr(per)
			print ("user serial number: %s" % strSerialNumber)

	if sys.version >= '3':
		# nConnectionNum = input("please input the number of the device to connect:")
		nConnectionNum = 0
	else:
		nConnectionNum = raw_input("please input the number of the device to connect:")

	if int(nConnectionNum) >= deviceList.nDeviceNum:
		print ("intput error!")
		sys.exit()

	# ch:创建相机实例 | en:Creat Camera Object
	cam = MvCamera()
	
	# ch:选择设备并创建句柄| en:Select device and create handle
	stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

	ret = cam.MV_CC_CreateHandle(stDeviceList)
	if ret != 0:
		print ("create handle fail! ret[0x%x]" % ret)
		sys.exit()

	# ch:打开设备 | en:Open device
	ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
	if ret != 0:
		print ("open device fail! ret[0x%x]" % ret)
		sys.exit()
	
	# ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
	if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
		nPacketSize = cam.MV_CC_GetOptimalPacketSize()
		if int(nPacketSize) > 0:
			ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize",nPacketSize)
			if ret != 0:
				print ("Warning: Set Packet Size fail! ret[0x%x]" % ret)
		else:
			print ("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

	# ch:设置触发模式为off | en:Set trigger mode as off
	ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
	if ret != 0:
		print ("set trigger mode fail! ret[0x%x]" % ret)
		sys.exit()

	# ch:获取数据包大小 | en:Get payload size
	stParam =  MVCC_INTVALUE()
	memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
	
	ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
	if ret != 0:
		print ("get payload size fail! ret[0x%x]" % ret)
		sys.exit()
	nPayloadSize = stParam.nCurValue

	# ch:开始取流 | en:Start grab image
	ret = cam.MV_CC_StartGrabbing()
	if ret != 0:
		print ("start grabbing fail! ret[0x%x]" % ret)
		sys.exit()

	data_buf = (c_ubyte * nPayloadSize)()

	try:
		hThreadHandle = threading.Thread(target=work_thread, args=(cam, byref(data_buf), nPayloadSize))
		hThreadHandle.start()
	except:
		print ("error: unable to start thread")
		
	print ("press a key to stop grabbing.")
	press_any_key_exit()

	g_bExit = True
	hThreadHandle.join()

	# ch:停止取流 | en:Stop grab image
	ret = cam.MV_CC_StopGrabbing()
	if ret != 0:
		print ("stop grabbing fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()

	# ch:关闭设备 | Close device
	ret = cam.MV_CC_CloseDevice()
	if ret != 0:
		print ("close deivce fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()

	# ch:销毁句柄 | Destroy handle
	ret = cam.MV_CC_DestroyHandle()
	if ret != 0:
		print ("destroy handle fail! ret[0x%x]" % ret)
		del data_buf
		sys.exit()
	cv2.destroyAllWindows()
	del data_buf

def mqtt_init():
    connect_mqtt()
    client.loop_start()

if __name__ == "__main__":
    g_bExit = False
    
    broker = '192.168.136.116'
    port = 1883
    topic_x = 'pose_x'
    topic_y = 'pose_y'
    topic_z = 'pose_z'
    
    client = mqtt_client.Client(transport='tcp')
    
    MARGIN = 10  # pixels
    FONT_SIZE = 1
    FONT_THICKNESS = 1
    HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green
    
    mqtt_init()
    hik_cam_init()