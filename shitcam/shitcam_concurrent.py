import cv2
import numpy as np
import json

# Multithreading
from multiprocessing import Process, connection, current_process
import threading

# For esp32 camera stream
import requests
from io import BytesIO
from PIL import Image

#for capturing timed images
from datetime import datetime, timedelta

class CameraWorker:
    def __init__(self,conf):
        self.config = conf
        self.CAPTURE_INTERVAL = 20

    def imageCapture(self, frame, timenow, cameraName):
        img_name = "D:\\shitcam\\"+ cameraName + "_{}.jpg".format(timenow.strftime("%y%m%d%H%M%S"))
        cv2.imwrite(img_name, frame, [cv2.IMWRITE_JPEG_QUALITY, 80] )
        print("{} written!".format(img_name))


    def startCamera(self, cameraSetting):
        name = cameraSetting["Name"]
        try:
            cam = cv2.VideoCapture(cameraSetting["Location"], cv2.CAP_DSHOW)
        
        except:
            print("Unable to connect to camera: ", name)
            return

        cam.set(cv2.CAP_PROP_FRAME_WIDTH, cameraSetting["Resolution"][0])
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, cameraSetting["Resolution"][1])
        
        cv2.namedWindow(name)
        delta = 1000
        while True:
            now = datetime.now()
            ret, frame = cam.read()
            cv2.imshow(name, frame)
            if delta>self.CAPTURE_INTERVAL:
                prev=now
                Process(target=self.imageCapture, args =(frame, now, name)).start()
            
            delta = (now-prev).total_seconds()

            if cv2.waitKey(10) & 0XFF == ord('q'):
                break

    def startStream(self, cameraSetting):
        name = cameraSetting["Name"]
        url = cameraSetting["Location"]
        #res = requests.get(url, stream=True)
        imageBytes = bytes()

        delta =1000

        with requests.get(url, stream = True) as res:
            for data in res.iter_content():#):chunk_size=100):
            # 输出data 查看每一张图片的开始与结尾，查找图片的头与尾截取jpg。并把剩余部分imageBytes做保存
                
                now = datetime.now()

                imageBytes += data
                a = imageBytes.find(b'\xff\xd8')
                b = imageBytes.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = imageBytes[a:b+2]
                    imageBytes = imageBytes[b+2:]

                    bytes_stream = BytesIO(jpg)
                    frame = Image.open(bytes_stream)
                    frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)
                    cv2.imshow(name, frame)

                    if delta>self.CAPTURE_INTERVAL:
                        prev=now
                        Process(target=self.imageCapture, args =(frame, now, name)).start()
                    
                    delta = (now-prev).total_seconds()

                    if cv2.waitKey(10) & 0XFF == ord('q'):
                        break

    def run(self):
        #pool = [Process(target=run) for _ in range(4)]
        tpool = []
        for camera in self.config["Cameras"]:
            print (camera)
            tpool.append(threading.Thread(target=self.startCamera, args=(camera,)))

        for stream in self.config["Streams"]:
            print(stream)
            tpool.append(threading.Thread(target=self.startStream, args=(stream,)))

        for t in tpool:
            t.start()

        for t in tpool:
            t.join()
    

def main():

    ## Load camera config
    f=open("./shitcam-v3/cameraConfig.json") 
    config = json.load(f)
    print(config)
    f.close()
    worker = CameraWorker(config)
    worker.run()


if __name__ == '__main__':
    main()