import cv2
import numpy as np
import json
import os 

# Multithreading
from multiprocessing import Process, connection, current_process
import threading

# For esp32 camera stream
import requests
from io import BytesIO
from PIL import Image

#for capturing timed images
from datetime import datetime, timedelta
import time

class CameraWorker:
    def __init__(self,conf):
        self.config = conf
        self.CAPTURE_INTERVAL = 20

    def getVideoWriter(self, cameraSetting)->cv2.VideoWriter:
        name = cameraSetting["Name"]
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        vid_name = "D:\\shitcam\\"+ name + "_{}.avi".format(datetime.now().strftime("%y%m%d%H%M%S"))
        out = cv2.VideoWriter(vid_name,fourcc, 30, (cameraSetting["Resolution"][0],cameraSetting["Resolution"][1]))
        return out


    # save image to file
    def imageCapture(self, frame, timenow, cameraName):
        directory = "D:\\shitcam\\{}\\".format(timenow.strftime("%y%m%d"))
        if not os.path.exists(directory):
            os.makedirs(directory)

        img_name = directory + "{}_{}.jpg".format(cameraName, timenow.strftime("%y%m%d%H%M%S"))
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

        vidCapture = True if cameraSetting["VideoCapture"] =="True" else False
        imgCapture = True if cameraSetting["ImageCapture"] =="True" else False
        
        videoWriter = None
        if vidCapture:
            videoWriter = self.getVideoWriter(cameraSetting=cameraSetting)
        
        cv2.namedWindow(name)
        delta = 1000
        while True:
            now = datetime.now()
            ret, frame = cam.read()
            cv2.imshow(name, frame)

            if imgCapture and delta>self.CAPTURE_INTERVAL:
                 prev=now
                 threading.Thread(target=self.imageCapture, args=(frame, now, name)).start()

            if vidCapture:
                videoWriter.write(frame)

            delta = (now-prev).total_seconds()
            
            # Quit
            keyin = cv2.waitKey(10) & 0XFF

            if keyin == ord('q'):
                if vidCapture:
                    videoWriter.release()
                    capture = False
                    print ("{} recording stopped".format(name))
                break
            
            # Record/stop record
            elif keyin == ord('r'):
                if vidCapture:
                    vidCapture = False
                    videoWriter.release()
                    videoWriter = None
                    print ("{} recording stopped".format(name))

                else:
                    videoWriter = self.getVideoWriter(cameraSetting=cameraSetting)
                    vidCapture = True
                    print ("{} recording start".format(name))
            
            # snapshot
            elif keyin == 32:
                #Process(target=self.imageCapture, args =(frame, now, name)).start()
                threading.Thread(target=self.imageCapture, args=(frame, now, name)).start()
            # toggle auto image capture
            elif keyin == ord('i'):
                if imgCapture:
                    imgCapture = False
                    print('Auto Image Capture: off')
                else:
                    imgCapture = False
                    print('Auto Image Capture: on')


                

    def startStream(self, cameraSetting):
        name = cameraSetting["Name"]
        url = cameraSetting["Location"]
        #res = requests.get(url, stream=True)

        vidCapture = True if cameraSetting["VideoCapture"] =="True" else False
        imgCapture = True if cameraSetting["ImageCapture"] =="True" else False
        
        videoWriter = None
        if vidCapture:
            videoWriter = self.getVideoWriter(cameraSetting=cameraSetting)
        
        imageBytes = bytes()
        delta =1000
        cv2.namedWindow(name)
        new_frame_time = 0
        prev_frame_time = 0
        average_fps = 0
        while True:
            try:
                with requests.get(url, stream = True) as res:
                    for data in res.iter_content(chunk_size=1024):
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
                            
                            # FPS
                            new_frame_time = time.time()
                            fps = 1/(new_frame_time-prev_frame_time)
                            average_fps = fps/20 + (average_fps*19)/20
                            #print (fps)
                            font = cv2.FONT_HERSHEY_SIMPLEX

                            cv2.putText(frame, str(int(average_fps)), (7, 30), font, 1, (100, 255, 0), 1, cv2.LINE_AA)
                            cv2.imshow(name, frame)

                            prev_frame_time = new_frame_time

                            if imgCapture and delta>self.CAPTURE_INTERVAL:
                                prev=now
                                threading.Thread(target=self.imageCapture, args=(frame, now, name)).start()

                            if vidCapture:
                                videoWriter.write(frame)

                            delta = (now-prev).total_seconds()
                            
                            # Quit
                            keyin = cv2.waitKey(10) & 0XFF

                            if keyin == ord('q'):
                                if vidCapture:
                                    videoWriter.release()
                                    capture = False
                                    print ("{} recording stopped".format(name))
                                break
                            
                            # Record/stop record
                            elif keyin == ord('r'):
                                if vidCapture:
                                    vidCapture = False
                                    videoWriter.release()
                                    videoWriter = None
                                    print ("{} recording stopped".format(name))

                                else:
                                    videoWriter = self.getVideoWriter(cameraSetting=cameraSetting)
                                    vidCapture = True
                                    print ("{} recording start".format(name))
                            
                            # snapshot
                            elif keyin == 32:
                                #Process(target=self.imageCapture, args =(frame, now, name)).start()
                                threading.Thread(target=self.imageCapture, args=(frame, now, name)).start()
                            # toggle auto image capture
                            elif keyin == ord('i'):
                                if imgCapture:
                                    imgCapture = False
                                    print('Auto Image Capture: off')
                                else:
                                    imgCapture = False
                                    print('Auto Image Capture: on')

            except Exception as e: 
                print (e)
                print ('{} read failed, retrying..'.format(cameraSetting['Name']))
            time.sleep(10)


    def run(self):
        #pool = [Process(target=run) for _ in range(4)]
        tpool = []
        for camera in self.config["Cameras"]:
            if camera["Enabled"]=="True":
                print (camera)
                tpool.append(threading.Thread(target=self.startCamera, args=(camera,)))
        for stream in self.config["Streams"]:
            if stream["Enabled"]=="True":
                print (stream)
                tpool.append(threading.Thread(target=self.startStream, args=(stream,)))
        for t in tpool:
            t.start()

        for t in tpool:
            t.join()
    

def main():

    ## Load camera config
    f=open("./shitcam/cameraConfig.json") 
    config = json.load(f)
    #print(config)
    f.close()
    worker = CameraWorker(config)
    worker.run()


if __name__ == '__main__':
    main()