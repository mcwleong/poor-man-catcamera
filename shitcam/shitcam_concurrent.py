import cv2
import numpy as np
import json
import os 

# Multithreading
import threading

# For esp32 camera stream
import requests
from io import BytesIO
from PIL import Image

#for capturing timed images
from datetime import datetime
import time



class CameraWorker:
    # Reworked the CameraWorker Class such that each camera thread stores its own local variable
    # and spawns its own persistant thrad for image capture
    # 
    # This will be easier for the image recognition and motion detection task later on

    def __init__(self, cameraSetting, isStream):
        self.name = cameraSetting["Name"]
        self.location = cameraSetting["Location"]
        self.isStream= isStream
        self.vidCapture = None
        self.imgCapture = True if cameraSetting["ImageCapture"] =="True" else False
        self.capture_interval = 20
        self.resolution = (cameraSetting["Resolution"][0], cameraSetting["Resolution"][1])
        self.cameraRunningThread = threading.Thread(target=self.setupCamera)
        self.grabbed =None
        self.frame = None
        self.prev_frame_blurred = None
        self.stop = False
        self.target_fps = 5


    def getVideoWriter(self)->cv2.VideoWriter:
        name = self.name
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        vid_name = "D:\\shitcam\\"+ name + "_{}.avi".format(datetime.now().strftime("%y%m%d%H%M%S"))
        out = cv2.VideoWriter(vid_name,fourcc, 20, (self.resolution[0], self.resolution[1]))
        print ("{} recording start".format(self.name))

        return out

    def putDateTimeText(self, buf):        
        cv2.putText(
            buf, 
            datetime.now().strftime("%d/%m/%y %H:%M:%S"), 
            (7, 30), 
            cv2.FONT_HERSHEY_DUPLEX, 
            1, 
            (255, 255, 255), 
            2,
            cv2.LINE_AA
        )
        return buf

    def imageCapture(self,framebuf):
        timenow = datetime.now()
        directory = "D:\\shitcam\\{}\\".format(timenow.strftime("%y%m%d"))
        if not os.path.exists(directory):
            os.makedirs(directory)
        img_name = directory + "{}_{}.jpg".format(self.name, timenow.strftime("%y%m%d%H%M%S"))
        buf =self.putDateTimeText(framebuf)
        cv2.imwrite(img_name, buf, [cv2.IMWRITE_JPEG_QUALITY, 80] )
        #print("{} written!".format(img_name))

    def stopRecording(self):
        if self.vidCapture!=None:
            self.vidCapture.release()
            self.videoCapture=None
            print ("{} recording stopped".format(self.name))

    def readFrame(self):
        self.grabbed=False
        return self.frame.copy()

    def detectMotion(self):
        frame_blurred = cv2.cvtColor(self.frame.copy(), cv2.COLOR_RGB2GRAY)
        frame_blurred = cv2.GaussianBlur(src=frame_blurred, ksize=(5,5), sigmaX=0)

        if self.prev_frame_blurred is None:
            self.prev_frame_blurred=frame_blurred
            return []
        
        diff_frame = cv2.absdiff(src1=self.prev_frame_blurred, src2=frame_blurred)
        self.prev_frame_blurred = frame_blurred
        diff_frame = cv2.dilate(diff_frame, np.ones((5 , 5)), 1)
        thresh_frame = cv2.threshold(src=diff_frame, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(image=thresh_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        contour_list =[]
        for contour in contours:
            if cv2.contourArea(contour) < 50:
                continue
            contour_list.append(cv2.boundingRect(contour))
        return contour_list


    def startCamera(self):
        # Wait until the camera/stream is set up
        self.cameraRunningThread.start()
        while self.grabbed != True:
            time.sleep(0.1)

        # start showing the image
        cv2.namedWindow(self.name)
        delta = 1000
        new_frame_time = 0
        prev_frame_time = 0
        average_fps = 0
        static_frame_count =1000
        isrecording = False
        move_count = 0
        while True:
            if self.grabbed == True:
                now = datetime.now()
                stream_buf = self.readFrame()

                contour_list=self.detectMotion() 

                if len(contour_list)>0:
                    if isrecording==False:
                        move_count= move_count+1
                        
                        if move_count >3:
                            self.vidCapture=self.getVideoWriter()
                            isrecording=True
                            move_count =0

                    for contour in contour_list:
                        cv2.rectangle(img=stream_buf, 
                                    pt1 = (contour[0], contour[1]),
                                    pt2 = (contour[0]+contour[2], contour[1]+contour[3]),
                                    color=(0, 255, 0), thickness=2)
                    static_frame_count=0
                else:
                    static_frame_count = static_frame_count+1
                    if static_frame_count>45 and isrecording == True:
                        self.stopRecording()
                        isrecording=False

                
                if self.imgCapture and delta>self.capture_interval:
                    prev=now
                    threading.Thread(target=self.imageCapture, args=(stream_buf.copy(),)).start()
                if self.vidCapture!=None:
                    self.vidCapture.write(self.putDateTimeText(self.frame.copy()))
                

                new_frame_time = time.time()
                fps = 1/(new_frame_time-prev_frame_time)
                
                average_fps = fps/100 + (average_fps*99)/100
                prev_frame_time = new_frame_time

                font = cv2.FONT_HERSHEY_SIMPLEX
                #cv2.putText(stream_buf, str(int(average_fps)), (7, 30), font, 1, (100, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(stream_buf, str(int(static_frame_count)) + " " + str(int(average_fps)), (7, 30), font, 1, (100, 255, 0), 1, cv2.LINE_AA)

                cv2.imshow(self.name, stream_buf)
                keyin = cv2.waitKey(1) &0xFF

                #sleeptime = sleeptime += (average_fps-self.target_fps)/10000
                #time.sleep(sleeptime if sleeptime > 0)
                #time.sleep(0.03)
                if self.imgCapture:
                        delta = (now-prev).total_seconds()

            if keyin == ord('q'):
                self.stopRecording()
                self.stop=True
                cv2.destroyWindow(self.name)
                return
            elif keyin == ord('r'):     # Record/stop record
                if self.vidCapture!=None:
                    self.stopRecording()
                else:
                    self.vidCapture = self.getVideoWriter()
            elif keyin == 32:                   #Space = Snapshot
                threading.Thread(target=self.imageCapture).start()
            
            # toggle auto image capture
            elif keyin == ord('i'):
                self.imgCapture = not self.imgCapture
                print("Auto Image Capture: ", self.imgCapture)


    def setupCamera(self):      
        if self.isStream ==False:
            self.captureCameraImage(cv2.VideoCapture(self.location)) # Camera
        else:
            self.captureStreamImage(requests.get(self.location,stream=True)) # Stream

    def captureCameraImage(self, cam): #Not tested!
        while True:
            self.grabbed, self.frame = cam.read()
            if self.grabbed == False or self.stop ==True:
                break
    
    def captureStreamImage(self, cam):
        while True:
            try:
                imageBytes = bytes()
                with cam as res:
                    for data in res.iter_content(chunk_size=64):
                        imageBytes += data
                        a = imageBytes.find(b'\xff\xd8')
                        b = imageBytes.find(b'\xff\xd9')
                        if a != -1 and b != -1:
                            jpg = imageBytes[a:b+2]
                            imageBytes = imageBytes[b+2:]

                            bytes_stream = BytesIO(jpg)
                            frame = Image.open(bytes_stream)
                            self.frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)
                            self.grabbed = True
                            #time.sleep(1)

                        if self.stop == True:
                            return
            
            except Exception as e: 
                print (e)
                print ('{} read failed, retrying..'.format(self.name))
            time.sleep(10)        

class CameraManager:
    def __init__(self, conf):
        self.config = conf

    def run(self):
        #pool = [Process(target=run) for _ in range(4)]
        workers = []
        for camera in self.config["Cameras"]:
            if camera["Enabled"]=="True":
                print (camera)
                workers.append(CameraWorker(camera, False))
        for stream in self.config["Streams"]:
            if stream["Enabled"]=="True":
                print (stream)
                workers.append(CameraWorker(stream, True))
        
        tpool = [threading.Thread(target=worker.startCamera) for worker in workers]
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
    worker = CameraManager(config)
    worker.run()


if __name__ == '__main__':
    main()