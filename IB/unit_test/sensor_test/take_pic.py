import picamera
import datetime



class Camera:
    
    def __init__(self):
        self.camera = picamera.PiCamera()
        print('キャメラ初期化完了')
    
    def take_pic(self, file_path):
        self.camera.capture(file_path)
        
    
if __name__ == "__main__":
    camera = Camera()
    
    file_path = '/~/ARLISS_IBIS/image_test.jpg'

    print("taking pic...: {}".format(file_path))
    camera.take_pic(file_path) # 写真を撮る