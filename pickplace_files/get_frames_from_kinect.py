import freenect
import cv2
import numpy as np

#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array

#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

if __name__ == "__main__":
    i = 0
    while 1:
        #get a frame from RGB camera
        frame = get_video()
        #get a frame from depth sensor
        depth = get_depth()
        #display RGB image
        cv2.imshow('RGB image',frame)
        #display depth image
        cv2.imshow('Depth image',depth)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:         # wait for ESC key to exit
            cv2.destroyAllWindows()
            break
        elif k == ord('s'): # wait for 's' key to save and exit
            cv2.imwrite('frame'+str(i)+'.png',frame)
            cv2.imwrite('depth'+str(i)+'.png',depth)
            i = i+1
    cv2.destroyAllWindows()
