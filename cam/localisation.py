import cv2
from dt_apriltags import Detector

#show picture from webcam
def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0)
    while True:
        ret_val, img = cam.read()
        if mirror:
            img = cv2.flip(img, 1)
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()

#return webcam picture
def cam_picture(cam_num = 0) -> cv2:
    cam = cv2.VideoCapture(cam_num)
    ret_val, img = cam.read()
    return img

def get_tag_location(img_grey):
    at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    tags = at_detector.detect(img_grey, estimate_tag_pose=False, camera_params=None, tag_size=None)
    return tags

def main():
    print("Hello world")
    pic = cam_picture(0)
    gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
    cv2.imshow('my webcam', gray)
    cv2.waitKey(0)
    
    
    tags = get_tag_location(gray)
    print(tags)

if __name__ == '__main__':
    main()