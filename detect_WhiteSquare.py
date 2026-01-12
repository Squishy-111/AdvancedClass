import cv2
import os
import numpy as np
import math

class Tracked_contours:

    def __init__(self, id, x, w, y, h):
        self.id = id
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.streak = 0
        self.status = 'available'
        self.missing_frames = 0
        
    def get_streak(self):
        return self.streak
    
    def get_id(self):
        return self.id
    
    def reset_status(self):
        self.status = "available"
        self.missing_frames += 1
    
    def get_status(self):
        return self.status
    
    def get_rect(self):
        return self.x, self.w, self.y, self.h
    
    def update(self, new_x, new_w, new_y, new_h):
        self.x = new_x
        self.w = new_w
        self.y = new_y
        self.h = new_h
        self.streak += 1
        self.missing_frames = 0
        self.status = "taken"

    def get_distance_to(self, x_2, w_2, y_2, h_2):
        x_1, w_1, y_1, h_1 = self.get_rect()

        c_x1 = x_1 + w_1/2
        c_x2 = x_2 + w_2/2
        c_y1 = y_1 + h_1/2
        c_y2 = y_2 + h_2/2

        dist = math.hypot(c_x1 - c_x2, c_y1 - c_y2)
        return dist
    

def found_match(objects, new_x, new_w, new_y, new_h):

    max_pixel_distance = 100
    best_distance = 1000
    best_obj = None

    for obj in objects:
        distance = obj.get_distance_to(new_x, new_w, new_y, new_h)
        if distance < best_distance:
            best_distance = distance
            best_obj = obj
    
    if best_distance < max_pixel_distance:
        if best_obj.get_status() == 'available':
            best_obj.update(new_x,new_w,new_y,new_h)
            return True
    return False 

def Whiteness_check(checked_contour, mask):
   
    ideal_whiteness_percentage = 0.95 
    
   
    x, y, w, h = cv2.boundingRect(checked_contour)
    if w == 0 or h == 0: return False

    roi_mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = checked_contour - [x, y]
    cv2.drawContours(roi_mask, [shifted_contour], -1, 255, -1) 

    
    mask_roi = mask[y:y+h, x:x+w]
    
    intersection_region = cv2.bitwise_and(mask_roi, roi_mask)

    white_pixels = cv2.countNonZero(intersection_region)
    total_pixels = cv2.countNonZero(roi_mask)

    if total_pixels == 0:
        return False
    
    if white_pixels/total_pixels > ideal_whiteness_percentage:
        return True
    
    return False
#------------------------------------------------------------
def square_test(checked_contour):
    lower_ratio = 0.5
    upper_ratio = 1.5

    rect = cv2.minAreaRect(checked_contour)
    (center_x, center_y), (width, height), angle = rect

    
    if width <= 0 or height <= 0:
        return False, None
    
    square_ratio = width / height

    if upper_ratio > square_ratio > lower_ratio:
        return True, square_ratio
    else:
        return False, None
#------------------------------------------------------------------------------
def vertex_amount_test(checked_contour):
    precision = 0.02
    epsilon = precision*cv2.arcLength(checked_contour, True)
    approx = cv2.approxPolyDP(checked_contour, epsilon, True)

    if len(approx) == 4 and cv2.isContourConvex(approx) :
            return True
        
    return False
#-------------------------------------------------------------------------------

path = "white_square_videos"
video_name = "IMG_4227.MOV"
full_path = os.path.join(path, video_name)

video = cv2.VideoCapture(full_path)

if not video.isOpened():
    print("Error: Could not open video.")
    exit()


cv2.namedWindow('webcam', cv2.WINDOW_NORMAL)
cv2.namedWindow('mask', cv2.WINDOW_NORMAL)

cv2.resizeWindow('webcam', 600, 800)
cv2.resizeWindow('mask', 600, 800)
# ---------------------------

#grabbed these three lines from a stack overflow post(not sure if ranges need to be tweaked)
sensitivity = 20
lower_hsv = np.array([0,0,255 - sensitivity])
upper_hsv = np.array([255,255 - sensitivity,255])

min_pixel_area = 300 # could cause problems if really far away maybe?
id_counter = 0
existing_valid_contours = []


while True:
    ret, img = video.read()

    for contour in existing_valid_contours:
        contour.reset_status()

    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower_hsv, upper_hsv)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        for contour in contours:
            if cv2.contourArea(contour) > min_pixel_area:
                x, y, w, h = cv2.boundingRect(contour) 
                
                if vertex_amount_test(contour):

                    if Whiteness_check(contour, mask):
                        if found_match(existing_valid_contours, x, w, y, h) != True:
                            valid_contour = Tracked_contours(id_counter, x, w, y, h)
                            existing_valid_contours.append(valid_contour)
                            id_counter += 1

    
    existing_valid_contours = [obj for obj in existing_valid_contours if obj.missing_frames < 10]

    for contour in existing_valid_contours:
        contour.reset_status() #important line
        streak = contour.get_streak()
        id = contour.get_id()
        x, w, y, h = contour.get_rect()
        if streak > 20:
            cv2.rectangle(img, (x, y), (x + w,y + h), (0,0,255), 3)
            cv2.putText(img, f"streak:{streak}_ID:{id}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    
    cv2.imshow('mask', mask)
    cv2.imshow('webcam', img)
    

    if cv2.waitKey(30) & 0xFF == 27:
        break

video.release()
cv2.destroyAllWindows()