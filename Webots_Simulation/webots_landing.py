import cv2
import os
import numpy as np
import math

#pretty much same code as my detection model 
# but just made ito function that I can grab inflight and detect landing zone all in webots

#initalize each contour that passes two tests into an object
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
    
#tests how close new contours are to the previous frame's contours to determine if they are the same white square
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
#-------------------------------------------------------------------------------------------------------------------
#checks the amount of whiteness in the detected square(shadows may be bad so should include patching line somewhere)
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
#------------------------------------------------------------------------------
#test to see how many vertexs needed to plot outer contour of detected object
def vertex_amount_test(checked_contour):
    precision = 0.02
    epsilon = precision*cv2.arcLength(checked_contour, True)
    approx = cv2.approxPolyDP(checked_contour, epsilon, True)

    if len(approx) == 4 and cv2.isContourConvex(approx) :
            return True
        
    return False
#-------------------------------------------------------------------------------

class LandingPadTracker:

    def __init__(self):
        # Configuration
        self.sensitivity = 20
        self.lower_hsv = np.array([0, 0, 255 - self.sensitivity])
        self.upper_hsv = np.array([255, 50, 255])
        
        self.min_pixel_area = 300 
        self.streak_min = 20
        
        # State Variables (Persist between frames)
        self.id_counter = 0
        self.existing_valid_contours = []

#-------------------------------------------------------------------------------
#call this function once in main flight loop
    def process(self, webots_img):

        img = webots_img.copy()

        # 1. Reset status of existing tracks
        for contour in self.existing_valid_contours:
            contour.reset_status()

        # 2. Image Processing
        image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image_hsv, self.lower_hsv, self.upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 3. Detection Logic
        if len(contours) != 0:
            for contour in contours:
                if cv2.contourArea(contour) > self.min_pixel_area:
                    x, y, w, h = cv2.boundingRect(contour) 
                    
                    if vertex_amount_test(contour):
                        if Whiteness_check(contour, mask):
                            if not found_match(self.existing_valid_contours, x, w, y, h):
                                valid_contour = Tracked_contours(self.id_counter, x, w, y, h)
                                self.existing_valid_contours.append(valid_contour)
                                self.id_counter += 1

        # 4. Cleanup faulty detections
        self.existing_valid_contours = [obj for obj in self.existing_valid_contours if obj.missing_frames < 10]

        target_coords = None
        target_found = False

        for contour in self.existing_valid_contours:
        
            streak = contour.get_streak()
            id = contour.get_id()
            x, w, y, h = contour.get_rect()
            
            if streak > self.streak_min:
                target_found = True
                target_coords = (x + w/2, y + h/2)
                
                cv2.rectangle(img, (x, y), (x + w, y + h), (0,0,255), 3)
                cv2.putText(img, f"streak:{streak}_ID:{id}", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # 5. Update Display
        cv2.imshow('mask', mask)
        cv2.imshow('webcam', img)
        cv2.waitKey(1) # CRITICAL: Keeps window alive

        return target_found, target_coords
        

        