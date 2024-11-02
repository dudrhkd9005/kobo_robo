from Brain.Robot_ball_distance import ball_distance
from Brain.Robot import Robot
import cv2
import numpy as np
import time
from PIL import Image
from imutils.video import WebcamVideoStream
from imutils.video import FPS

binary_mask = 0 

class Camera:
    def __init__(self):
        self.primary_object_lower_threshold = np.array([12, 86, 180], dtype=np.uint8)
        self.primary_object_upper_threshold = np.array([111, 160, 255], dtype=np.uint8)

        self.target_zone = [500, 600]

        self.camera_stream = WebcamVideoStream(-1).start()
        self.fps_counter = FPS()
        shape = (self.height, self.width, _) = self.capture_frame().shape
        print(shape)
        time.sleep(2)

    def capture_frame(self):
        try:
            print("image get")
            return self.camera_stream.read()
        except AttributeError:
            print("Attribute Error")
            return np.zeros(shape=(480, 640, 3), dtype="uint8")
    
    def get_hsv_range(color):
        c = np.uint8([[color]])
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        
        lowerLimit = hsvC[0][0][0] - 10, 100, 100
        upperLimit = hsvC[0][0][0] + 10, 255, 255
        
        lowerLimit = np.array(lowerLimit, dtype=np.uint8)
        upperLimit = np.array(upperLimit, dtype=np.uint8)
        
        return lowerLimit, upperLimit

    def find_object(self, input_image):
        hsv_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2YUV)
        binary_mask = cv2.inRange(hsv_image, self.primary_object_lower_threshold, self.primary_object_upper_threshold)
        binary_mask = cv2.erode(binary_mask, None, iterations=1)
        binary_mask = cv2.dilate(binary_mask, None, iterations=1)
        label_count, connected_labels, region_stats, region_centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
        largest_area = -1
        highest_density = -1
        largest_area_index = -1
        
        minimum_density = 0.5

        for i in range(1, label_count):
            area = region_stats[i,cv2.CC_STAT_AREA]
            density = region_stats[i, cv2.CC_STAT_AREA] / (region_stats[i, cv2.CC_STAT_WIDTH] * region_stats[i, cv2.CC_STAT_HEIGHT])

            if density > minimum_density and area > largest_area:
                largest_area = area
                largest_area_index = i
        if largest_area < 100:
                largest_area_index = -1
        if largest_area_index != -1:
            x1, y1, w, h, _ = region_stats[largest_area_index]
            x2, y2 = x1 + w, y1 + h
            return True, (x1, y1), (x2, y2)
        return False, (False, False), (False, False)
        
    def apply_preprocessing(self,input_image):
        lower_yellow = np.array([15, 35, 159])
        upper_yellow = np.array([52, 255, 255])
        hsv_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)


        return yellow_mask
    
    def identify_tip(self, points, convex_hull):
        length = len(points)
        indices = np.setdiff1d(range(length), convex_hull)

        for i in range(2):
            j = indices[i] + 2
            if j > length - 1:
                j = length - j
            if np.all(points[j] == points[indices[i - 1] - 2]):
                return tuple(points[j])  
        
    def find_arrow(self, input_image):
        yellow_mask = self.apply_preprocessing(input_image)
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                if len(approx) == 7:
                    return True, (approx.ravel()[0], approx.ravel()[1])
        return False, (False, False)

    def find_circle(self, input_image):
        processed_image = self.apply_preprocessing(input_image)
        circles = cv2.HoughCircles(processed_image, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=100)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                return True, (x - r, y - r, x + r, y + r)
        return False, (0, 0, 0, 0)
        
    def find_hole(self, input_image):
        processed_image = self.apply_preprocessing(input_image)
        contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            area = cv2.contourArea(contour)
            if len(approx) > 8 and area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                return True, (x, y)
        return False, (False, False)
    
    def find_hole_v2(self, input_image):
        processed_image = self.apply_preprocessing(input_image)
        contours, _ = cv2.findContours(processed_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            circularity = 4 * np.pi * (area / (cv2.arcLength(contour, True) ** 2))
            if 0.7 < circularity < 1.2 and area > 200:
                x, y, w, h = cv2.boundingRect(contour)
                return True, x, y
        return False, None, None

    def find_hole_close(self, input_image):
        processed_image = self.apply_preprocessing(input_image)
        contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        closest_distance = float('inf')
        closest_point = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 150:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    distance = np.sqrt((cx - input_image.shape[1] // 2) ** 2 + (cy - input_image.shape[0] // 2) ** 2)
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_point = (cx, cy)

        if closest_point is not None:
            return True, closest_point[0], closest_point[1]
        return False, None, None

    def find_hole_center(self, input_image):
        processed_image = self.apply_preprocessing(input_image)
        contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 150:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if abs(cx - input_image.shape[1] // 2) < 50 and abs(cy - input_image.shape[0] // 2) < 50:
                        return True, (cx, cy)
        return False, (False, False)
        
    def analyze_short_path(self, input_image):
        result, (x, y) = self.find_hole(input_image)
        if result == False:
            result, [x, y] = self.is_arrow(input_image)
            if result == True:
                return "NoHole", input_image
            else:
                result, (x, y) = self.find_hole_center(input_image)
                if result == False:
                    return "NoHole", input_image

        cv2.line(input_image, (0, 250), (640, 250), (255,0,0), 2)
        cv2.line(input_image, (0, 320), (640, 320), (0,255,0), 2)
        cv2.line(input_image, (0, 400), (640, 400), (255,0,0), 2)
        
        cv2.line(input_image, (255, 0), (255, 480), (255,0,0), 2)
        cv2.line(input_image, (333, 0), (333, 480), (0,255,0), 2)
        cv2.line(input_image, (420, 0), (420, 480), (255,0,0), 2)
        
        cv2.circle(input_image, (x,y), 3, (0,0,255), 3)
        if result == True and 250<y<400 and x<252:
            return "Shot", input_image
        elif result == True and 250<y<400 and x>423:
            return "R-Shot", input_image
        elif result == True  and 242<=y<=408 and 252<=x<=423:
            print("pos : ", (x,y))
            cv2.imwrite("yellow_image.jpg", binary_mask)
            cv2.imwrite("goal_image.jpg", input_image)
            return "Goal", input_image
        elif result == True and y <= 250 and x<333:
            return "L-turn", input_image
        elif result == True and y <= 250 and x>=333:
            return "R-turn-20", input_image
        elif result == True and 400 <= y and x<333:
            return "R-turn-20", input_image
        elif result == True and 400 <= y and x>=333:
            return "L-turn", input_image
        else:
            return "NoHole", input_image
        
    def analyze_short_path_r(self, input_image):
        result, x, y = self.find_hole_v2(input_image)
        if result == False:
            return "NoHole", input_image

        cv2.line(input_image, (280, 0), (280, 480), (255,0,0), 2)
        cv2.line(input_image, (560, 0), (560, 480), (255,0,0), 2)
        
        cv2.circle(input_image, (x,y), 3, (0,0,255), 3)
        if x<280:
            return "LLL-turn", input_image
        elif x<560:
            return "LLL-turn", input_image
        else:
            return "LLLL-turn", input_image
    
    def analyze_long_path_r(self, input_image):
        for y in range(100,480):
            input_image[y, :, :] = 0
            
        result, point = self.find_hole(input_image)
        if result == True:
            x, y = point[0], point[1]
        else:
            result, x, y = self.find_hole_v2(input_image)
            
        if result == False:
            return "R-turn", input_image, None
        
        cv2.line(input_image, (230,0), (120,480), (0,0,255), 1)
        cv2.line(input_image, (150,0), (45,480), (255,0,0), 1)

        cv2.line(input_image, (310,0), (120,480), (200,200,0), 1)
        cv2.line(input_image, (110,0), (120,480), (200,200,0), 1)

        cv2.line(input_image, (360,0), (120,480), (0,200,200), 1)
        cv2.line(input_image, (0,0), (120,480), (0,200,200), 1)

        cv2.line(input_image, (460,0), (120,480), (200,0,200), 1)
        
        cv2.circle(input_image, (x,y), 3, (0,255,0), 3)
        print("x:", x)
        print("y:", y)
        print("x_upper",  ((4800-32*x)/7))
        print("x_under", (11040-48*x)/11)
        if result == True and ( ((4800-32*x)/7) < y < ((11040-48*x)/11) ):
            return "R-Shot", input_image, 20
        elif result == True and y <= (4800-32*x)/7:
            if y < (-5280+48*x):  
                return "R-turn-5", input_image, None
            elif y < (-4*x):
                return "R-turn-10", input_image, None
            else:
                return "R-turn-20", input_image, None
        elif result == True and y >= (11040-48*x)/11:
            if y < (14880-48*x)/19:
                return "L-turn-5", input_image, None
            elif y < (720-2*x):
                return "L-turn-10", input_image, None
            elif y < (11040-24*x)/17:
                return "L-turn-20", input_image, None
            else:
                return "L-turn", input_image, None
        else:
            return "R-turn", input_image, None
        
    def analyze_long_path_par3(self, input_image):
        for y in range(100,480):
            input_image[y, :, :] = 0

        result, point = self.find_hole(input_image)
        if result == True:
            x, y = point[0], point[1]
        else:
            result, x, y = self.find_hole_v2(input_image)
            
        if result == False:
            return "L-turn", input_image, None
        
        cv2.line(input_image, (325,0), (522,480), (0,0,255), 1)
        cv2.line(input_image, (340,0), (580,480), (255,0,0), 1)
        cv2.line(input_image, (310,0), (470,480), (255,0,0), 1)
        
        cv2.line(input_image, (430,0), (522,480), (0,200,200), 1)
        cv2.line(input_image, (253,0), (522,480), (0,200,200), 1)
        
        cv2.line(input_image, (500,0), (522,480), (200,0,200), 1)
        cv2.line(input_image, (150,0), (522,480), (200,0,200), 1)
        
        cv2.circle(input_image, (x,y), 3, (0,255,0), 3)
        print("x:", x)
        print("y:", y)
        dist = ball_distance(70, y)
        print("ball_distance: ", dist)
        print("x_upper",  3*(-310+x))
        print("x_under", 2*(-340+x))
        if result == True and ( 3*(-300+x) > y > 2*(-340+x) ):
            if dist > 130:
                power = 20
            elif dist > 110:
                power = 18
            elif dist > 80:
                power = 16
            elif dist > 75:
                power = 15
            elif dist > 70:
                power = 14
            elif dist > 60:
                power = 13
            elif dist > 50:
                power = 12
            elif dist > 45:
                power = 11
            elif dist > 30:
                power = 10
            else:
                power = 9
            return "Shot", input_image, power
        elif result == True and y <= (-680+2*x):
            if y < (-120000+240*x)/11:  
                return "L-turn-20", input_image, None
            elif y < (-51600+120*x)/23:
                return "L-turn-10", input_image, None
            else:
                return "L-turn-5", input_image, None
        elif result == True and y >= (-930+3*x):
            if y > (-6000+40*x)/31:
                return "R-turn-20", input_image, None
            elif y > (-121440+480*x)/269:
                return "R-turn-10", input_image, None
            else:
                return "R-turn-5", input_image, None
        else:
            return "L-turn", input_image, None
        
    def analyze_long_path(self, input_image):
        result, point = self.find_hole(input_image)
        if result == True:
            x, y = point[0], point[1]
        else:
            result, x, y = self.find_hole_v2(input_image)
            
        if result == False:
            return "L-turn", input_image, None
        
        cv2.line(input_image, (344,0), (520,480), (0,0,255), 1)
        cv2.line(input_image, (350,0), (560,480), (255,0,0), 1)
        cv2.line(input_image, (338,0), (480,480), (255,0,0), 1)
        
        cv2.line(input_image, (374,0), (520,480), (200,200,0), 1)
        
        cv2.line(input_image, (430,0), (520,480), (0,200,200), 1)
        cv2.line(input_image, (250,0), (520,480), (0,200,200), 1)
        
        cv2.line(input_image, (510,0), (520,480), (200,0,200), 1)
        cv2.line(input_image, (140,0), (520,480), (200,0,200), 1)
        
        cv2.circle(input_image, (x,y), 3, (0,255,0), 3)
        print("x:", x)
        print("y:", y)
        dist = ball_distance(70, y)
        print("ball_distance: ", dist)
        print("x_upper",  (-81120+240*x)/71)
        print("x_under", (-5600+16*x)/7)
        if result == True and ( (-81120+240*x)/71 > y > (-5600+16*x)/7):
            dist = ball_distance(70, y)
            if dist > 130:
                power = 20
            elif dist > 110:
                power = 20
            elif dist > 80:
                power = 20
            elif dist > 75:
                power = 20
            elif dist > 70:
                power = 19
            elif dist > 60:
                power = 18
            elif dist > 50:
                power = 17
            elif dist > 45:
                power = 16
            elif dist > 30:
                power = 15
            else:
                power = 15
            return "Shot", input_image, power
        elif result == True and y <= (-5600+16*x)/7:
            if y < (-24480+48*x):  
                return "L-turn", input_image, None
            elif y < (-6880+16*x)/3:
                return "L-turn-20", input_image, None
            elif y < (-89760+240*x)/73:
                return "L-turn-10", input_image, None
            else:
                return "L-turn-5", input_image, None
        elif result == True and y >= (-81120+240*x)/71:
            if y > (-3360+24*x)/19:
                return "R-turn-20", input_image, None
            elif y > (-4000+16*x)/9:
                return "R-turn-10", input_image, None
            else:
                return "R-turn-5", input_image, None
        else:
            return "L-turn", input_image, None
    
    def analyze_long_path_gamble(self, input_image):
        result, point = self.find_hole(input_image)
        if result == True:
            x, y = point[0], point[1]
        else:
            result, x, y = self.find_hole_close(input_image)
            
        if result == False:
            return "L-turn", input_image, None
        
        cv2.line(input_image, (340,0), (522,480), (255,0,0), 1)
        cv2.line(input_image, (245,0), (465,480), (255,0,0), 1)

        cv2.line(input_image, (200,0), (522,480), (200,200,0), 1)

        cv2.line(input_image, (430,0), (522,480), (0,200,200), 1)
        cv2.line(input_image, (90,0), (522,480), (0,200,200), 1)

        cv2.line(input_image, (570,0), (522,480), (200,0,200), 1)
        
        cv2.circle(input_image, (x,y), 3, (0,255,0), 3)
        print("x:", x)
        print("y:", y)
        dist = ball_distance(70, y)
        print("ball_distance: ", dist)
        print("x_upper",  (-5880+24*x)/11)
        print("x_under", (-81600+240*x)/91)
        
        if result == True and ( (-5880+24*x)/11 > y > (-81600+240*x)/91):
            dist = ball_distance(70, y)
            if dist > 130:
                power = 22
            elif dist > 110:
                power = 20
            elif dist > 80:
                power = 20
            elif dist > 75:
                power = 19
            elif dist > 70:
                power = 18
            elif dist > 60:
                power = 17
            elif dist > 50:
                power = 16
            elif dist > 45:
                power = 15
            elif dist > 30:
                power = 13
            else:
                power = 13
            return "Shot", input_image, power
        elif result == True and y <= (-81600+240*x)/91:
            if y > (5700-10*x):  
                return "L-turn", input_image, None
            else:
                if y > (-51600+120*x)/23:
                    return "L-turn-5", input_image, None
                else:
                    return "L-turn-10", input_image, None
        elif result == True and y >= (-5880+24*x)/11:
            if y < (-48000+240*x)/161:
                return "R-turn-5", input_image, None
            elif y < (-900+10*x)/9:
                return "R-turn-10", input_image, None
            else:
                return "R-turn-20", input_image, None
        else:
            return "L-turn", input_image, None
    
if __name__ == "__main__":
    camera = Camera()
    while True:
        frame = camera.capture_frame()
        result, ball_box = camera.find_circle(frame.copy())
        if result:
            cv2.rectangle(frame, (ball_box[0], ball_box[1]), (ball_box[2], ball_box[3]), (0,0,255), 2)
        cv2.imshow("detect", frame)
        if cv2.waitKey(16) == ord("q"):
            break
        
        
    cv2.destroyAllWindows()
