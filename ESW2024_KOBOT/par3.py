import time
import sys
import cv2

from Actuator.Motion import Motion
from Sensor.Camera import Camera
from Brain.Robot import Robot
from Brain.Robot_calc_ball_dist import calc_ball_dist

shot_count = 0
plain_frame_count = 0
clockwise = "Left"
shot_direction = "Left"
shot_power = 8
shot_roi = False

FIND_BALL = 1
MOVE_TO_BALL = 2
SHORT_SHOT = 3
LONG_SHOT = 4
MOVE_TO_GOAL = 5
SHOT = 6
CEREMONY = 7

ball_lost_start_time = None
ball_lost_threshold = 2

if __name__ == "__main__":

    Robot = Robot()
    Motion = Motion()
    Camera = Camera()

    Motion.set_initial_positionialize()
    Motion.set_initial_position(True)
    Motion.adjust_neck(50)
    neck_before_find = 50

    time.sleep(1)
    while True:
        frame = Camera.capture_frame()

        img = frame.copy()
        Robot.bool_ball, ballBox1, ballBox2 = Camera.find_object(img)

        if Robot.bool_ball:
            plain_frame_count = 0
            ball_lost_start_time = None
            cv2.rectangle(frame, ballBox1, ballBox2, (0, 0, 255), 2)
        else:
            if ball_lost_start_time is None:
                ball_lost_start_time = time.time()

        if Robot.mission == FIND_BALL:
            if Robot.bool_ball:
                Robot.mission = MOVE_TO_BALL
            else:
                Robot.mission = FIND_BALL
        elif Robot.mission == MOVE_TO_BALL:
            if Robot.bool_ball:
                cv2.rectangle(frame, ballBox1, ballBox2, (0, 255, 0), 2)

                ball_distance = calc_ball_dist(ballBox1, ballBox2)

                ball_center_x = (ballBox1[0] + ballBox2[0]) / 2
                frame_center_x = frame.shape[1] / 2
                tolerance = 20

                if abs(ball_center_x - frame_center_x) < tolerance and 11.5 <= ball_distance <= 13:
                    Robot.mission = SHORT_SHOT
            else:
                if ball_lost_start_time is not None and (time.time() - ball_lost_start_time) > ball_lost_threshold:
                    Robot.mission = FIND_BALL

        elif Robot.mission == SHORT_SHOT:
            Robot.bool_shot_zone, frame = Camera.analyze_short_path(img)
            shot_roi = True

            if Robot.bool_shot_zone in ["Shot", "R-Shot"]:
                if shot_roi and shot_count == 1:
                    Motion.prepare_shot_alignment() 
                    shot_roi = False
                Robot.bool_long_shot = False
                shot_direction = "Left" if Robot.bool_shot_zone == "Shot" else "Right"
                shot_power = 11 if Robot.bool_shot_zone == "Shot" else 6
                Robot.mission = SHOT
            elif Robot.bool_shot_zone == "NoHole" and shot_count == 1 and shot_roi:
                shot_roi = False
                neck_before_find = Robot.pitch
                Motion.adjust_neck(80)
                time.sleep(1)
                shortchecker = Camera.capture_frame()
                Robot.bool_shot_zone, shortchecker = Camera.analyze_short_path_R(shortchecker)
                Robot.mission = MOVE_TO_BALL
                if Robot.bool_shot_zone in ["L-rotate_body", "LL-rotate_body", "LLL-rotate_body", "LLLL-rotate_body"]:
                    for _ in range(Robot.bool_shot_zone.count("L")):
                        Motion.circular_orbit("Left", False)
                    if "LLL" in Robot.bool_shot_zone:
                        Motion.step_forward("BACK")
                        Motion.step_forward("BACK")
                    elif "LLLL" in Robot.bool_shot_zone:
                        Motion.side_step("LEFT")
                        Motion.side_step("LEFT")
                Motion.adjust_neck(neck_before_find)
                time.sleep(0.5)
            elif Robot.bool_shot_zone == "NoHole":
                Robot.mission = LONG_SHOT
                neck_before_find = Robot.pitch
                Motion.pitch = 70
                Motion.adjust_neck(70)
            elif Robot.bool_shot_zone in ["R-rotate_body-20", "L-rotate_body"]:
                if shot_roi and shot_count == 1:
                    shot_roi = False
                clockwise = "Left"
                Robot.rotation_angle = 200 if Robot.bool_shot_zone == "R-rotate_body-20" else 100
                Robot.mission = MOVE_TO_GOAL
            else:
                Robot.mission = CEREMONY

        elif Robot.mission == LONG_SHOT:
            if shot_count == 0:
                Robot.bool_shot_zone, frame, shot_power = Camera.analyze_long_path_par3_first_shot(img)
            else:
                Robot.bool_shot_zone, frame, shot_power = Camera.analyze_long_path(img)

            if Robot.bool_shot_zone == "Shot":
                if shot_power < 10:
                    Robot.bool_long_shot = False
                else:
                    Robot.bool_long_shot = True
                shot_direction = "Left"
                Robot.mission = SHOT
            else:
                Robot.mission = MOVE_TO_GOAL
                Robot.pitch = neck_before_find
                Motion.adjust_neck(Robot.pitch)
                if Robot.bool_shot_zone in ["R-rotate_body", "R-rotate_body-20", "R-rotate_body-10", "R-rotate_body-5"]:
                    clockwise = "Right"
                    Robot.rotation_angle = int(Robot.bool_shot_zone.split('-')[-1])
                elif Robot.bool_shot_zone in ["L-rotate_body", "L-rotate_body-20", "L-rotate_body-10"]:
                    clockwise = "Left"
                    Robot.rotation_angle = int(Robot.bool_shot_zone.split('-')[-1])
                else:
                    clockwise = "Left"
                    Robot.rotation_angle = 5

        elif Robot.mission == MOVE_TO_GOAL:
            Robot.mission = MOVE_TO_BALL
        elif Robot.mission == SHOT:
            shot_direction = "Left"
            Robot.mission = MOVE_TO_BALL
            shot_count += 1
            if shot_count == 1:
                shot_roi = True
        elif Robot.mission == CEREMONY:
            print("미션 종료")
            break

        if Motion.getLockStatus() and Robot.mission != MOVE_TO_BALL:
            pass
        elif Robot.mission == FIND_BALL:
            if shot_count == 0:
                if neck_before_find < 100:
                    neck_before_find += 10
                    Motion.adjust_neck(neck_before_find)
                else:
                    Motion.turn_body(clockwise)
                    neck_before_find = 50
            elif shot_count == 1:
                if neck_before_find < 80:
                    neck_before_find += 5
                    Motion.adjust_neck(neck_before_find)
                else:
                    Motion.turn_body(clockwise)
                    neck_before_find = 50
            else:
                if neck_before_find < 70:
                    neck_before_find += 5
                    Motion.adjust_neck(neck_before_find)
                else:
                    Motion.turn_body(clockwise)
                    neck_before_find = 50
        elif Robot.mission == MOVE_TO_BALL:
            (xmin, ymin) = ballBox1
            (xmax, ymax) = ballBox2
            xmean = (xmin + xmax) // 2
            ymean = (ymin + ymax) // 2

            if Motion.getLockStatus():
                  if xmean < 150:
                      Motion.side_step("Left")
                  elif xmean < 190:
                      Motion.side_step("Left")
                  elif xmean > 550:
                      Motion.side_step("Right")
                  elif xmean > 510:
                      Motion.side_step("Right")
                  elif ymean < 80 and Robot.pitch < 80:
                      Motion.adjust_neck(-20)
                  elif ymean < 100 and Robot.pitch < 100:
                      Motion.adjust_neck(-10)
                  elif ymean > 420 and Robot.pitch > -80:
                      Motion.adjust_neck(20)
                  elif ymean > 380 and Robot.pitch > -100:
                      Motion.adjust_neck(10)
                  elif 11.5 <= calc_ball_dist(ballBox1, ballBox2) <= 13:
                      Robot.mission = SHORT_SHOT
                  elif calc_ball_dist(ballBox1, ballBox2) > 15:
                      Motion.walk_continuously()
                  else:
                      Motion.step_forward()

        elif Robot.mission == SHORT_SHOT:
            if Motion.getLockStatus():
                Motion.adjust_neck(45)
        
        elif Robot.mission == LONG_SHOT:
            if Motion.getLockStatus():
                if shot_direction == "Left":
                    Motion.turn_left()
                    Motion.move_backward()
                    Motion.turn_left_slight()
                else:
                    Motion.turn_right_slight()
                Robot.mission = SHOT
        elif Robot.mission == MOVE_TO_GOAL:
            if Motion.getLockStatus():
                if shot_direction == "Left":
                    Motion.turn_left()
                else:
                    Motion.turn_right()
                if ball_distance > 13:
                    Motion.move_forward()
                else:
                    Motion.move_forward()
        elif Robot.mission == SHOT:
            if Motion.getLockStatus():
                if shot_direction == "Left":
                    Motion.shot(shot_power, "Left")
                else:
                    Motion.shot(shot_power, "Right")
                ball_distance = calc_ball_dist(ballBox1, ballBox2)
                if ball_distance > 13:
                    shot_power = min(max(int(ball_distance), 8), 15)
                else:
                    Motion.shot(shot_power, shot_direction)
        elif Robot.mission == CEREMONY:
            if Motion.getLockStatus():
                Motion.set_initial_position()
            print("celebration")
            sys.exit(1)

    cv2.destroyAllWindows()
