import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import signal
import keyboard
import time
import bb_pwm # library we wrote
from pid import Error_PID_Controller

# based on: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/




def define_globals():
    global throttle_pin
    throttle_pin = "P9_14"
    #go_forward = 7.91

    # Steering
    global steering_pin
    steering_pin = "P9_16"

    # Max number of loops
    global max_ticks
    max_ticks = 2000

    # Booleans for handling stop light
    global passed_stop_light
    passed_stop_light = False
    global at_stop_light
    at_stop_light = False
    global passed_first_stop_sign
    passed_first_stop_sign = False

    global PWM
    PWM = bb_pwm.BB_PWM()

    # this is a PID Controller that should be used to control the car's steering
    global steering_pid
    steering_pid = Error_PID_Controller()

    # this is a PID Controller that should be used to control the speed of the car
    global speed_pid
    speed_pid = Error_PID_Controller()

    # this boolean is used to tell the car to stop
    # it should override any output from the PID controller
    global stopped
    stopped = True

    global counter
    counter = 0

    # arrays for making the final graphs
    global p_vals
    p_vals = []
    global d_vals
    d_vals = []
    global err_vals
    err_vals = []
    global speed_pwm
    speed_pwm = []
    global steer_pwm
    steer_pwm = []

    global stopSignCheck
    stopSignCheck = 1
    global sightDebug
    sightDebug = False
    global isStop2SignBool
    isStopSignBool = False


def getRedFloorBoundaries():
    """
    Gets the hsv boundaries and success boundaries indicating if the floor is red
    :return: [[lower color and success boundaries for red floor], [upper color and success boundaries for red floor]]
    """
    return getBoundaries("redboundaries.txt")


def isRedFloorVisible(frame):
    """
    Detects whether or not the floor is red
    :param frame: Image
    :return: [(True is the camera sees a red on the floor, false otherwise), video output]
    """
    print("Checking for floor stop")
    boundaries = getRedFloorBoundaries()
    return isMostlyColor(frame, boundaries)


def getTrafficRedLightBoundaries():
    """
    Gets the traffic red light hsv boundaries and success boundaries
    :return: [[lower color and success boundaries for red light], [upper color and success boundaries for red light]]
    """
    return getBoundaries("trafficRedBoundaries.txt")


def isTrafficRedLightVisible(frame):
    """
    Detects whether or not we can see a stop sign
    :param frame:
    :return: [(True is the camera sees a stop light, false otherwise), video output]
    """
    #print("Checking for traffic stop")
    boundaries = getTrafficRedLightBoundaries()
    return isMostlyColor(frame, boundaries)


def getTrafficGreenLightBoundaries():
    """
    Gets the traffic green light hsv boundaries and success boundaries
    :return: [[lower color and success boundaries for green light], [upper color and success boundaries for green light]]
    """
    return getBoundaries("trafficGreenboundaries.txt")


def isTrafficGreenLightVisible(frame):
    """
    Detects whether or not we can see a green traffic light
    :param frame:
    :return: [(True is the camera sees a green light, false otherwise), video output]
    """
    #print("Checking For Green Light")
    boundaries = getTrafficGreenLightBoundaries()
    return isMostlyColor(frame, boundaries)


def isMostlyColor(image, boundaries):
    """
    Detects whether or not the majority of a color on the screen is a particular color
    :param image:
    :param boundaries: [[color boundaries], [success boundaries]]
    :return: boolean if image satisfies provided boundaries, and an image used for debugging
    """
    #Convert to HSV color space
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #parse out the color boundaries and the success boundaries
    color_boundaries = boundaries[0]
    percentage = boundaries[1]

    lower = np.array(color_boundaries[0])
    upper = np.array(color_boundaries[1])
    mask = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

    #Calculate what percentage of image falls between color boundaries
    percentage_detected = np.count_nonzero(mask) * 100 / np.size(mask)
    #print("percentage_detected " + str(percentage_detected) + " lower " + str(lower) + " upper " + str(upper))
    # If the percentage percentage_detected is betweeen the success boundaries, we return true, otherwise false for result
    result = percentage[0] < percentage_detected <= percentage[1]
    #if result:
        #(percentage_detected)
    return result, output


def getBoundaries(filename):
    """
    Reads the boundaries from the file filename
    Format:
        [0] lower: [H, S, V, lower percentage for classification of success]
        [1] upper: [H, S, V, upper percentage for classification of success]
    :param filename: file containing boundary information as above
    :return: [[lower color and success boundaries], [upper color and success boundaries]]
    """
    default_lower_percent = 50
    default_upper_percent = 100
    with open(filename, "r") as f:
        boundaries = f.readlines()
        lower_data = [val for val in boundaries[0].split(",")]
        upper_data = [val for val in boundaries[1].split(",")]

        if len(lower_data) >= 4:
            lower_percent = float(lower_data[3])
        else:
            lower_percent = default_lower_percent

        if len(upper_data) >= 4:
            upper_percent = float(upper_data[3])
        else:
            upper_percent = default_upper_percent

        lower = [int(x) for x in lower_data[:3]]
        upper = [int(x) for x in upper_data[:3]]
        boundaries = [lower, upper]
        percentages = [lower_percent, upper_percent]
    return boundaries, percentages


def initialize_car():
    # give 7.75% duty at 50Hz to throttle
    PWM.default_vals(throttle_pin)
    # wait for car to be ready
    print("Set throttle to default value, press enter when ESC calibrated")
    input()
    PWM.default_vals(steering_pin)
    return


def stop():
    """
    Stops the car
    :return: none
    """
    PWM.default_vals(throttle_pin)
    global stopped
    stopped = True


def go():
    """
    Sends the car forward at a default PWM
    :return: none
    """
    #PWM.set_duty_cycle(throttle_pin, go_forward)
    global stopped
    stopped = False
    return


def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype="uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # cv2.imshow("mask",mask)

    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    # cv2.imshow("edges",edges)

    return edges


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("roi",cropped_edges)

    return cropped_edges


def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments


def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines


def make_points(frame, line):
    height, width, _ = frame.shape

    slope, intercept = line

    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi

    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle


def plot_pd(p_vals, d_vals, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    plt.clf()


def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed PWM")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering PWM")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PWM Values")
    ax2.set_ylabel("Error Value")

    plt.title("PWM Values over time")
    fig.legend()
    plt.savefig("pwm_plot.png")

    plt.clf()
    
def get_encoder_time():
	# open the driver
	driver_file = open("/dev/encoder_driver")
	# read from the driver
	line = driver_file.readline()
	# convert to an int
	line = line.strip()
	value = int(line)
	# close driver
	driver_file.close()
	# return the value
	return value


def update_throttle():
    # TODO this
    # get value from PID
    # verify not too high or low
    # set PWM
    return

def update_steering():
    # TODO this
    # get value from PID
    # verify not too high or low
    # set PWM
    return


def init_video():
    global video
    # set up video
    video = cv2.VideoCapture(2)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    # wait for video to load
    time.sleep(1)


def init_pids():
    global steering_pid
    global speed_pid

    steering_pid.set_p_gain(0)
    steering_pid.set_i_gain(0)
    steering_pid.set_d_gain(0)

    speed_pid.set_p_gain(0)
    speed_pid.set_i_gain(0)
    speed_pid.set_d_gain(0)

    # degrees from straight
    steering_pid.set_target(0)

    # convert times from ns to us
    speed_pid.set_target(40000) # set to 40 ms


    # tune PIDs here


def main_init():
    define_globals()
    initialize_car()
    init_video()
    init_pids()


def main_loop():

    go()

    global p_vals
    global d_vals
    global err_vals
    global speed_pwm
    global steer_pwm
    global counter
    global stopSignCheck
    global passed_first_stop_sign
    global secondStopSignTick

    cv2.namedWindow("original")
    cv2.namedWindow("heading line")


    while counter < max_ticks:
        # print counter value to console
        print(counter)
        # read frame
        ret, original_frame = video.read()
        # copy/resize frame
        frame = cv2.resize(original_frame, (160, 120))
        if sightDebug:
            cv2.imshow("Resized Frame", frame)

        # check for stop sign/traffic light every couple ticks
        if ((counter + 1) % stopSignCheck) == 0:

            # removed stop light logic - not needed for undergrad teams

            # check for the first stop sign
            if not passed_first_stop_sign:
                isStopSignBool, floorSight = isRedFloorVisible(frame)
                if sightDebug:
                    cv2.imshow("floorSight", floorSight)
                if isStopSignBool:
                    print("Detected first stop sign, stopping")
                    stop()
                    time.sleep(2)
                    passed_first_stop_sign = True
                    # this is used to not check for the second stop sign until many frames later
                    secondStopSignTick = counter + 500
                    # now check for stop sign less frequently
                    stopSignCheck = 3
                    print("first stop finished!")
            # check for the second stop sign
            elif passed_first_stop_sign and counter > secondStopSignTick:
                isStop2SignBool, _ = isRedFloorVisible(frame)
                print("is a floor stop: ", isStopSignBool)
                if isStop2SignBool:
                    # last stop sign detected, exits while loop
                    print("detected second stop sign, stopping")
                    stop()
                    break
        
        # removed more stop light logic here - not needed

        # process the frame to determine the desired steering angle
        cv2.imshow("original",frame)
        edges = detect_edges(frame)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame, line_segments)
        lane_lines_image = display_lines(frame, lane_lines)
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        cv2.imshow("heading line",heading_image)

        # calculate changes for PD
        #now = time.time()
        #dt = now - lastTime
        if sightDebug:
            cv2.imshow("Cropped sight", roi)
        deviation = steering_angle - 90
        # note positive deviation means need to turn right
        # negative deviation means we need to turn left

        print("deviation: " + deviation.__str__())
        print("Steering angle: " + steering_angle.__str__())

        # PD Code
        #error = -deviation
        #base_turn = 7.5
        #proportional = kp * error
        #derivative = kd * (error - lastError) / dt

        # take values for graphs
        #p_vals.append(proportional)
        #d_vals.append(derivative)
        #err_vals.append(error)

        # determine actual turn to do
        #turn_amt = base_turn + proportional + derivative

        # TODO implement turning here

        # TODO implemented SPEED PID here

        # take values for graphs
        #steer_pwm.append(turn_amt)
        #speed_pwm.append(current_speed)

        # update PD values for next loop
        #lastError = error
        #lastTime = time.time()



        counter += 1


def cleanup():
    # clean up resources
    video.release()
    cv2.destroyAllWindows()
    #PWM.set_duty_cycle(throttle_pin, 7.5)
    #PWM.set_duty_cycle(steering_pin, 7.5)
    PWM.default_vals(throttle_pin)
    PWM.default_vals(steering_pin)
    #PWM.stop(throttle_pin)
    #PWM.stop(steering_pin)
    #PWM.cleanup()

    plot_pd(p_vals, d_vals, err_vals, True)
    plot_pwm(speed_pwm, steer_pwm, err_vals, True)






# Main Script here


# Main Script here
# Functions defined below

main_init()
try:
    main_loop()
except KeyboardInterrupt:
    print("Received command to exit early!")
cleanup()

