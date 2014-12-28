import time
import numpy
from numpy import array
import math
import random
import copy
import serial
from datetime import datetime, timedelta
from bisect import bisect_right

import cv, cv2

from models import Vertex, Frame, Scan, Box
from utils import rotate

SQRT_2 = math.sqrt(2.0)
DEBUG = True


def debug(message):
    if DEBUG:
        print message

class Scanner(object):
    def __init__(
                self,
                device,
                laser_threshold=50,
                rotation_step=2*math.pi/200,
                camera_index=0,
                scale=10.0,
                fov=1.047,
                format_ratio=1.7777
            ):

        self.device = device

        self.area = None
        self.mode = None
        self.keep_running = True

        self.platform_middle = None
        self.rotation_angle = 0
        self.rotation_step = rotation_step

        self.width = None
        self.height = None
        self.scale = scale
        self.fov = fov
        self.format_ratio = format_ratio
        self.fov_x = format_ratio*fov/math.sqrt(1+format_ratio**2)
        self.fov_y = fov/math.sqrt(1+format_ratio**2)
        # FIXME: This should be calibrated, not hardcoded
        # To be able to calculate the camera position, we need the camera angle and (for example) the size of the platform
        # m = rotate(array([1, 0, 0]), numpy.array([0, 1, 0]), camera_angle) # Straight ahead rotated by camera angle
        self.camera_position = array([0, -0.3, 0.15])

        self.last_raw_points = None
        self.laser_threshold = laser_threshold
        self.is_laser_on = False
        self.display_image = None

        self.processed_frames = Scan()

        self.ui_messages = []
        self.capture = cv2.VideoCapture(camera_index)
        self.window_name = "ui"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.handle_mouse)

        # Arduino connection
        try:
            self.ser = serial.Serial(device, 9600, timeout=1)
        except OSError:
            print "Couldn't establish serial connection"
            self.ser = None

    def send_command(self, command, wait_for_reply=False):
        """ Send serial command to arduino """
        if not self.ser:
            return

        self.ser.write(command)
        if wait_for_reply:
            self.ser.readline().strip()

    def calibrate(self, x=None):
        frame = self.capture_diff()
        self.height, self.width, channels = frame.shape

    def handle_mouse(self, e, x, y, flags, param):
        if self.mode == 'set_middle':
            if e == cv2.EVENT_LBUTTONDOWN:
                self.platform_middle = (x, y)
                self.mode = None
        else:
            if e == cv2.EVENT_LBUTTONDOWN:
                self.area = Box(x, y, None, None)
            elif e == cv2.EVENT_LBUTTONUP:
                self.area.x2 = x
                self.area.y2 = y

    def add_message(self, message):
        if len(self.ui_messages) >= 5:
            self.ui_messages = self.ui_messages[-5:]
        self.ui_messages.append((datetime.now(), message))

    def clear_old_messages(self):
        new_messages = []
        now = datetime.now()
        max_delta = timedelta(seconds=5)
        for (timestamp, message) in self.ui_messages:
            if now - timestamp < max_delta:
                new_messages.append((timestamp, message))
        self.ui_messages = new_messages

    def handle_keyboard(self):
        k = cv2.waitKey(1) & 0xFF
        if k == 115: # 's' for scan
            if not self.platform_middle:
                self.add_message('Please set platform middle before scanning (press m and click)')
                return
            # Enable motors to avoid a delay on the first step
            self.send_command('M')
            time.sleep(0.5)
            self.mode = 'scan'
        elif k == 27:
            if self.mode == 'scan':
                self.mode = 'stop_scan'
            else:
                self.keep_running = False
        elif k == 105: # 'i' for info
            print self.send_command('d')
        elif k == 108:
            self.set_laser(not self.is_laser_on)
        elif k == 109: # 'm' to set platform middle
            self.mode = 'set_middle'

    def _capture_frame(self):
        # if self.is_laser_on:
        #     return cv2.imread("sketch1_laser.png")
        # return cv2.imread("sketch1.png")
        return self.capture.read()[1]

    def set_laser(self, is_on):
        command = 'L' if is_on else 'l'
        self.send_command(command)
        self.is_laser_on = is_on

    def capture_diff(self, thresholded=False):
        # Captures two frames, one with laser on and one with laser off
        # Set thresholded to True to return the frame with self.laser_threshold applied

        lower_red = numpy.array([15,50,50])
        upper_red = numpy.array([165,255,255])

        # Threshold the HSV image to get only red colors


        # Capture frame with laser on
        self.set_laser(True)
        time.sleep(0.2)
        frame = self._capture_frame()



        # Set frame with laser to be displayed

        # Capture frame with laser off
        self.set_laser(False)
        time.sleep(0.2)
        frame_no_laser = self._capture_frame()

        frame_diff = self.red_filter(cv2.absdiff(frame, frame_no_laser))
        self.display_image = frame_diff
        if not thresholded:
            return frame_diff

        blue, green, red = cv2.split(frame_diff)
        retval, thresholded = cv2.threshold(red, self.laser_threshold, 255, cv2.THRESH_BINARY)
        return thresholded

    def get_laser_plane_intersection(self, v):
        """ Calculates the intersection of a vector with the laser plane.
            The camera position is represented by the vector [0, c_y, c_z], where c_y and c_z are the camera coordinates.
            c_y should be negative, as the y axis is positive away from the camera, starting at the platform middle.
            The plane is represented by the plane created by the vectors [1,1,0] and [0,0,1]
        """
        c_y, c_z = self.camera_position[1:3]
        x, y, z = v[:3]
        lam = c_y/(x-y)

        return array([
            lam*x,
            lam*y + c_y,
            lam*z + c_z
        ])
        return v + array([0, ])

    def process_frame(self, thresholded_frame):
        if self.area and self.area.is_complete:
            thresholded_frame = thresholded_frame[self.area.y1:self.area.y2, self.area.x1:self.area.x2]

        processed_frame = Frame()
        points = numpy.transpose(thresholded_frame.nonzero())

        if not len(points):
            return

        # Precalculations
        tan_half_fov_x = math.tan(self.fov_x/2)
        tan_half_fov_y = math.tan(self.fov_y/2)

        # m is the vector from the camera position to the origin
        m = self.camera_position * -1
        w = self.width/2
        h = self.height/2

        for point in points:
            img_y, img_x = point

            if self.area and self.area.is_complete:
                img_y += self.area.y1
                img_x += self.area.x1

            # Horizontal angle between platform middle (in image) and point
            delta_x = float(img_x - self.platform_middle[0])/2
            tau = math.atan(delta_x/w*tan_half_fov_x)

            # Vertical angle
            delta_y = float(img_y - self.platform_middle[1])/2
            rho = math.atan(delta_y/h*tan_half_fov_y)

            # Rotate vector m around tau and rho to point towards 'point'
            v = m
            v = rotate('z', v, tau) # Rotate around z axis for horizontal angle
            v = rotate('x', v, rho) # Rotate around x axis for vertical angle

            v = self.get_laser_plane_intersection(v)

            # Ignore any vertices that have negative z coordinates (pre scaling)
            if v[2] < 0:
                continue

            x,y,z = v*self.scale
            x,y,z = rotate('z', v, self.rotation_angle)

            vertex = Vertex(x, y, z)
            processed_frame.append(vertex)

        self.processed_frames.append(processed_frame)

    def show_frame(self):
        cv2.line(self.display_image, (self.width/2, 0), (self.width/2, self.height), (255, 255, 255))
        cv2.line(self.display_image, (0, self.height/2), (self.width, self.height/2), (255, 255, 255))

        if self.platform_middle:
            cv2.line(
                self.display_image,
                (self.platform_middle[0]-10, self.platform_middle[1]),
                (self.platform_middle[0]+10, self.platform_middle[1]),
                (255, 0, 0)
            )
            cv2.line(
                self.display_image,
                (self.platform_middle[0], self.platform_middle[1]-10),
                (self.platform_middle[0], self.platform_middle[1]+10),
                (255, 0, 0)
            )

        if self.area and self.area.is_complete():
            x1, y1, x2, y2 = [int(c) for c in self.area.to_tuple()]
            cv2.rectangle(self.display_image, (x1, y1), (x2, y2), (255, 100, 0))

        for index, (timestamp, message) in enumerate(self.ui_messages):
            cv2.putText(self.display_image, message, (10, self.height-10-15*index), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (255, 255, 255))

        cv2.imshow(self.window_name, self.display_image)

    def rotate(self):
        self.rotation_angle += self.rotation_step
        self.send_command('s')

    def closest_vertex_y(self, y, frame):
        """ Finds vertex in frame the y coordinate of which is closest to y """

        'Find rightmost value less than or equal to x'
        return max(0, bisect_right([v.y for v in frame], y)-1)

    def save_image(self):
        f = open('output.obj', 'w')
        print "writing to file..."

        for frame in self.processed_frames:
            for vertex in frame:
                f.write('v %s %s %s\n' % vertex.to_tuple())

        f.close()
        self.processed_frames = []
        print "Done writing output file."

    def red_filter(self, image):
        begin_lower_border = numpy.array([0, 50, 20])
        begin_upper_border = numpy.array([35, 255, 255])

        end_lower_border = numpy.array([145, 50, 20])
        end_upper_border = numpy.array([180, 255, 255])

        white_lower_border = numpy.array([0, 0, 180])
        white_upper_border = numpy.array([180, 255, 255])

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(image, begin_lower_border, begin_upper_border)
        mask2 = cv2.inRange(image, end_lower_border, end_upper_border)
        mask3 = cv2.inRange(image, white_lower_border, white_upper_border)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.bitwise_or(mask, mask3)
        image = cv2.bitwise_and(image, image, mask=mask)
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)

        return image

    def loop(self):
        frame = 0
        self.calibrate()

        while self.keep_running:
            self.handle_keyboard()

            if self.mode == 'scan' or self.mode == 'stop_scan':
                thresholded_frame = self.capture_diff(thresholded=True)
                if self.rotation_angle > math.pi*2 or self.mode == 'stop_scan':
                    self.save_image()
                    self.mode = None
                    self.rotation_angle = 0
                    self.set_laser(False)
                    continue

                self.process_frame(thresholded_frame)
                if frame % 100:
                    debug(self.rotation_angle)
                self.rotate()
                time.sleep(0.1)
            else:
                self.display_image = self._capture_frame()

            self.clear_old_messages()
            self.show_frame()
            frame += 1
