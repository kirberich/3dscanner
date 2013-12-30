import time
import numpy
import math
import random
import copy
import serial
from bisect import bisect_right

import cv, cv2

from structs import Box, Vertex, Frame, Scan

SQRT_2 = math.sqrt(2.0)
DEBUG = True


class Scanner(object):
    def __init__(
            self, 
            laser_threshold=50, 
            detection='white', 
            rotation_step=2*math.pi/200,
            frames_per_step=1,
            camera_index=0, 
            scale=.1, 
            view_angle_y=0.785
            ):
        self.calibration = None
        self.area = None
        self.mode = None
        self.keep_running = True
        self.frames_per_step = frames_per_step

        self.zero_x = None
        self.rotation_angle = 0
        self.rotation_step = rotation_step

        self.width = None
        self.height = None
        self.scale = scale
        self.view_angle_y = view_angle_y

        self.frame = None
        self.last_raw_points = None
        self.laser_threshold = laser_threshold
        self.thresholded = None
        self.detection = detection
        self.display_image = None

        # Start numbering vertices at 3, to allow for the bottom and top center points
        self.processed_frames = Scan(vertex_index=3)
        self.start_bottom_vertex = Vertex(0, 99999, 0)
        self.start_top_vertex = Vertex(0, -99999, 0)

        self.capture = cv2.VideoCapture(camera_index)
        self.window_name = "ui"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.handle_mouse)

        # Arduino connection
        try:
            self.ser = serial.Serial('/dev/tty.usbmodem1411', 9600, timeout=1)
        except OSError:
            print "Couldn't establish serial connection"
            self.ser = None

    def send_command(self, command):
        """ Send serial command to arduino """
        if not self.ser:
            return []

        retval = []
        self.ser.write(command)
        retval.append(self.ser.readline().strip())
        while self.ser.inWaiting():
            retval.append(self.ser.readline().strip())
        return retval

    def calibrate(self, x=None):
        self.record_frame()
        self.height, self.width, channels = self.frame.shape

        self.zero_x = x or self.width/2
        self.calibration = Box(x1=self.zero_x, y1=0, x2=self.zero_x, y2=self.height)

    def handle_mouse(self, e, x, y, flags, param):
        if e == cv2.EVENT_LBUTTONDOWN:
            if self.mode == 'calibrate':
                self.calibrate(x)
            elif self.mode == 'box':
                self.area = Box(x, y, None, None)
        elif e == cv2.EVENT_LBUTTONUP:
            if self.mode == 'box':
                self.area.x2 = min(self.zero_x, x) or x
                self.area.y2 = y
            self.mode = None

    def handle_keyboard(self):
        k = cv2.waitKey(1) & 0xFF
        if k == 99:
            self.mode = 'calibrate'
        elif k == 98: # 'b' for box
            self.mode = 'box'
        elif k == 115: # 's' for scan
            self.mode = 'scan'
        elif k == 27:
            self.keep_running = False
        elif k == 105: # 'i' for info
            print self.send_command('d')

    def _capture_frame(self):
        #return cv2.imread("sketch1_laser.png")
        return self.capture.read()[1]

    def record_frame(self):
        self.frame = self._capture_frame()/self.frames_per_step
        for i in range(1, self.frames_per_step):
            self.frame = self.frame + self._capture_frame()/self.frames_per_step

        self.display_image = self.frame

        blue, green, red = cv2.split(self.frame)
        if self.detection == 'red':
            red = red - cv2.min(red, blue)
            red = red - cv2.min(red, green)
            to_threshold = red
        else:
            to_threshold = blue/3+green/3+red/3

        retval, self.thresholded = cv2.threshold(to_threshold, self.laser_threshold, 255, cv2.THRESH_BINARY)

    def process_frame(self):
        area = self.thresholded[self.area.y1:self.area.y2, self.area.x1:self.area.x2] if self.area else self.thresholded

        if DEBUG:
            cv2.namedWindow("test")
            cv2.imshow("test", area)

        processed_frame = Frame()
        points = numpy.transpose(area.nonzero())

        # Test empty frames
        # if random.random() < 0.05:
        #   points = []

        # If a frame is completely empty, use the last one
        # FIXME: This is at least bad because it doesn't interpolate between frames before and after a gap
        # But because this is happening online, the frames after the gap aren't known yet, so it stays like this for now
        if len(points) < 2:
            if self.last_raw_points is not None:
                points = self.last_raw_points
            else:
                print "no data in first frame(s)!"
                return

        self.last_raw_points = points
        row = points[0][0]
        row_medians = {row:0}
        num_row_points = {row:0}

        for y, x in points:
            row_medians[y] = row_medians.setdefault(y, 0) + x
            num_row_points[y] = num_row_points.setdefault(y, 0) + 1

        last_point = None
        last_vertex = None

        row_median_list = sorted(row_medians.items(), key = lambda p: p[0])
        for point in row_median_list:
            # Test frames with different amounts of vertices
            # if random.random() < 0.05:
            #   continue
            img_y, img_x = point
            # Average division for x coordinates in row happening here to save a loop
            img_x = float(img_x)/num_row_points[img_y] 

            if self.area:
                img_y += self.area.y1
                img_x += self.area.x1

            # horizontal distance from middle of frame (and therefore middle of object)
            delta_x = float(self.zero_x) - img_x
            d = SQRT_2*delta_x

            # vertical angle to point from middle of frame
            half_height = float(self.height)/2
            y_from_middle = img_y - half_height
            alpha = math.atan((y_from_middle/half_height) * self.view_angle_y/2)

            x = math.cos(self.rotation_angle) * d * self.scale
            y = self.height - math.sin(alpha)/math.sin(self.view_angle_y/2) * half_height * self.scale
            z = math.sin(self.rotation_angle) * d * self.scale

            # If some (image) y coordinates didn't contain any data, interpolate their values
            # FIXME: This is quite naive, it doesn't allow for holes or other complex geometries
            if last_point and last_point[0] < img_y - 1:
                d_img_y = last_point[0] + 1
                total_diff = img_y - last_point[0]
                while d_img_y < img_y:
                    dx = (float(img_y - d_img_y)/total_diff) * last_vertex.x + (float(d_img_y-last_point[0])/total_diff) * x
                    dy = (float(img_y - d_img_y)/total_diff) * last_vertex.y + (float(d_img_y-last_point[0])/total_diff) * y
                    dz = (float(img_y - d_img_y)/total_diff) * last_vertex.z + (float(d_img_y-last_point[0])/total_diff) * z
                    processed_frame.append(Vertex(dx, dy, dz))
                    d_img_y += 1

            vertex = Vertex(x, y, z)
            processed_frame.append(vertex)
            last_point = img_y, img_x
            last_vertex = vertex

        self.processed_frames.append(processed_frame)

        # Update top and bottom center points
        if processed_frame[0].y < self.start_bottom_vertex.y:
            self.start_bottom_vertex = Vertex(0, processed_frame[0].y, 0)
        if processed_frame[-1].y > self.start_top_vertex.y:
            self.start_top_vertex = Vertex(0, processed_frame[-1].y, 0)

    def display_frame(self):
        if self.calibration:
            x1, y1, x2, y2 = [int(c) for c in self.calibration.to_tuple()]
            cv2.rectangle(self.display_image, (x1, y1), (x2, y2), (100, 255, 0))

        if self.area and self.area.is_complete():
            x1, y1, x2, y2 = [int(c) for c in self.area.to_tuple()]
            cv2.rectangle(self.display_image, (x1, y1), (x2, y2), (255, 100, 0))

        cv2.imshow(self.window_name, self.display_image)

    def rotate(self):
        self.rotation_angle += self.rotation_step
        print self.send_command('s')

    def normalize_y(self, vertex):
        """ Substracts the globally lowest y coordinate (self.start_bottom_vertex) from another vertex 
            to make the object touch the origin
        """
        return Vertex(vertex.x, vertex.y-self.start_bottom_vertex.y, vertex.z)

    def closest_vertex_y(self, y, frame):
        """ Finds vertex in frame the y coordinate of which is closest to y """

        'Find rightmost value less than or equal to x'
        return max(0, bisect_right([v.y for v in frame], y)-1)

    def save_image(self):
        f = open('output.obj', 'w')
        print "writing to file"

        # Write bottom center and top center points
        f.write('v %s %s %s\n' % self.normalize_y(self.start_bottom_vertex).to_tuple())
        f.write('v %s %s %s\n' % self.normalize_y(self.start_top_vertex).to_tuple())

        for frame in self.processed_frames:
            for vertex in frame:
                f.write('v %s %s %s\n' % self.normalize_y(vertex).to_tuple())

        for frame in self.processed_frames:
            next_frame = self.processed_frames.next_frame()

            for vertex in frame:
                v1 = vertex.index
                if frame.current_vertex_index == 0:
                    v2 = next_frame[0].index
                    v3 = 1
                    f.write('f %s %s %s\n' % (v1, v2, v3))

                if frame.current_vertex_index == frame.num_vertices - 1:
                    # If the next frame has more vertices, insert faces from current frame's last vertex to them
                    if frame.num_vertices < next_frame.num_vertices:
                        for extra_vertex in range(frame.num_vertices-1, next_frame.num_vertices-1):
                            v2 = next_frame[extra_vertex+1].index
                            v3 = next_frame[extra_vertex].index
                            f.write('f %s %s %s\n' % (v1, v2, v3))
                    v2 = 2
                    v3 = next_frame[-1].index
                    f.write('f %s %s %s\n' % (v1, v2, v3))
                else:
                    v2 = frame.next_vertex().index
                    v3 = next_frame.get_vertex(frame.current_vertex_index + 1, or_last=True).index
                    v4 = next_frame.get_vertex(frame.current_vertex_index, or_last=True).index
                    if v3 == v4:
                        f.write('f %s %s %s\n' % (v1, v2, v3))
                    else:
                        f.write('f %s %s %s %s\n' % (v1, v2, v3, v4))
                        
        print "done writing"
        f.close()

    def loop(self):
        frame = 0
        self.record_frame()
        self.calibrate()

        while self.keep_running:
            self.handle_keyboard()
            self.record_frame()

            if self.mode == 'scan':
                if self.rotation_angle > math.pi*2:
                    self.mode = None
                    self.save_image()
                else:
                    self.process_frame()
                    if frame % 100:
                        print self.rotation_angle
                    self.rotate()
            self.display_frame()
            frame += 1


scanner = Scanner(detection='white', frames_per_step=5)
scanner.loop()