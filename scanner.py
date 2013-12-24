import cv, cv2

import time
import numpy
import math

from structs import Box, Vertex

SQRT_2 = math.sqrt(2.0)
DEBUG = True

class Scanner(object):
	def __init__(self, laser_threshold=20, rotation_step=0.00314159265 * 30, camera_index=0, scale=.1, view_angle_y=0.785):
		self.calibration = None
		self.area = None
		self.mode = None
		self.keep_running = True

		self.zero_x = None
		self.rotation_angle = 0
		self.rotation_step = rotation_step

		self.width = None
		self.height = None
		self.scale = scale
		self.view_angle_y = view_angle_y

		self.frame = None
		self.laser_threshold = laser_threshold
		self.thresholded = None
		self.display_image = None
		self.processed_frames = []
		self.start_bottom_vertex = Vertex(0, 99999, 0)
		self.start_top_vertex = Vertex(0, -99999, 0)

		self.capture = cv2.VideoCapture(camera_index)
		self.window_name = "ui"
		cv2.namedWindow(self.window_name)
		cv2.setMouseCallback(self.window_name, self.handle_mouse)

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

	def record_frame(self):
		self.frame = cv2.imread("sketch1_laser.png")
		#self.frame = self.capture.read()[1]
		self.display_image = self.frame

		blue, green, red = cv2.split(self.frame)
		red = red - cv2.min(red, blue)
		red = red - cv2.min(red, green)

		retval, self.thresholded = cv2.threshold(red, self.laser_threshold, 255, cv2.THRESH_BINARY)

	def process_frame(self):
		area = self.thresholded[self.area.y1:self.area.y2, self.area.x1:self.area.x2] if self.area else self.thresholded

		if DEBUG:
			cv2.namedWindow("test")
			cv2.imshow("test", area)

		processed_frame = []
		points = numpy.transpose(area.nonzero())

		if len(points) < 2:
			self.processed_frames.append([])
			return

		row = points[0][0]
		row_medians = {row:0}
		num_row_points = {row:0}

		for y, x in points:
			row_medians[y] = row_medians.setdefault(y, 0) + x
			num_row_points[y] = num_row_points.setdefault(y, 0) + 1

		for point in row_medians.items():
			img_y, img_x = point
			# Average division happening here to save a loop
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

			x = math.cos(self.rotation_angle) * d
			y = self.height - math.sin(alpha)/math.sin(self.view_angle_y/2) * half_height
			z = math.sin(self.rotation_angle) * d
			processed_frame.append(Vertex(x*self.scale,y*self.scale,z*self.scale))
			
		processed_frame = sorted(processed_frame, key = lambda v: v.y)
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

	def normalize_y(self, vertex):
		""" Substracts the globally lowest y coordinate (self.start_bottom_vertex) from another vertex 
			to make the object touch the origin
		"""
		return Vertex(vertex.x, vertex.y-self.start_bottom_vertex.y, vertex.z)

	def save_image(self):
		f = open('output.obj', 'w')
		print "writing to file"
		vertex_index = 3 # Starts at 1, plus bottom center and top center points
		vertex_indices = []
		frame_index = 0
		num_frames = len(self.processed_frames)
		vertex_indices = []

		# Write bottom center and top center points
		f.write('v %s %s %s\n' % self.normalize_y(self.start_bottom_vertex).to_tuple())
		f.write('v %s %s %s\n' % self.normalize_y(self.start_top_vertex).to_tuple())

		for frame in self.processed_frames:
			vertex_indices.append([])
			for vertex in frame:
				f.write('v %s %s %s\n' % self.normalize_y(vertex).to_tuple())
				vertex_indices[frame_index].append(vertex_index)
				vertex_index += 1
			frame_index += 1

		frame_index = 0
		for frame in self.processed_frames:
			if frame_index > num_frames - 1:
				break
			vertex_index = 0
			num_vertices = len(frame)
			for vertex in frame:
				next_frame = (frame_index + 1) % num_frames
				v1 = vertex_indices[frame_index][vertex_index]
				v3 = vertex_indices[next_frame][vertex_index]
				if vertex_index == 0:
					v2 = 1
					f.write('f %s %s %s\n' % (v1, v2, v3))

				if vertex_index == num_vertices - 1:
					v2 = 2
					f.write('f %s %s %s\n' % (v1, v2, v3))
				else:
					v2 = vertex_indices[frame_index][vertex_index + 1]
					v4 = vertex_indices[next_frame][vertex_index + 1]
					f.write('f %s %s %s %s\n' % (v1, v2, v4, v3))
				vertex_index += 1
			frame_index += 1
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


scanner = Scanner()
scanner.loop()