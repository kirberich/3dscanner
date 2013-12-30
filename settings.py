### General settings
CAMERA_INDEX = 0 			# OpenCV camera index. If there is only one camera attached, this will be 0
VIEW_ANGLE_Y=0.785 			# Vertical view angle of the camera used (in radians)

### Scanning settings
LASER_THRESHOLD = 50 		# Difference threshold (in brightness values ranging 0-255) for distinguishing laser points
					 		# (Lower in case of false positives, increase in case of false negatives)
DETECTION='brightness' 		# Detection method for points, can be 'brightness' to use any bright point, 'red' for using red points 
ROTATION_STEP=2*math.pi/200 # Angle of a single step of the scanning platform's stepper motor (in radians)
FRAMES_PER_STEP=1 			# How many frames to take and average for every step

### Output settings
SCALE=.1 					# Scale the created object by this factor
FILL_HOLES=True 			# Interpolate gaps in measured data (per step) Note: this breaks everything for complex objects
MAKE_FACES=True 			# Create faces on the output model, output is a point cloud if this is set to False