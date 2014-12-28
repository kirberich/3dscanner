import math

### General settings
CAMERA_INDEX = 0            # OpenCV camera index. If there is only one camera attached, this will be 0
FOV=1.047                   # Diagonal field of view of camera used (in radians)

### Scanning settings
LASER_THRESHOLD = 150       # Difference threshold (in brightness values ranging 0-255) for distinguishing laser points
                            # (Lower in case of false positives, increase in case of false negatives)

ROTATION_STEP=2*math.pi/200 # Angle of a single step of the scanning platform's stepper motor (in radians)

### Output settings
SCALE=1.0                   # Scale the created object by this factor
