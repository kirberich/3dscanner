import sys

from scanner import Scanner
import settings

scanner = Scanner(
    camera_index=settings.CAMERA_INDEX,
    fov=settings.FOV,
    laser_threshold=settings.LASER_THRESHOLD,
    rotation_step=settings.ROTATION_STEP,
    scale=settings.SCALE,
    device=sys.argv[1]
)
scanner.loop()
