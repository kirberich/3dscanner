from scanner import Scanner
import settings

scanner = Scanner(
	camera_index=settings.CAMERA_INDEX,
	view_angle_y=settings.view_angle_y,
	laser_threshold=settings.LASER_THRESHOLD,
	detection=settings.DETECTION,
	rotation_step=settings.ROTATION_STEP,
	frames_per_step=settings.FRAMES_PER_STEP,
	scale=settings.SCALE,
	fill_holes=settings.FILL_HOLES,
	make_faces=settings.MAKE_FACES
	)
scanner.loop()