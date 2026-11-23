# One sign detected in a camera image.
# Class ids follow the Lab 6 label convention (team85_final.vision.signs.SignClass).
uint8 EMPTY=0
uint8 LEFT=1
uint8 RIGHT=2
uint8 DO_NOT_ENTER=3
uint8 STOP=4
uint8 GOAL=5

uint8 sign
string label
float32 confidence

# Bounding box in pixels of the source image (center + size).
float32 cx
float32 cy
float32 width
float32 height

# Horizontal angle of the box center from the optical axis (rad, + = left,
# i.e. counter-clockwise like ROS yaw). From cx and the camera's horizontal FOV.
float32 bearing
