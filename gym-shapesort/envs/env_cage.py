import pybullet as p
import math

def init():
	plane_desc = '/home/max/workspace/visiontoaction/description/plane.urdf'
	wall_desc = '/home/max/workspace/visiontoaction/description/wall.urdf'

	arm_length = 2.0
	wall_thickness = 0.2
	wall_distance = arm_length + wall_thickness

	floor 	  = p.loadURDF(plane_desc, [0, 0, -0.5])
	wall_left = p.loadURDF(wall_desc,  [0, wall_distance/2, 0])
	wall_right = p.loadURDF(wall_desc, [0, -wall_distance/2, 0])

	pi_2_rot = p.getQuaternionFromEuler([0, 0, math.pi/2])
	wall_front = p.loadURDF(wall_desc, [wall_distance/2, 0, 0], pi_2_rot)
	# wall_back = p.loadURDF(wall_desc, [-wall_distance/2, 0, 0], pi_2_rot)

	return floor, wall_left, wall_right, wall_front