import pybullet as p
import math
import pkg_resources

def init():
	plane_desc = pkg_resources.resource_filename('gym_shapesort', 'description/plane.urdf')
	wall_desc = pkg_resources.resource_filename('gym_shapesort', 'description/wall.urdf')
	print(plane_desc)

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