import pybullet as p
from gym_shapesort.envs import env_cage
from gym_shapesort.envs import gripper_interface
import gym
import numpy as np
from random import random
from gym import spaces

	
def randf(start, end):
	return random()*(end-start)+start

class Reach(gym.Env):
	def __init__(self):

		self.frame_skip = 5

		p.connect(p.SHARED_MEMORY)
		p.resetSimulation()

		self.render_width = 224
		self.render_height = 224

		env_cage.init()
		self.body = gripper_interface.GripperInterface(p.loadURDF('/home/max/workspace/visiontoaction/description/gripper.urdf', [-0.55,-0.55,0]))
		self.target = p.loadURDF('/home/max/workspace/visiontoaction/description/target.urdf')

		linkpos = self.body.getLinkStates()[0]
		pinchpos = self.body.getPincherCentroid()
		self.armlength = pinchpos[0] - linkpos[0][0]

		self.camera_pos = [-0.7, 0, 1.0]

		self._reset_target()

		p.setGravity(0, 0, -9.81)

	def _simulate(self, action):
		self.body.setVelocities(action)

		for i in range(self.frame_skip):
			p.stepSimulation()

	def _get_obs(self):
		target_pos = (0, 0, 0)
		camera_up = (0, 0, 1)
		near_val, far_val = 1, 20
		fov = 80

		view_mat = p.computeViewMatrix(self.camera_pos, target_pos, camera_up)
		proj_mat = p.computeProjectionMatrixFOV(fov, 1.0, near_val, far_val)

		_w, _h, rgba, _depth, _objects = p.getCameraImage(
			self.render_width, self.render_height, 
			view_mat, proj_mat, shadow=1)

		# convert from 1d uint8 array to (H,W,3) hacky hardcode whitened float16 array.
		# TODO: for storage concerns could just store this as uint8 (which it is)
		# and normalise 0->1 + whiten later.
		rgba_img = np.reshape(np.asarray(rgba, dtype=np.float32),
		                      (self.render_height, self.render_width, 4))
		rgb_img = rgba_img[:,:,:3]  # slice off alpha, always 1.0
		rgb_img /= 255
		return rgb_img

	def _step(self, action):
		pinchpos = np.array(self.body.getPincherCentroid())
		reward_dist = - np.linalg.norm(self.targetpos - pinchpos)
		reward_ctrl = - np.square(action).sum()
		reward = reward_dist + reward_ctrl

		self._simulate(action)

		ob = self._get_obs()

		done = False
		return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

	def _reset_target(self):
		randpos = np.array([randf(-0.5, 0.5), randf(-0.4, 0.4), randf(-0.5, 0.5)])
		arm_root = np.array(p.getBasePositionAndOrientation(self.body.bodyid)[0])
		
		# ensure the target is reachable
		relpos = randpos - arm_root
		if np.linalg.norm(relpos) > self.armlength:
			relpos /= np.linalg.norm(relpos) * self.armlength

		self.targetpos = arm_root + relpos

		p.resetBasePositionAndOrientation(self.target, self.targetpos, [0,0,0,1])


	def _reset(self):
		self._reset_target()
		self.body.reset()

		return self._get_obs()
		

	def _render(self, mode='human', close=False):
		# rendering is done by bulletphysics if self.mode == p.GUI
		pass

	@property
	def action_space(self):
		control_rng = self.body.getControlRange()
		return spaces.Box(np.array(control_rng[0]), np.array(control_rng[1]))

	@property
	def observation_space(self):
		high = np.ones((self.render_height, self.render_width, 3))
		low = np.zeros((self.render_height, self.render_width, 3))
		return spaces.Box(low, high)

	
