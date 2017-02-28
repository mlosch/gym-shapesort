import pybullet as p
from gym_shapesort.envs import cage_world
from gym_shapesort.envs import gripper_interface
import gym
import numpy as np
from random import random
from gym import spaces
import pkg_resources
import matplotlib.pyplot as plt
from skimage.transform import resize as imresize

	
def randf(start, end):
	return random()*(end-start)+start

class Reacher(gym.Env):
	metadata = {'render.modes': ['human'], 'video.frames_per_second': 1}

	def __init__(self, 
		frame_skip=2, 
		obs_type='rgb', 
		render_width=256, render_height=256, 
		image_width=112, image_height=112, 
		pybullet_mode='SHARED_MEMORY'):

		self.frame_skip = frame_skip
		self.obs_type = obs_type

		pybullet_connect = {
			'SHARED_MEMORY': p.SHARED_MEMORY,
			'GUI': p.GUI,
			'DIRECT': p.DIRECT
		}
		p.connect(pybullet_connect[pybullet_mode.upper()])
		p.setRealTimeSimulation(0)
		p.resetSimulation()

		self.render_width = render_width
		self.render_height = render_height
		self.image_width = image_width
		self.image_height = image_height

		self.plt_initialized = False
		self.current_obs = None

		# cage_world.init()

		arm_desc = pkg_resources.resource_filename('gym_shapesort', 'description/gripper.urdf')
		target_desc = pkg_resources.resource_filename('gym_shapesort', 'description/target.urdf')

		self.body = gripper_interface.GripperInterface(p.loadURDF(arm_desc, [-0.55,0,0]))
		self.target = p.loadURDF(target_desc)

		linkpos = self.body.getLinkStates()[0]
		pinchpos = self.body.getPincherCentroid()
		self.armlength = pinchpos[0] - linkpos[0][0]

		camera_pos = {
			'q': None,
			'rgb': [(-0.7, 0, 1.0)], 
			'rgbd': [(-0.7, 0, 1.0)], 
			'birgb': [(-0.7, 0.1, 1.0), (-0.7, -0.1, 1.0)]
		}
		self.camera_pos = camera_pos[obs_type]

		self._reset_target()

		p.setGravity(0, 0, -9.81)

	def _simulate(self, action):
		self.body.setVelocities(action)

		for i in range(self.frame_skip):
			p.stepSimulation()

	def _get_obs(self):
		if self.obs_type == 'q':
			pinchpos = np.array(self.body.getPincherCentroid())
			state, velocities = self.body.getJointStates()
			q = np.array(state)
			qdot = np.array(velocities)
			dist = self.targetpos - pinchpos
			return np.concatenate((q, qdot, dist), axis=0)

		channels = 3
		if self.obs_type == 'rgbd':
			channels = 4

		obs = np.empty((self.image_height, self.image_width, channels*len(self.camera_pos)))

		target_pos = (0, 0, 0)
		camera_up = (0, 0, 1)
		near_val, far_val = 0.01, 5
		fov = 80
		proj_mat = p.computeProjectionMatrixFOV(fov, 1.0, near_val, far_val)

		for i, camera_pos in enumerate(self.camera_pos):
			view_mat = p.computeViewMatrix(camera_pos, target_pos, camera_up)

			_w, _h, rgba, depth, _objects = p.getCameraImage(
				self.render_width, self.render_height, 
				view_mat, proj_mat, shadow=0)

			rgba = rgba.astype(np.float32) / 255.0

			if self.obs_type == 'rgbd':
				depth = depth.clip(0, 5)
				rgba[:,:,3] = imresize(depth, (self.image_height, self.image_width))
			else:
				rgba = imresize(rgba[:,:,:3], (self.image_height, self.image_width))  # slice off alpha, always 1.0
			obs[:,:,(i*channels):((i+1)*channels)] = rgba

		# import matplotlib.pyplot as plt
		# for i in range(len(self.camera_pos)):
		# 	plt.subplot(1,len(self.camera_pos), i+1)
		# 	plt.imshow(obs[:,:,(i*3):((i+1)*3)])
		# plt.show()

		return obs

	def _step(self, action):
		pinchpos = np.array(self.body.getPincherCentroid())
		reward_dist = - np.linalg.norm(self.targetpos - pinchpos)

		act_space = self.action_space
		norm_action = (action - act_space.low) / (act_space.high - act_space.low)
		norm_action -= 0.5

		reward_ctrl = - np.square(norm_action).sum()
		reward = reward_dist + reward_ctrl

		# print(reward_dist, reward_ctrl)

		self._simulate(action)

		ob = self._get_obs()
		self.current_obs = ob

		done = False
		return ob, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

	def _reset_target(self):
		# randpos = np.array([randf(-0.5, 0.5), randf(-0.4, 0.4), randf(-0.5, 0.5)])
		randpos = np.array([randf(-0.5, 0.9), randf(-0.7, 0.7), randf(-0.5, 0.5)])
		arm_root = np.array(p.getBasePositionAndOrientation(self.body.bodyid)[0])
		
		# ensure the target is reachable
		relpos = randpos - arm_root
		# if np.linalg.norm(relpos) > self.armlength:
		# 	relpos /= np.linalg.norm(relpos) * self.armlength

		self.targetpos = arm_root + relpos

		p.resetBasePositionAndOrientation(self.target, self.targetpos, [0,0,0,1])


	def _reset(self):
		self._reset_target()
		self.body.reset()

		rnd_vel = (np.random.rand(6) * 10) - 5
		self.body.setVelocities(rnd_vel)
		for i in range(50):
			p.stepSimulation()

		pos, _ = self.body.getJointStates()
		self.body.reset(pos)

		ob = self._get_obs()
		self.current_obs = ob
		return ob
		

	def _open_visualizer(self):
		import pyglet
		self.window = pyglet.window.Window(self.render_width, self.render_height)

	def _render(self, mode='human', close=False):
		# rendering is done by bulletphysics if self.mode == p.GUI
		if self.current_obs is None:
			self._get_obs()
		obs = self.current_obs
		if self.obs_type == 'q':
			print('{:35s} | {:35s} | {:14s}\n{: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} | {: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} {: 4.2f} | {: 4.2f} {: 4.2f} {: 4.2f}'.format(
				'Position', 'Velocity', 'Distance', obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8], obs[9], obs[10], obs[11], obs[12], obs[13], obs[14]))
		else:
			if not self.plt_initialized:
				plt.ion()
				self.plt_im = plt.imshow(np.zeros((self.image_height, self.image_width, 3)))
				self.plt_initialized = True
			
			self.plt_im.set_data(obs[:,:,:3])
			plt.draw()
			plt.pause(0.001)

			if self.plt_initialized and close:
				plt.ioff()
				self.plt_initialized = False

	@property
	def action_space(self):
		control_rng = self.body.getControlRange()
		return spaces.Box(np.array(control_rng[0]), np.array(control_rng[1]))

	@property
	def observation_space(self):
		if self.obs_type == 'q':
			nvals = self._get_obs().size
			high = 2 * np.pi * np.ones((nvals,))
			low = -high
			return spaces.Box(low, high)
		else:
			channels = 3
			if self.obs_type == 'rgbd':
				channels = 4
			elif self.obs_type == 'birgb':
				channels = 6
			high = np.ones((self.image_height, self.image_width, channels))
			low  = -np.ones((self.image_height, self.image_width, channels))
			return spaces.Box(low, high)

	
