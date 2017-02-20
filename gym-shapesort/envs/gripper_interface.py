import sys
sys.path.append('/home/max/bullet3/build_cmake/examples/pybullet')

from collections import OrderedDict
import pybullet as p
import math

class BodyJointInterface(object):
	def __init__(self, bodyid, active_motors=False):
		self.bodyid = bodyid
		self.joints, self.joint_types, self.zero_state = self._getJointInfos()
		self.bodyid = bodyid

		if not active_motors:
			for _, jid in self.joints.items():
				self.setJointVelocity(jid, 0, maxForce=0)

	def _getJointInfos(self):
		info = OrderedDict()
		meta = OrderedDict()
		zero = OrderedDict()
		for i in range(p.getNumJoints(self.bodyid)):
			jid, jname, jtype, jq0, jv0, jflags, jdamp, jfrict = p.getJointInfo(self.bodyid, i)
			if jtype != p.JOINT_FIXED:
				jpos, jvel, jforces, jtorque = p.getJointState(self.bodyid, jid)
				zero[jname] = (jpos, jvel)

			info[jname] = jid
			meta[jname] = jtype
		return info, meta, zero

	def getControlRange(self):
		raise NotImplementedError

	def reset(self):
		# reset velocities
		for name, jid in self.joints.items():
			_, jvel = self.zero_state[name]
			self.setJointVelocity(jid, jvel)

		# reset joint states
		for name, jid in self.joints.items():
			jpos, _ = self.zero_state[name]
			p.resetJointState(self.bodyid, jid, jpos)

	def getJointStates(self):
		state = [0]*len(self.joints)
		for i, (name, jid) in enumerate(self.joints.items()):
			jpos, jvel, jforces, jtorque = p.getJointState(self.bodyid, jid)
			state[i] = jpos
		return state

	def getLinkStates(self):
		worldpos = [0] * (len(self.joints)-1)
		worldorn = [0] * (len(self.joints)-1)
		for i in range(len(self.joints)-1):
			pos, orn, localinertialpos, localinertialorn, framepos, frameorn = p.getLinkState(self.bodyid, i)
			worldpos[i] = pos
			worldorn[i] = orn

		return worldpos, worldorn

	def setJointVelocity(self, jid, velocity, maxForce=5.0):
		if maxForce is None:
			p.setJointMotorControl2(
			bodyIndex=self.bodyid, 
			jointIndex=jid, 
			controlMode=p.VELOCITY_CONTROL,
			targetVelocity=velocity)
		else:
			p.setJointMotorControl2(
				bodyIndex=self.bodyid, 
				jointIndex=jid, 
				controlMode=p.VELOCITY_CONTROL,
				targetVelocity=velocity,
				force=maxForce)


class GripperInterface(BodyJointInterface):
	def __init__(self, bodyid, active_motors=True):
		super(GripperInterface, self).__init__(bodyid, active_motors)

		self.pincher_joints = [
			self.joints['joint_pincher_top_left'], 
			self.joints['joint_pincher_top_right']]

		for name, jtype in self.joint_types.items():
			if jtype == p.JOINT_FIXED:
				del self.joints[name]

	def getControlRange(self):
		return [-2.0]*(len(self.joints)-1), [2.0]*(len(self.joints)-1)

	def setVelocities(self, velocities):
		assert(velocities.size == len(self.joints) - 1)
		# assert(len(velocities) == len(self.joints)-1)

		for i in range(len(self.joints) - 1):
			self.setJointVelocity(i, velocities[i])
		self.setPincherVelocity(velocities[-1])

	def getJointStates(self):
		state = super(GripperInterface, self).getJointStates()
		# collapse pinch states to one
		collapsed_state = []
		for i in range(len(state)-1):
			collapsed_state.append(state[i])

		return collapsed_state

	def setWristVelocity(self, velocity):
		pwrist = self.joints['joint_wrist_x']
		self.setJointVelocity(pwrist, velocity)

	def setPincherVelocity(self, velocity):
		pleft = self.joints['joint_pincher_bottom_left']
		pright = self.joints['joint_pincher_bottom_right']
		self.setJointVelocity(pright, velocity)
		self.setJointVelocity(pleft, -velocity)

	def getPincherCentroid(self):
		left  = p.getLinkState(self.bodyid, self.pincher_joints[0])[0]
		right = p.getLinkState(self.bodyid, self.pincher_joints[1])[0]

		return [(left[0]+right[0])/2, (left[1]+right[1])/2, (left[2]+right[2])/2]

	def getPincherContact(self):
		state = [0, 0]

		contacts = p.getContactPoints(self.bodyid)

		for contact in contacts:
			# the fourth/third element in contact defines the link index.
			contact_link = contact[3]
			sign = 1
			if contact[1] != self.bodyid:
				sign = -1
				contact_link = contact[4]

			# the last element in contact defines the normal force
			# we set the force to negative to indicate external impact
			if contact_link == self.pincher_joints[0]:
				if contact[-1] > abs(state[0]):
					state[0] = sign * contact[-1]
			elif contact_link == self.pincher_joints[1]:
				if contact[-1] > abs(state[1]):
					state[1] = sign * contact[-1]

		return state
