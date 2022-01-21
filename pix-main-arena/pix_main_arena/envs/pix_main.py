import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import random
from os.path import normpath, basename

class PixArena(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		"""Constructor.

		Function to initialize and load the Arena
		List of Functions:
			
			move_husky
			reset
			camera_feed
			remove_car
			respawn_car

		No Arguments
		"""

		p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setGravity(0,0,-15)
		p.loadURDF('rsc/plane.urdf',[0,0,-0.1], useFixedBase=1)
		p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
		p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
		

		self.husky = None
		self.arena_size = 12
		self.cover_plates = [None] * (self.arena_size ** 2)

		self.__load_arena()
		self.respawn_car()

		self._width = 720
		self._height = 720

	def move_husky(self, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel):
		"""
		Function to give Velocities to the wheels of the robot.
			
		Arguments:

			leftFrontWheel - Velocity of the front left wheel  
			rightFrontWheel - Velocity of the front right wheel  
			leftRearWheel - Velocity of the rear left wheel  
			rightRearWheel - Velocity of the rear right wheel  


		Return Values:

			None
		"""

		self.__move(self.husky, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel)

	def reset(self):
		"""
		Function to restart the simulation.

		This will undo all the previous simulation commands and the \
		arena along with the robot will be loaded again.
		
		Only for testing purposes. Won't be used in final evaluation.

		Arguments:

			None

		Return Values:

			None
		"""
		np.random.seed(0)
		p.resetSimulation()
		p.setGravity(0,0,-10)

		p.loadURDF('rsc/plane.urdf',[0,0,-0.1], useFixedBase=1)
		p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
		p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
		
		self.__load_arena()
		self.respawn_car()

	def __load_arena(self):
		"""
		Function to load the arena
		"""
		
		# self.arena = np.random.randint(low = 0, high = 6, size=(9, 9))
		# Arena reference
		# 0 -> black base
		# 1 -> light green base
		# 2 -> blue base
		# 3 -> red base
		# 4 -> purple base
		# 5 -> yellow base
		# 6 -> white base
		# 7 -> dark green base
		self.arena = np.array([
			[0, 1, 4, 1, 0, 1, 0, 0, 0, 1, 1, 1],
			[6, 6, 6, 6, 6, 5, 5, 1, 3, 5, 5, 5],
			[6, 0, 0, 3, 0, 5, 0, 2, 0, 0, 3, 0],
			[6, 0, 5, 5, 6, 6, 0, 1, 0, 1, 1, 1],
			[6, 0, 6, 1, 0, 0, 1, 6, 6, 6, 0, 0],
			[6, 1, 6, 1, 0, 1, 5, 0, 1, 6, 0, 4],
			[3, 0, 5, 2, 0, 1, 1, 5, 6, 6, 1, 3],
			[3, 0, 0, 5, 6, 6, 1, 5, 0, 5, 0, 3],
			[5, 1, 6, 1, 1, 0, 5, 0, 0, 5, 1, 1],
			[0, 6, 1, 0, 0, 1, 1, 3, 3, 0, 0, 5],
			[1, 6, 6, 1, 0, 3, 0, 0, 5, 1, 0, 3],
			[0, 6, 0, 1, 0, 3, 0, 5, 1, 1, 1, 7],	
		])
		# Shape Matrix
		# 1: Square
		# 2: Circle
		# 3: Triangle Pointing Up
		# 4: Triangle Pointing Right
		# 5: Triangle Pointing Left
		# 6: Triangle Pointing Down
		self.shapes = np.array([
			[0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],	
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1],
			[6, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
		])
		# self.arena = np.rot90(self.arena, 3)
		# self.shapes = np.rot90(self.shapes, 3)
		# print(self.arena)
		base_plate_dict = {
			1: 'rsc/base plate/base plate green.urdf',
			2: 'rsc/base plate/base plate cyan.urdf',
			3: 'rsc/base plate/base plate red.urdf',
			4: 'rsc/base plate/base plate purple.urdf',
			5: 'rsc/base plate/base plate yellow.urdf',
			6: 'rsc/base plate/base plate white.urdf',
			7: 'rsc/base plate/base plate darkgreen.urdf',
			# 1: 'rsc/base plate/base plate blue.urdf',
		}
		shape_colour_dict = {
			# 0: 'rsc/square/square yellow.urdf',
			# 1: 'rsc/circle/circle yellow.urdf',
			# 2: 'rsc/triangle/triangle yellow.urdf',
			# 3: 'rsc/square/square red.urdf',
			# 4: 'rsc/circle/circle red.urdf',
			# 5: 'rsc/triangle/triangle red.urdf',
			1: 'rsc/circle/circle blue.urdf',
			2: 'rsc/square/square blue.urdf',
			3: 'rsc/triangle/triangle blue.urdf',
			4: 'rsc/triangle/triangle blue.urdf',
			5: 'rsc/triangle/triangle blue.urdf',
			6: 'rsc/triangle/triangle blue.urdf',
		}
		self.shape_color = shape_colour_dict
		def get_postion(i, j):
			if self.shapes[i, j] == 3:
				return [5.4-i*1,5.52-j*1,0.02]
			elif self.shapes[i, j] == 4:
				return [5.48-i*1,5.56-j*1,0.02]
			elif self.shapes[i, j] == 5:
				return [5.48-i*1,5.4-j*1,0.02]
			elif self.shapes[i, j] == 6:
				return [5.56-i*1,5.48-j*1,0.02]
			return [5.5-i*1,5.5-j*1,0.02]
		
		def get_triangle_orientation(x):
			if x == 3:
				return p.getQuaternionFromEuler([0, 0, 0])
			elif x == 4:
				return p.getQuaternionFromEuler([0, 0, -np.pi/2])
			elif x == 5:
				return p.getQuaternionFromEuler([0, 0, np.pi/2])
			elif x == 6:
				return p.getQuaternionFromEuler([0, 0, np.pi])
		
		def get_base_plate_position(i, j):
			return [5.5-i*1,5.5-j*1,0]
		
		def get_cover_plate_position(i, j):
			return [5.5-i*1,5.5-j*1,0.03]

		# Insert list of cover plate locations
		cover_plate_locs = [(0, 2), (5, 11)]

		for i in range(self.arena_size):
			for j in range(self.arena_size):
				if self.arena[i, j] == 0:
					continue
				p.loadURDF(base_plate_dict[self.arena[i, j]], get_base_plate_position(i, j), p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=1)
				if self.shapes[i, j] == 0:
					continue
				if self.shapes[i, j] in [3, 4, 5, 6]:
					p.loadURDF(shape_colour_dict[self.shapes[i, j]], get_postion(i, j), get_triangle_orientation(self.shapes[i, j]), useFixedBase=1)
					continue
				p.loadURDF(shape_colour_dict[self.shapes[i, j]], get_postion(i, j), p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
		for (i, j) in cover_plate_locs:
			self.cover_plates[i * self.arena_size + j] = p.loadURDF(base_plate_dict[self.arena[i, j]], get_cover_plate_position(i, j), p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=1)

	def remove_cover_plate(self, i, j):
		"""
		Function to remove the cover plate at (i, j)

		Arguments:

			i: Abscissa of the plate to be removed
			j: Ordinate of the plate to be removed
		
		Return Values:

			True if there was a cover plate at that location, otherwise False
		"""
		if(self.cover_plates[i * self.arena_size + j] is None):
			return False
		else:
			p.removeBody(self.cover_plates[i * self.arena_size + j])
			self.cover_plates[i * self.arena_size + j] = None
			return True

	def __move(self, car, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel):
		p.setJointMotorControl2(car,  4, p.VELOCITY_CONTROL, targetVelocity=leftFrontWheel, force=30)
		p.setJointMotorControl2(car,  5, p.VELOCITY_CONTROL, targetVelocity=rightFrontWheel, force=30)
		p.setJointMotorControl2(car,  6, p.VELOCITY_CONTROL, targetVelocity=leftRearWheel, force=30)
		p.setJointMotorControl2(car,  7, p.VELOCITY_CONTROL, targetVelocity=rightRearWheel, force=30)

	def camera_feed(self, is_flat = False):
		"""
		Function to get camera feed of the arena.

		Arguments:

			None
		
		Return Values:

			numpy array of RGB values
		"""
		look = [0, 0, 0.2]
		cameraeyepos = [0, 0, 6.5]
		cameraup = [0, -1, 0]
		self._view_matrix = p.computeViewMatrix(cameraeyepos, look, cameraup)
		fov = 92
		aspect = self._width / self._height
		near = 0.8
		far = 10
		self._proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
		img_arr = p.getCameraImage(width=self._width,
								height=self._height,
								viewMatrix=self._view_matrix,
								projectionMatrix=self._proj_matrix,
								renderer=p.ER_BULLET_HARDWARE_OPENGL)
		rgb = img_arr[2]
		if is_flat == True:
			# Only for those who are getting a blank image in opencv
			rgb = np.array(rgb)
			rgb = np.reshape(rgb, (self._width, self._height, 4))
		rgb = np.uint8(rgb)
		rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
		rgb = np.rot90(rgb, 3)
		return rgb

	def remove_car(self):
		"""
		Function to remove the car from the arena.

		Arguments:

			None

		Return Values:

			None
		"""
		p.removeBody(self.husky)
		self.husky = None

	def respawn_car(self):
		"""
		Function to respawn the car from the arena.

		Arguments:

			None

		Return Values:

			None
		"""
		if self.husky is not None:
			print("Old Car being Removed")
			p.removeBody(self.husky)
			self.husky = None

		pos = [[11, 11]]
		ori = [np.pi/2, 0, np.pi/2, np.pi]
		x = np.random.randint(0, len(pos))
		self.husky = p.loadURDF('rsc/car/car.urdf', [5.5-1*pos[x][0],5.5-1*pos[x][1],0], p.getQuaternionFromEuler([0,0,ori[x]]))
		#self.husky = p.loadURDF('husky/husky.urdf', [4-1*pos[x][0],4-1*pos[x][1],0], p.getQuaternionFromEuler([0,0,ori[x]]))
		#self.aruco = p.loadURDF('rsc/aruco/aruco.urdf', [4-1*pos[x][0],4-1*pos[x][1],1.2], p.getQuaternionFromEuler([1.5707,0,ori[x]]))
		#p.createConstraint(self.husky, -1, self.aruco, -1, p.JOINT_FIXED, [0,0,1], [0,0,0.4], [0,0,0], childFrameOrientation = p.getQuaternionFromEuler([0,0,1]))
		for x in range(100):
			p.stepSimulation()
