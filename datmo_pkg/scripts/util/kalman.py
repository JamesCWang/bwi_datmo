import numpy as np

# https://en.wikipedia.org/wiki/Kalman_filter
class KalmanFilter:
	def __init__(self, t, x, y):
		self.t0 = t
		self.X0 = np.array([x, y, 0, 0]) # initial state is x, y, v_x, v_y
		self.P0 = np.identity(4) * 1  # TODO initial covariance (think it's just identiy?)
										# Actually 0 according to wikipedia if know perfectly? but then again we dont
										# feel like it should be measure covariance
		self.P0[2][2] = self.P0[2][2]*10
		self.P0[3][3] = self.P0[3][3]*10

		self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Measurement matrix
		self.Q = np.identity(4) * .01  # TODO TUNE process error covariance(stuff like wind) should make velocity more?
		self.Q[2][2] = self.Q[2][2]*10
		self.Q[3][3] = self.Q[3][3]*10
		self.R = np.identity(2) # TODO TUNE measurement covariance make it angular?


		# add w and v as just random numbers each time?

	# Set X1 and P1 to our estimate of X0 and P0 at t
	def predict(self, t):
		dt = t - self.t0
		F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
		X1 = np.matmul(F, self.X0)
		P1 = np.matmul(np.matmul(F, self.P0), np.transpose(F)) + self.Q
		return X1, P1

	# Update X0 and P0 based on new observation
	def update(self, t, x, y):
		dt = t - self.t0
		self.t0 = t
		F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
		X1 = np.matmul(F, self.X0)
		P1 = np.matmul(np.matmul(F, self.P0), np.transpose(F)) + self.Q

		K = np.matmul(np.matmul(P1, np.transpose(self.H)), np.linalg.inv(np.matmul(self.H, np.matmul(P1, np.transpose(self.H))) + self.R))
		Y = np.array([x, y]) - np.matmul(self.H, X1)

		self.X0 = X1 + np.matmul(K, Y)
		self.P0 = np.matmul(np.identity(4)-np.matmul(K, self.H), P1)
