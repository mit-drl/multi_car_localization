import numpy as np
from numpy.random import uniform
from numpy.random import randn
# For random samples from N(\mu, \sigma^2), use:
# sigma * np.random.randn(...) + mu
from numpy.linalg import norm
from numpy.random import multivariate_normal
import pdb

def relative_dubins(T, u):
	u1 = u[:2]
	numT = np.shape(T)[0]/3
	dT = np.asmatrix(np.zeros(np.shape(T)))
	for i in range(numT):
		fi = 3*i 
		uj = u[2*(i+1):2*(i+1)+2]
		dT[fi]   = uj.item(0)*np.cos(T[fi+2].item()) + T[fi+1].item()*u1.item(1) - u1.item(0)
		dT[fi+1] = uj.item(0)*np.sin(T[fi+2].item()) - T[fi].item()*u1.item(1)
		dT[fi+2] = uj.item(1) - u1.item(1)

	return dT

def rk4(y0, u, dt, f):
	steps = 3
	h = dt/steps

	yn = y0

	for n in range(steps):
		k1 = relative_dubins(yn, u)
		k2 = relative_dubins(yn + h*k1/2, u)
		k3 = relative_dubins(yn + h*k2/2, u)
		k4 = relative_dubins(yn + h*k3, u)

		yn = yn + (h/6)*(k1+2*k2+2*k3+k4)

	return yn

class ParticleFilter(object):

	def __init__(self):
		self.StateTransitionFcn = None
		self.MeasurementLikelihoodFcn = None


	def rot(self, theta):
		return np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

	# bounds is d x 2 where d is dimensions of a particle
	def create_uniform_particles(self, N, bounds, circ_var):
		dim = np.shape(bounds)[0]

		particles = np.asmatrix(np.empty((N, dim)))

		for i in np.arange(dim):
			particles[:, i] = np.asmatrix(uniform(bounds[i, 0], bounds[i, 1], size=N)).T
			if circ_var[i] == 1:
				particles[:, i] %= 2 * np.pi 

		return particles

	def pfStateTransition(self, prevParticles, dt, u):
		predictParticles = np.asmatrix(np.zeros(np.shape(prevParticles)))

		N = np.shape(prevParticles)[0]

		r = np.asmatrix(multivariate_normal((0,0,0,0,0,0),np.diag([0.1, 0.05, 0.1, 0.05, 0.1, 0.05]),N))

		new_u = r + u.T

		for i in np.arange(np.shape(prevParticles)[0]):
			state = prevParticles[i].T
			u = new_u[i].T
			newParticle = rk4(state, u, dt, relative_dubins)
			predictParticles[i,:] = newParticle.T

		return predictParticles

if __name__ == "__main__":
	pf = ParticleFilter()

	Ncars = 3

	initial_positions = np.matrix([[0, 0, np.pi/2], [1, 1, -np.pi/2], [-1, -2, 0]])
	initial_transforms = np.asmatrix(np.empty(((Ncars-1), 3)))
	circ_var = [0, 0, 1, 0, 0, 1]
	limits = np.matrix([0.5, 0.5, np.pi/2, 0.5, 0.5, np.pi/2]).T
	bounds = np.asmatrix(np.empty((3*(Ncars-1), 2)))

	for i in range(Ncars-1):
		init_xy = pf.rot(-initial_positions[0,2])*initial_positions[i+1,:2].T
		init_theta = initial_positions[i+1, 2] - initial_positions[0, 2]
		initial_transforms[i,:] = np.append(init_xy.T, [[init_theta]], axis=1)

		fi = 3*i
		bounds[fi:fi+3,0] = initial_transforms[i,:].T - limits[fi:fi+3]
		bounds[fi:fi+3,1] = initial_transforms[i,:].T + limits[fi:fi+3]

	particles = pf.create_uniform_particles(10, bounds, circ_var)
	np = pf.pfStateTransition(particles, 0.05, np.matrix([1, 1, 1, 1, 1, 1]).T)

	pdb.set_trace()