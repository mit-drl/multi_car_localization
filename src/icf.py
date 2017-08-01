# information-weighted consensus filter
# see Information Weighted Consensus Filter and
# their Application in Distributed Camera Networks
# by A.T. Kamal, J.A. Farrell and A.K. Roy-Chowdhury
# for more info

import numpy as np





xi_prior
Ji_prior
Hi = np.matrix([[],[]])
# consensus speed factor
epsilon
# number of consensus iterations
K
# number of communication nodes/vertices in the graph
N
# a list of all neighbors
neighbors

# STEP 1:
# get measurement zi and measurement information matrix Bi
zi
Bi

# STEP 2:
# compute consensus proposals
Vi = Ji_prior/N + Hi.T*Bi*Hi
vi = Ji_prior*xi_prior/N + Hi.T*Bi*zi

# STEP 3:
# perform average consensus on Vi and vi independently
Vi = consensus(Vi, neighbors, K)
vi = consensus(vi, neighbors, K)

# STEP 4:
# computer a posteriori state estimate and information matrix for time t
xi_post = np.inv(Vi)*vi
Ji_post = N*Vi

# STEP 5:
Ji_prior = # inverse of the predicted next covariance matrix
xi_prior = # forward simulation using particle filter



def consensus(v, neighbors K):
	for i in range(0, K):
	for j, neighbor in enumerate(neighbors):
		send(v, neighbor)
		v_neighbors[j] = receive()
	v = v + epsilon*np.sum(v_neighbors - v)

	return v