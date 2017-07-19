Np = 100
Ncars = 3
Ndim = 2
Nmeas = 5
dt = 0.1
x0 = np.array([[0, 0],
               [2, 1],
               [0, 1]])
x_cov = np.diag(Ncars * Ndim * [0.1])
measurement_cov = np.diag(Ncars * [0.1, 0.1, 0.01, 0.01, 0.01])
v_cov = np.diag(Ncars * Ndim * [0.1])
particles = np.random.multivariate_normal(x0.flatten(), x_cov, size=Np)
particles = particles.reshape((Np, Ncars, Ndim))
weights = 1.0 / Np * np.ones((Np,))
v_func = lambda t: np.array([
    [0.1 * t, 3 * np.cos(t)],
    [2 * np.cos(0.5 * t), 1 * np.sin(t)],
    [1.4 * np.cos(t), 3 * np.sin(0.5 * t)]])
Nsecs = 100.0
Nsteps = int(Nsecs / dt)
xs = np.zeros((Nsteps + 1, Ncars, Ndim))
xs_pred = np.zeros_like(xs)
measurements = np.zeros((Nsteps, Ncars, Nmeas))
xs[0] = x0
xs_pred[0] = x0
resample_perc = 0.3
error = np.zeros((Nsteps,))
for i in tnrange(1, Nsteps + 1):
    vs = v_func(i * dt)
    xs[i] = xs[i - 1] + vs * dt
    means = np.zeros((Ncars, Nmeas))
    for j in xrange(Ncars):
        means[j, :2] = xs[i, j, :2]
        for k in xrange(Ncars):
            if j != k:
                means[j, k + 2] = np.linalg.norm(xs[i, j] - xs[i, k])
    measurements[i - 1] = np.random.multivariate_normal(
        means.flatten(),measurement_cov).reshape(Ncars, Nmeas)
    control_probs = np.zeros((Np,))
    for j in xrange(Np):
        vel = np.random.multivariate_normal(vs.flatten(), v_cov)
        new_particle = particles[j] + vel.reshape(3, 2) * dt
        control_probs[j] = multivariate_normal.pdf(
            new_particle.flatten(),
            mean=(particles[j] + vs).flatten(),
            cov=v_cov)
        particles[j] = new_particle
        #particles[j] = particles[j] + vs * dt
    for j in xrange(Np):
        p_means = np.zeros((Ncars, Nmeas))
        for k in xrange(Ncars):
            p_means[k, :2] = particles[j, k, :2]
            for l in xrange(Ncars):
                if k != l:
                    p_means[k, l + 2] = np.linalg.norm(
                        particles[j, k] - particles[j, l])
        prob = multivariate_normal.pdf(measurements[i - 1].flatten(),
                                       mean=p_means.flatten(), cov=measurement_cov)
        weights[j] = prob * control_probs[j]
        #weights[j] = prob
        #weights[j] *= control_probs[j]
    weights += 1e-32
    weights = weights / weights.sum()
    for j in xrange(Np):
        xs_pred[i] += (weights[j] / weights.sum())  * particles[j]

    n_eff = 1.0 / (weights ** 2).sum()
    if n_eff < resample_perc * Np:
        distr = rv_discrete(values=(np.arange(Np), weights))
        particles = particles[distr.rvs(size=Np)]
        weights = 1.0 / Np * np.ones_like(weights)

    for j in xrange(Ncars):
        error[i - 1] += np.linalg.norm(xs_pred[i, j] - xs[i, j]) / Ncars
