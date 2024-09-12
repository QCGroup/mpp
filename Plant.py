import numpy as np

class Plant():
    g = 9.81
    rhoNom = 1.225

    #Init function that coppies over the config file into the class variables.
    def __init__(self, config):
        self.dt = config.dt 
        self.m = config.m
        self.Iyy = config.Iyy
        self.S = config.S
        self.Cl_0 = config.Cl_0
        self.Cl_alpha = config.Cl_alpha
        self.K = config.K
        self.Cm_0 = config.Cm_0
        self.Cd_0 = config.Cd_0
        self.Cm_alpha = config.Cm_alpha
        self.Cm_alpha_dot = config.Cm_alpha_dot
        self.Cm_delta_e = config.Cm_delta_e
        self.rhoSTD = config.rhoSTD
        self.Restimation = config.Restimation

        self.x_thres = config.x_thres
        self.z_thresHigh = config.z_thresHigh
        self.z_thresLow = config.z_thresLow
        self.v_thres = config.v_thres
        self.theta_thres = config.theta_thres
        self.theta_dot_thres = config.theta_dot_thres
        self.gamma_thres = config.gamma_thres

        self.state = None
        self.prev_action = None
        self.steps_beyond_terminated = None
        self.time = 0.
        self.objects = None

    #Helper function to calculate the derivative of the system at a given state
    def calculateStateDerivatives(self, state, control, rho):
        x, z, v, theta, theta_dot, gamma = state
        q = 0.5*rho*v**2
        alpha = theta - gamma
        (thrust, delta_e) = control

        #alpha dot
        Cl = self.Cl_0 + self.Cl_alpha*alpha
        L = q*self.S*Cl
        gamma_dot = (L - self.m*self.g*np.cos(gamma) +thrust*np.sin(alpha)) / (self.m*v)
        alpha_dot = theta_dot-gamma_dot

        # Other forces
        Cd = self.Cd_0 + self.K*Cl**2
        Cm = self.Cm_0 + self.Cm_alpha*alpha + self.Cm_alpha_dot*alpha_dot + self.Cm_delta_e*delta_e

        D = q*self.S*Cd
        M = q*self.S*Cm

        #derivatives
        x_dot = v*np.cos(gamma)
        z_dot = v*np.sin(gamma)
        v_dot = (-D - self.m*self.g*np.sin(gamma) + thrust*np.cos(alpha)) / self.m
        theta_dot = theta_dot
        theta_ddot = M / self.Iyy
        return np.array([[x_dot], [z_dot], [v_dot], [theta_dot], [theta_ddot], [gamma_dot]])

    #Main iteration function.  Steps the physics and time dt foward.
    def step(self, action):
        #Error checking
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.state is not None, "Call reset before using step method."

        #Extract states
        x, z, v, theta, theta_dot, gamma = self.state
        rho = np.random.normal(self.rhoNom,self.rhoSTD)

        #Calculate derivatives and integrate
        x_dot, z_dot, v_dot, theta_dot, theta_ddot, gamma_dot = self.calculateStateDerivatives(self.state, action,rho)
        x += x_dot*self.dt
        z += z_dot*self.dt
        v += v_dot*self.dt 
        theta += theta_dot*self.dt
        theta_dot += theta_ddot*self.dt
        gamma += gamma_dot*self.dt
        state = np.array([x,z,v,theta,theta_dot,gamma])
        self.state = (list(np.reshape(state,(6,))))
        self.time += self.dt

        #Observation model
        observation = np.array([v,theta]) + np.array([np.random.multivariate_normal([0,0], self.Restimation)]).T
        #Check if the system is violating a limit
        terminated = bool(x < -self.x_thres
            or x > self.x_thres
            or z > self.z_thresHigh
            or z < self.z_thresLow
            or v < -self.v_thres
            or v > self.v_thres
            or theta < -self.theta_thres
            or theta > self.theta_thres
            or theta_dot < -self.theta_dot_thres
            or theta_dot > self.theta_dot_thres
            or gamma < -self.gamma_thres
            or gamma > self.gamma_thres)

        #Returns
        if not terminated:
            reward = 1.0
        elif self.steps_beyond_terminated is None:
            self.steps_beyond_terminated = 0
            reward = 0.0
            print(x < -self.x_thres
            ,z > self.z_thresHigh
            , z < self.z_thresLow
            ,v < -self.v_thres
            ,v > self.v_thres
            ,theta < -self.theta_thres
            ,theta > self.theta_thres
            ,theta_dot < -self.theta_dot_thres
            ,theta_dot > self.theta_dot_thres
            ,gamma < -self.gamma_thres
            ,gamma > self.gamma_thres)
        else:
            if self.steps_beyond_terminated == 0:
                print(
                    "You are calling 'step()' even though this environment has already returned terminated = True.")
            self.steps_beyond_terminated += 1
            reward = 0.0

        return observation, reward, terminated

    #Resets the model to the entered initial state.
    def reset(self, initialState):
        self.state = initialState
        self.steps_beyond_terminated = None
        self.time = 0
        return self.state

    def close(self):
        pass