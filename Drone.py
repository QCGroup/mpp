import numpy as np
import scipy as sci
import Plant
import control as ctrl
import gymnasium as gym

class Drone():
    g = 9.81
    rhoNom = 1.225

    #Init function that coppies over the config file into the class variables.
    def __init__(self, config, configPlant):
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
        self.Pestimation = config.Pestimation
        self.Qestimation = config.Qestimation

        self.state = None
        self.prev_action = None
        self.steps_beyond_terminated = None
        self.time = 0.
        self.objects = None

        self.plant = Plant.Plant(configPlant)
        self.numObstacles = config.numObstacles

        self.Qcontrol = config.Qcontrol
        self.Rcontrol = config.Rcontrol

        self.lidar_range = config.lidar_range
        self.lidar_angle = config.lidar_angle
        self.lidar_res = config.lidar_res

    #Calculates nominal veloticty and flight path angle given elevator and thrust command
    def vGammaFromElevator(self, T, delta_e):
        alphaNom = -(self.Cm_0+self.Cm_delta_e*delta_e)/self.Cm_alpha
        Clnom = self.Cl_0 + self.Cl_alpha*alphaNom
        Cdnom = self.Cd_0 + self.K*Clnom**2
        gammaNom = Cdnom/Clnom*(1-T/self.m/self.g*np.sin(alphaNom)) + T*np.cos(alphaNom)/self.m/self.g
        vNom = np.sqrt(2*(-T*np.sin(alphaNom)+self.m*self.g*np.cos(gammaNom))/(self.S*self.rhoNom))
        return vNom, gammaNom

    #Given a thrust and pitch, finds the necesary elevator command 
    def solveForElevator(self, T, thetaDes):
        #Calculates pitch angle from 
        def thetaFromElevator(self, T,thetaDes, delta_e):
            alphaNom = -(self.Cm_0+self.Cm_delta_e*delta_e)/self.Cm_alpha
            Clnom = self.Cl_0 + self.Cl_alpha*alphaNom
            Cdnom = self.Cd_0 + self.K*Clnom**2
            return thetaDes- (-Cdnom/Clnom + T/(self.m*self.g)*(1-Cdnom/Clnom*alphaNom))-alphaNom
        func = lambda x: thetaFromElevator(T, thetaDes,x)
        sol = sci.optimize.root_scalar(func, method='secant', x0 = 0, x1 = 1)
        return sol.root

    #Calculates the elevator command given a desired angle of attack
    def elevatorFromAlpha(self, alpha):
        return -(self.Cm_alpha*alpha+self.Cm_0)/self.Cm_delta_e 

    # Calculates a "coherent" (steady state stable) command from thrust and gamma command
    # Returns airspeed, theta, thetadot, and flight path angle (gamma)
    def coherentCommand(self, T, gamma):
        airspeedGuess = 15
        thetaGuess = 0
        thetaDot = 0
        verticalForces = lambda X: ((self.Cl_0 + self.Cl_alpha*(X[0]- gamma))*1/2*(X[1]**2)*self.rhoNom*self.S - self.m*self.g*np.cos(gamma) + T*np.sin(X[0]-gamma), \
               ((self.Cd_0+self.K*(self.Cl_0 + self.Cl_alpha*(X[0]- gamma))**2)*1/2*(X[1]**2)*self.rhoNom*self.S -T*np.cos(X[0]-gamma)-self.m*self.g*np.sin(gamma)))

        sol = sci.optimize.root(verticalForces, x0 = [thetaGuess, airspeedGuess])
        X =sol.x      
        return [sol.x[1], sol.x[0], thetaDot, gamma]

    # Call this function to update the estimated state of the system
    def updateEKF(self, controlIn, observation):
        v, theta, theta_dot, gamma = self.stateEstimate
        alpha = theta-gamma
        q = 0.5*self.rhoNom*v**2

        #Calculate alphaDot
        Cl = self.Cl_0 + self.Cl_alpha*alpha
        L = q*self.S*Cl
        gamma_dot = (L - self.m*self.g*np.cos(gamma) +controlIn[0]*np.sin(alpha)) / (self.m*v)
        alpha_dot = theta_dot-gamma_dot

        #Other aero
        Cd = self.Cd_0 + self.K*Cl**2
        Cm = self.Cm_0 + self.Cm_alpha*alpha + self.Cm_alpha_dot*alpha_dot + self.Cm_delta_e*controlIn[1]
        D = q*self.S*Cd
        M = q*self.S*Cm

        #derivatives
        v_dot = (-D - self.m*self.g*np.sin(gamma) + controlIn[0]*np.cos(alpha)) / self.m
        #theta_dot = theta_dot
        theta_ddot = M / self.Iyy
        gamma_dot = (L - self.m*self.g*np.cos(gamma) +controlIn[0]*np.sin(alpha)) / (self.m*v)

        AFull, BFull = self.calculateCTSABMatrix([v, theta, theta_dot, gamma], controlIn)
        F = AFull*self.dt + np.eye(4) 

        #Observability 
        H = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0]])

        #predict
        v += v_dot*self.dt
        theta += theta_dot*self.dt 
        theta_dot += theta_ddot * self.dt
        gamma += gamma_dot*self.dt
        xEst = np.array([[v], [theta], [theta_dot], [gamma]])
        
        self.Pestimation = (F)@self.Pestimation @ np.transpose(F) + self.Qestimation
        #update
        y = np.array([[v],[theta]]) - np.array([observation[0],observation[1]])
        S = H @ self.Pestimation @ H.T + self.Restimation
        K = self.Pestimation @ np.transpose(H) @ sci.linalg.inv(S)
        xEst -= K @ y
        self.Pestimation = (np.eye(4) - K @ H) @ self.Pestimation
        self.stateEstimate = xEst.flatten()

    # Returns the state and control Jacobians
    def calculateCTSABMatrix(self, stateIn, controlIn):
        (thrust, delta_e) = controlIn
        v, theta, theta_dot, gamma = stateIn
        q = 0.5*self.rhoNom*v**2
        alpha = theta-gamma

        #Calculate alphaDot
        Cl = self.Cl_0 + self.Cl_alpha*alpha
        L = q*self.S*Cl
        gamma_dot = (L - self.m*self.g*np.cos(gamma) +thrust*np.sin(alpha)) / (self.m*v)
        alpha_dot = theta_dot-gamma_dot

        #Other aero
        Cd = self.Cd_0 + self.K*Cl**2
        Cm = self.Cm_0 + self.Cm_alpha*alpha + self.Cm_alpha_dot*alpha_dot + self.Cm_delta_e*delta_e
        D = q*self.S*Cd
        M = q*self.S*Cm

        #Partial derivatives of aeroforces
        pDpv = self.rhoNom*v*self.S*Cd
        pDptheta = 2*self.K*q*self.S*(self.Cl_alpha * self.Cl_0 + self.Cl_alpha**2*alpha)
        pDpthetaDot = 0
        pDpgamma = -2*self.K*q*self.S*(self.Cl_alpha* self.Cl_0+self.Cl_alpha**2*alpha)
        pMpv = self.rhoNom*v*self.S*Cm
        pMptheta = q*self.S*self.Cm_alpha
        pMpthetaDot = 0
        pMpgamma = -q*self.S*self.Cm_alpha
        pLpv = self.rhoNom*v*self.S*Cl
        pLptheta = q*self.Cl_alpha*self.S
        pLpthetaDot = 0
        pLpgamma = -q*self.Cl_alpha*self.S

        #out of order entries
        pf4ptheta = (pLptheta + thrust*np.cos(alpha))/(self.m*v)
        pf4pgamma = (pLpgamma/self.m +self.g*np.sin(gamma)-thrust/self.m*np.cos(alpha))/v

        #Partial derivatives of nonlinear map with respect to tstate
        pf1pv = - 1/self.m*pDpv
        pf1ptheta = (-pDptheta-thrust*np.sin(alpha))/self.m
        pf1pthetaDot = 0
        pf1pgamma = (-pDpgamma/self.m - self.g*np.cos(gamma) + thrust/self.m*np.sin(alpha))
        pf2pv = 0
        pf2ptheta = 0
        pf2pthetaDot = 1
        pf2pgamma = 0
        pf3pv = pMpv/self.Iyy
        pf3ptheta = pMptheta/self.Iyy + q*self.S*self.Cm_alpha_dot/self.Iyy*(pf2ptheta-pf4ptheta)
        pf3pthetaDot = 0
        pf3pgamma = pMpgamma/self.Iyy + q*self.S*self.Cm_alpha_dot/self.Iyy*(pf2pgamma-pf4pgamma)
        pf4pv = pLpv/(self.m*v)
        pf4pthetaDot = 0

        AFull = np.array([[pf1pv, pf1ptheta, pf1pthetaDot, pf1pgamma],[pf2pv, pf2ptheta, pf2pthetaDot, pf2pgamma],
                      [pf3pv, pf3ptheta, pf3pthetaDot, pf3pgamma],[pf4pv, pf4ptheta, pf4pthetaDot, pf4pgamma]])
        BFull = np.array([[0], [0], [q*self.Cm_delta_e*self.S/self.Iyy],[0]])
        return AFull, BFull

    # Calculate LQR gains around initial state
    def calculateGains(self, initState):
        x, y, v, theta, theta_dot, gamma = initState
        AFull, BFull =  self.calculateCTSABMatrix(initState[2:], (4, self.elevatorFromAlpha(theta - gamma)))
        self.KFull = ctrl.lqr(AFull, BFull, self.Qcontrol, self.Rcontrol)[0]

    # Calculates ray traces for lidar
    def update_rays(self):

        # filter objects to ones in lidar cone
        def seenObjects(state,angle,beam,objects):

            x,y,theta = state

            # objects within range
            idx = np.logical_and(
                (objects[0,:]>= x),
                (np.sqrt((objects[0,:]-x)**2+(objects[1,:]-y)**2) <= beam)
            )

            # above and below lines
            upper = np.tan(theta+angle/2)
            lower = np.tan(theta-angle/2)
            ind = np.arange(len(idx))
            for i in ind:
                if idx[i]:
                    dx = objects[0,i] - x
                    if y+upper*dx < objects[1,i] or y+lower*dx > objects[1,i]:
                        idx[i] = False

            return objects[:,idx]

        # intersection of line and circle
        def intersect(p1,p2,obj):
            x1 = p1[0] - obj[0]
            x2 = p2[0] - obj[0]
            y1 = p1[1] - obj[1]
            y2 = p2[1] - obj[1]

            r = obj[2]

            dx = x2-x1
            dy = y2-y1
            dr = np.sqrt(dx*dx + dy*dy)
            D = x1*y2 - x2*y1

            cross = False
            disc = r**2*dr**2 - D**2
            if disc>=0:
                cross = True
                xp = (D*dy + np.sign(dy)*dx*np.sqrt(r**2*dr**2 - D**2)) / dr**2
                xm = (D*dy - np.sign(dy)*dx*np.sqrt(r**2*dr**2 - D**2)) / dr**2
                yp = (-D*dx + np.abs(dy)*np.sqrt(r**2*dr**2 - D**2)) / dr**2
                ym = (-D*dx - np.abs(dy)*np.sqrt(r**2*dr**2 - D**2)) / dr**2
                
                dp = np.sqrt( (xp+obj[0])**2 + (yp+obj[1])**2 )
                dm = np.sqrt( (xm+obj[0])**2 + (ym+obj[1])**2 )
                if dp >= dm:
                    p2 = (xm+obj[0],ym+obj[1])
                else:
                    p2 = (xp+obj[0],yp+obj[1])

            return p2, cross

        x,z,_,theta,_,_ = self.plant.state
        beam = self.lidar_range
        angle = self.lidar_angle
        res = self.lidar_res
        
        rays = np.zeros((2,res))
        objects = seenObjects([x,z,theta],angle,beam,self.objects)
            
        angles = np.linspace(-angle/2,angle/2,num=res)
        for i,a in enumerate(angles):
            x1,y1 = x,z
            x2,y2 = x+beam*np.cos(theta+a),z+beam*np.sin(theta+a)
            
            # check for lidar intersection
            for obj in objects.T:
                p,cross = intersect((x1,y1),(x2,y2),obj)
                if cross and (rays[0,i] == 0 or np.linalg.norm(rays[:,i]) > np.sqrt((p[0]-x1)**2+(p[1]-y1)**2)):
                    x2,y2 = p
                    rays[0,i] = x2-x1
                    rays[1,i] = y2-y1
        return rays

    # Updates occupency grid
    def update_grid(self, rays,res=1):
        # returns the neighbors of a vertex v in G
        def neighbors(v,G):
            
            # initial neighbors
            i,j = v
            row = np.array([i-1,i-1,i-1,i,i,i+1,i+1,i+1])
            col = np.array([j-1,j,j+1,j-1,j+1,j-1,j,j+1])

            # remove cells out of range
            r,c = np.shape(G)
            low = np.logical_and(row>=0, col>=0)
            high = np.logical_and(row<=r, col<=c)
            idx = np.logical_and(low, high)
            row = np.delete(row,~idx)
            col = np.delete(col,~idx)

            return list(zip(row,col))
        
        beam = self.lidar_range
        r,c = int(1.5*beam/res), int(3*beam/res)
        grid = np.zeros((r,c))
        x,y = int(c/6),int(r/2)
        self.ogOrigin = [x,y]
                    
        grid[y,x] = 1

        r,c = np.shape(rays)
        for i in range(c):
            if rays[0,i] != 0:
                grid[(y+rays[1,i]).astype(int),(x+rays[0,i]).astype(int)] = 1
        
        grid[y,x] = 0
        padded = grid.copy()
        r,c = np.shape(grid)
        for i in range(r):
            for j in range(c):
                if grid[i,j]:
                    n = neighbors((i,j),grid)
                    for k in n:
                        padded[k] = 1
    
        return padded

    # Resets the state of the system and creates obstacles
    def reset(self, initState):
        self.plant.reset(initState)
        self.stateEstimate = initState[2:]
        self.droneTime = 0
        self.calculateGains(initState)
        self.delta_e_actual = 0
        self.ogOrigin = [0,0]

        # Setting up obstacles
        x_pos = np.linspace(initState[0]+25,initState[0]+240,self.numObstacles)
        #x_pos = np.hstack((x_pos,x_pos))
        y_pos = np.random.uniform(low=initState[1]-10,high=initState[1]+10,size=self.numObstacles)
        #y_pos = np.hstack((y_pos,y_pos+8))
        sizes = np.random.uniform(low=1,high=1,size=self.numObstacles)
        self.objects = np.stack((x_pos,y_pos,sizes))

        return self.state

    # Main step of drone simulation
    def step(self, actionDrone):
        # Control inputs
        thrust, delta_eFF, stateCommand  = actionDrone

        error = (np.array([self.stateEstimate]).T-np.array([stateCommand]).T)
        controlFull = self.KFull @ error
        delta_e = delta_eFF-controlFull[0][0]
        actionPlant = (thrust, delta_e)
        self.delta_e_actual = delta_e

        observation, reward, terminated = self.plant.step(actionPlant)
        self.updateEKF(actionPlant, observation)
        self.droneTime += self.dt

        rays = self.update_rays()
        grid = self.update_grid(rays)

        for obstacleI in range(self.objects.shape[1]):
            xtrue = self.plant.state[0]
            ytrue = self.plant.state[1]
            if ((self.objects[0, obstacleI] - xtrue)**2 + (self.objects[1, obstacleI] - ytrue)**2) < self.objects[2, obstacleI]**2:
                terminated = True
                print("Obstacle Collision")

        if not terminated:
            reward = 1.0
        elif self.steps_beyond_terminated is None:
            self.steps_beyond_terminated = 0
            reward = 0.0
        else:
            if self.steps_beyond_terminated == 0:
                print( "You are calling 'step()' even though this environment has already returned terminated = True.")
            self.steps_beyond_terminated += 1
            reward = 0.0

        return self.stateEstimate, grid, reward, terminated

