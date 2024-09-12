import mock
import numpy as np
def defaultConfig():
    config = mock.Mock()

    config.dt = 0.01

    #Shape of vehicle characteristics
    config.m = 3.2
    config.Iyy = 1/12*config.m*0.8**2
    config.S = 0.25
    config.c = 0.13

    #Aero parameters
    config.Cl_0 = 0.5
    config.Cl_alpha = 0.1*180/np.pi
    config.Cd_0 = 0.1
    config.K = 0.05
    config.Cm_0 = 0.5
    config.Cm_alpha = -0.14*180/np.pi
    config.Cm_alpha_dot = -0.008*180/np.pi
    config.Cm_delta_e = 0.2

    #Random values
    config.rhoSTD = 0
    config.Restimation = np.diag([0.25, (0.25/180*np.pi)**2])
    config.Pestimation = np.eye(4)/100
    config.Qestimation = np.diag([1/100, 0.1/180*np.pi**2, 0.1/180*np.pi**2,0.1/180*np.pi**2 ])

    # Thresholds for cutting sim
    config.x_thres = 10000
    config.z_thresHigh = 500
    config.z_thresLow = 0
    config.v_thres = 50
    config.theta_thres = np.pi/6*2
    config.theta_dot_thres = np.pi*10
    config.gamma_thres = np.pi/4

    #Obstacle settings
    config.numObstacles = 0

    #Controls settings
    config.Qcontrol =  np.array([[10,0,0,0],[0,10, 0,0], [0,0, 0,0], [0, 0, 0, 10000]])
    config.Rcontrol =  np.array([[5]])

    config.lidar_range = 50
    config.lidar_angle = 100*np.pi/180
    config.lidar_res = int(config.lidar_angle*180/np.pi)

    return config