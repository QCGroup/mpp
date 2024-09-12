import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
from matplotlib.collections import LineCollection
from matplotlib import cm

# Traditional system state plots
def state_plots(traj, elvActual):

    def plot_state(ax,x,y,xlabel,ylabel):
        lim = 2
        ax.plot(x,y)
        ax.set_ylim([min(y)-lim,max(y)+lim])
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

        return
    x = traj[0,:]
    z = traj[1,:]
    v = traj[2,:]
    theta = traj[3,:]
    thetaDot = traj[4,:]
    gamma = traj[5,:]
    time = traj[6,:]

    plt.close()
    fig, axs = plt.subplots(3,2, figsize=(8,8))
    plot_state(axs[0, 0],x,z,"Downrange distance (m)","Height (m)")
    plot_state(axs[0, 1],time,(theta-gamma)*180/np.pi,"Time (s)","alpha (deg)")
    plot_state(axs[1, 0],time,gamma*180/np.pi,"Time (s)","Gamma (deg)")
    plot_state(axs[1, 1],time,v,"Time (s)","Velocity (m/s)")
    plot_state(axs[2, 0],time,theta*180/np.pi,"Time (s)","Theta (deg)")
    plot_state(axs[2, 1],time,elvActual,"Time (s)","Elevator")

    fig.show()
    return

# Plots for the MPC
def state_plots_command(traj,control,xCommand, yCommand, elivCommand,elvActual, refCommand, drone):
    def plot_state(ax,x,y,xlabel,ylabel):
        lim = 2
        ax.plot(x,y)
        ax.set_ylim([min(y)-lim,max(y)+lim])
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

        return
    vRef = refCommand[:,0]
    thetaRef = refCommand[:,1]
    gammaRef = refCommand[:,3]

    x = traj[0,:]
    z = traj[1,:]
    v = traj[2,:]
    theta = traj[3,:]
    thetaDot = traj[4,:]
    gamma = traj[5,:]
    time = traj[6,:]

    plt.close()
    fig, axs = plt.subplots(3,3, figsize=(14,10))
    plot_state(axs[0, 0],xCommand,yCommand,"Downrange distance (m)","Height (m)")
    plot_state(axs[0, 0],x,z,"Downrange distance (m)","Height (m)")
    axs[0,0].scatter(drone.objects[0,:],drone.objects[1,:], color = "red" )
    axs[0,0].set_ylim((30, 70))
    print(gamma.shape)
    plot_state(axs[0, 1],time,(thetaRef-gammaRef)*180/np.pi,"Time (s)","alpha (deg)")
    plot_state(axs[0, 1],time,(theta-gamma)*180/np.pi,"Time (s)","alpha (deg)")
    plot_state(axs[0, 2],time,elivCommand,"Time (s)","Elevator Command")
    plot_state(axs[0, 2],time,elvActual,"Time (s)","Elevator Actual")
    plot_state(axs[1, 0],time,gammaRef*180/np.pi,"Time (s)","Gamma (deg)")
    plot_state(axs[1, 0],time,gamma*180/np.pi,"Time (s)","Gamma (deg)")
    plot_state(axs[1, 1],time,vRef,"Time (s)","Velocity (m/s)")
    plot_state(axs[1, 1],time,v,"Time (s)","Velocity (m/s)")
    plot_state(axs[2, 0],time,thetaRef*180/np.pi,"Time (s)","Theta (deg)")
    plot_state(axs[2, 0],time,theta*180/np.pi,"Time (s)","Theta (deg)")
    plot_state(axs[2, 1],time,control,"Time (s)","Thrust Command (N)")
    fig.show()
    return

# Kalman filter plots
def est_plots(traj, trajEst):

    def plot_state(ax,x,y,xlabel,ylabel):
        lim = 2
        ax.plot(x,y)
        ax.set_ylim([min(y)-lim,max(y)+lim])
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)
        return
    def plot_state_dashed(ax,x,y,xlabel,ylabel):
        lim = 2
        ax.plot(x,y,'--')
        ax.set_ylim([min(y)-lim,max(y)+lim])
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

        return
    
    x = traj[0,:]
    z = traj[1,:]
    v = traj[2,:]
    vEst =  trajEst[0,:]
    theta = traj[3,:]
    thetaEst = trajEst[1,:]
    thetaDot = traj[4,:]
    thetaDotEst = trajEst[2,:]
    gamma = traj[5,:]
    gammaEst = trajEst[3,:]
    time = traj[6,:]

    plt.close()
    fig, axs = plt.subplots(4,2, figsize=(10,10))
    plot_state(axs[0, 0],time,theta*180/np.pi,"Time (s)", "Theta (deg)")
    plot_state(axs[0, 1],time,thetaEst*180/np.pi,"Time (s)", "Theta Estimate (deg)")
    plot_state(axs[1, 0],time,thetaDot*180/np.pi,"Time (s)", "ThetaDot (deg/s)")
    plot_state(axs[1, 1],time,thetaDotEst*180/np.pi,"Time (s)", "ThetaDot Estimate (deg/s)")
    plot_state(axs[2, 0],time,v,"Time (s)", "Velocity (m/s)")
    plot_state(axs[2, 1],time,vEst,"Time (s)", "Velocity Estimate (m/s)")
    plot_state(axs[3, 0],time,gamma*180/np.pi,"Time (s)", "Gamma (deg)")
    plot_state(axs[3, 1],time,gammaEst*180/np.pi,"Time (s)", "Gamma Estimate (deg)")
    fig.show()
    return

# Plots for open loop trajectory planning
def open_loop_plots(tStart, tEnd,xCommand, yCommand,thrstCommand, elivCommand, drone):
    def plot_state(ax,x,y,xlabel,ylabel):
        lim = 2
        ax.plot(x,y)
        ax.set_ylim([min(y)-lim,max(y)+lim])
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

        return
    time = np.array(range(tStart, tEnd))/100
    plt.close()
    fig, axs = plt.subplots(3,1, figsize=(10,10))
    plot_state(axs[0],xCommand[tStart:tEnd],yCommand[tStart:tEnd],"Downrange distance (m)","Height (m)")
    axs[0].scatter(drone.objects[0,:],drone.objects[1,:], color = "red" )
    axs[0].set_ylim((min(yCommand[tStart:tEnd]) - 10, max(yCommand[tStart:tEnd]) + 10))
    axs[0].set_xlim((min(xCommand[tStart:tEnd]) - 10, max(xCommand[tStart:tEnd]) + 10))

    plot_state(axs[1],time,elivCommand[tStart:tEnd],"Time (s)","Elevator Command")
    plot_state(axs[2],time,thrstCommand[tStart:tEnd],"Time (s)","Thrust Command")

    fig.show()
    return

def plot_rrt_lines_flipped(ax, T, color_costs=True, cmap="viridis", color="tan", alpha=1.0):
    lines = []
    costs = []
    for e1, e2 in T.edges():
        lines.append((np.flip(T.nodes[e1]["pt"]), np.flip(T.nodes[e2]["pt"])))
        costs.append(T.edges[e1, e2]["cost"])
    if color_costs:
        norm = Normalize(vmin=min(costs), vmax=max(costs))
        colors = cm.get_cmap(cmap)(norm(costs))
        lc = LineCollection(lines, colors=colors, alpha=alpha)
    else:
        lc = LineCollection(lines, color=color, alpha=alpha)
    ax.add_collection(lc)

def tree_plot_2d_vectors(vectors, og: np.ndarray, cmap: str = "Greys", vmin=0, vmax=1):
    fig, ax = plt.subplots()

    for vector in vectors:
        x = vector[0]
        y = vector[1]
        index = round(vector[2])

        if index > -1 and index < len(vectors):
            target_vector = vectors[index]
            target_x = target_vector[0]
            target_y = target_vector[1]

            ax.plot([x, target_x], [y, target_y], 'b-')
        ax.scatter(x, y, color='royalblue')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('RRT Tree plot')
    ax.grid(True)

    norm = Normalize(vmin=vmin, vmax=vmax)
    ax.imshow(og.T, cmap=cmap, norm=norm, origin="lower", interpolation=None)
    plt.show()
