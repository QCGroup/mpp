import numpy as np
import math
from scipy.spatial import Delaunay
from cvxopt import matrix, solvers
from rrtplanner import RRTStar
from mosek import iparam
import time
from RRTAR import RRTAR
import cProfile

# Calculates the length of a path given as a series of points
def pathlens(path_points):
    dist = np.zeros(path_points.shape[0])
    for i in range(path_points.shape[0]):
        dist[i] = math.dist(path_points[i, 0,:], path_points[i,1,:])
    return dist

# Calculates an occupancy grid given convex constraints
def calculateOccupancyGrid(constraints):
    yv, xv = np.meshgrid(range(0, 200), range(0, 200))
    og = np.zeros_like(xv)

    for constraint in constraints:
        hull = Delaunay(constraint)
        ogInner = np.zeros_like(xv)
        len, wid = xv.shape
        for i in range(len):
            for j in range(wid):
                ogInner[i, j] = hull.find_simplex([xv[i,j],yv[i,j]])
        og[ogInner != -1] = 1
    return og

# Resamples a path into a series of points that can be optimized.  These points are evenly spaced.
def calculateReferencePoints(timeEnd, path_points):
    pathLengths = pathlens(path_points)
    vel = sum(pathLengths)/timeEnd
    distanceOfPoints = np.linspace(start = 0, stop = sum(pathLengths), num = timeEnd)
    referencePoints = []
    referencePoints.append(path_points[0, 0, :] * 1) #Fixes shallow copy bug
    referenceVels = []
    pathLengths[-1] = pathLengths[-1] + 10**-3
    index = 0
    for i in range(1, distanceOfPoints.size):
        while(distanceOfPoints[i] > pathLengths[index]):
            distanceOfPoints = distanceOfPoints - pathLengths[index]
            index = index + 1
        multiplier = distanceOfPoints[i]/pathLengths[index]
        referencePoints.append(path_points[index, 0, :] *(1-multiplier) + path_points[index, 1, :]*multiplier)
        referenceVels.append(referencePoints[i]- referencePoints[i-1])
    referenceVels.append((referencePoints[0]- referencePoints[1]) * 0)

    return referencePoints, referenceVels

# Performs a search of the occupency grid along a single direction
def singleRay(origin, direction, og, limit = 10):
    xlim, ylim = og.shape
    direction[direction == 0] = 10**-8
    cord = np.array(origin)
    traverse = 0
    invdirection = 1 / direction
    directionLessThanZero = direction < 0
    while traverse < limit:
        distance = np.floor(cord+1) - cord
        distance[directionLessThanZero] = np.ceil(cord[directionLessThanZero] -1) - cord[directionLessThanZero]
        ratio = distance * invdirection
        len = min(ratio)
        cord = len*direction + cord
        if (cord[0] >= xlim or cord[1] >= ylim):
            return np.floor(cord)
        if (og[int(cord[0]),int(cord[1])]):
            return np.floor(cord)
        traverse += len
    return np.floor(cord)


def singleRay2(origin, direction, og, limit=10):
    cord = np.array(origin)
    traverse = 0
    shape = og.shape  # Cache the shape of the occupancy grid
    direction[direction == 0] = 10**-8
    
    while traverse < limit:
        distance = 1  # You can adjust the distance increment as needed
        cord += distance * direction
        
        if (np.greater((cord.astype(int) + 1), shape)).any() or og[tuple(cord.astype(int))]:
            return np.floor(cord)
        
        traverse += distance
    
    return np.floor(cord)

# Performs a search of the occupency grid in multiple directions from a single point.
def searchFromCord(cord, directions, og, limit = 10):
    sol = []
    for direct in directions:
        sol.append(singleRay(cord, direct, og, limit))
    return np.array(sol)

# Returns the directions used in the searchFromCord function
def generateDirections(numDirects = 8):
    angles = np.linspace(0, 2*np.pi*(numDirects-1)/(numDirects), numDirects)
    sol = []
    for ang in angles:
        sol.append([np.cos(ang), np.sin(ang)])
    return np.array(sol)


# The main function for this library.  Search as occupency grid around the given reference points to create a set of linearly constrained open points.  
# These points are optimized to minimize a control effort and comply with the system dynamics.
def localTrajOpt(A, B, tEnd, og, referencePoints, referencePointsDyn, xstart, xgoal):

    dimX = len(A)
    dimU = B.shape[1]


    #Inequality constraints
    numDirections = 8
    directs = generateDirections(numDirections)
    Gbar = np.zeros(((numDirections+1)*(tEnd+1), (dimX+dimU)*tEnd+dimX))
    hbar = np.zeros(((numDirections+1)*(tEnd+1), 1))
    Refs0 = list(zip(*referencePoints))[0]
    Refs1 = list(zip(*referencePoints))[1]
    RefsDyn0 = list(zip(*referencePointsDyn))[0]
    RefsDyn1 = list(zip(*referencePointsDyn))[1]

    for i in range(tEnd+1):
        cords = searchFromCord([Refs0[i],Refs1[i]], directs, og, limit = 20)
        hbar[i*numDirections:(i+1)*numDirections, 0] = np.sum(directs*(cords-np.array([Refs0[i],Refs1[i]])+np.array([RefsDyn0[i],RefsDyn1[i]])), axis=1)-1*(i/tEnd)-1.5
        Gbar[i*numDirections:(i+1)*numDirections, i*(dimX+dimU):i*(dimX+dimU)+2] = np.flip(directs, axis = 1)

    # Adding positive control constraints
    for i in range(tEnd):
        hbar[(tEnd+1)*numDirections+i, 0] = 4.3
        Gbar[(tEnd+1)*numDirections+i, i*(dimX+dimU)+ dimX+1] = -1
    hbar = hbar

    IA = np.eye(dimX)
    Abar = np.zeros((dimX*(tEnd+1),(dimX+dimU)*tEnd+dimX))
    Bbar = np.zeros((dimX*(tEnd+1),1))

    steadyStates = [3, 4]
    for x in steadyStates:
        IA[x, :] = IA[x, :] - A[x, :] 
        A[x, :] = A[x, :] * 0

    #Equality constraints
    for i in range(tEnd):
        leftInd = i*(dimX+dimU)
        righInd =  (i+1)*(dimX+dimU)+dimX
        upInd = i*dimX
        botInd = (i+1)*dimX
        Abar[upInd:botInd, leftInd:righInd] = np.hstack([A, B, -IA])
        Bbar[upInd:upInd+len(referencePointsDyn[i])] = 0
    leftInd = 0
    righInd =  dimX
    upInd = tEnd*dimX
    botInd = (tEnd+1)*dimX
    Abar[upInd:botInd, leftInd:righInd] = IA
    Bbar[upInd:botInd,0] = xstart

    #toc = time.perf_counter()
    #print(f"line 143 in {toc - tic:0.4f} seconds")
    #Cost function
    finalStateWeight = 100
    Qbar = np.zeros(((dimX+dimU)*(tEnd)+dimX,(dimX+dimU)*(tEnd)+dimX))
    IU = np.eye(dimU)
    IU[1,1] = 3
    psiDotWeight = 0
    for i in range(tEnd):
        psiDotIndex = i*(dimX+dimU)+5
        Qbar[psiDotIndex, psiDotIndex] = psiDotWeight

        leftInd = i*(dimX+dimU)+dimX
        righInd =  (i)*(dimX+dimU)+dimU+dimX
        upInd = i*(dimX+dimU)+dimX
        botInd = (i)*(dimX+dimU) +dimU+dimX
        Qbar[upInd:botInd, leftInd:righInd] = IU
    leftInd = tEnd*(dimX+dimU)
    righInd =  (tEnd)*(dimX+dimU)+dimX
    upInd =tEnd*(dimX+dimU)
    botInd =  (tEnd)*(dimX+dimU)+dimX
    Qbar[upInd:botInd, leftInd:righInd] = np.eye(dimX)*finalStateWeight

    Pbar = np.zeros(((dimX+dimU)*(tEnd)+dimX,1))
    Pbar[upInd:botInd, 0] = -xgoal*finalStateWeight
    
    #Run CVX
    Qop = matrix(Qbar)
    pop = matrix(Pbar)
    Gop = matrix(Gbar)
    hop = matrix(hbar)
    Aop = matrix(Abar)
    bop = matrix(Bbar)
    options = {'show_progress': False, 'maxiters':10}#, 'mosek': {iparam.log: 0, iparam.max_num_warnings:0}}
    sol = solvers.qp(Qop, pop, Gop, hop, Aop, bop, options = options)#, solver="mosek")
    # options = {'show_progress': False, 'maxiters':10, 'mosek': {iparam.log: 0, iparam.max_num_warnings:0}}
    # sol = solvers.qp(Qop, pop, Gop, hop, Aop, bop, options = options, solver="mosek")

    z = sol['x']
    cost = sol['primal objective']
    infeasibility = sol['primal infeasibility']


    #Output solution
    xsol = []
    usol = []
    if (infeasibility is not None):
        for i in range(tEnd):
            xsol.append(z[i*(dimX+dimU):i*(dimX+dimU)+dimX])
            usol.append(z[i*(dimX+dimU)+dimX:i*(dimX+dimU)+dimX+dimU])
        xsol.append(z[(tEnd)*(dimX+dimU)])
    else: 
        infeasibility = 10**10

    return xsol, usol, cost, infeasibility

# Trajectory Generation Demo rolled into a single function to make the demo cleaner
def TrajGenMPP(drone, grid, thrustCommand, elivCommand, xCommand, yCommand, refStates, refCommand, thrustCommandVal,elivCommandRef, iterations, iter, pathSamples=25):
    midpoint = drone.ogOrigin[1]
    offset = 10
    lookAhead = 54
    orignX = drone.ogOrigin[0]
    xstart = np.array([orignX, offset]) 
    xgoal = [[orignX+lookAhead, offset], [orignX+lookAhead, offset/2],  [orignX+lookAhead, offset/4*2],  [orignX+lookAhead, offset*3/2]]

    searchGrid = grid[midpoint-offset:midpoint+offset, 0:85].T.astype(int)
    vectorsSave = []
    limitIter = 100
    li = 0
    rrt = RRTAR()

    while(len(vectorsSave) < pathSamples and li < limitIter):
        vectors = rrt.runRRT(xstart[0], xstart[1], 150, 2.5, 5, searchGrid, xgoal)
        li = li+1
        for i in range(1, len(vectors)):
            if (len(vectorsSave) < pathSamples):
                vectorsSave.append(vectors[i])

    thrustCommandVal = 4.3
    refStates = drone.coherentCommand(thrustCommandVal,0/180*np.pi)
    controlRef = (thrustCommandVal, drone.elevatorFromAlpha(refStates[1]-refStates[3]))

    timeMulti = 25
    xstart = np.concatenate(([0.,0.], np.array(drone.stateEstimate) - np.array(refStates) ))
    xgoal = np.concatenate(([0.,50 - drone.plant.state[1]], [0.,0.,0.,0.]))
    tEnd = 450

    minCost = 10000000

    for vi in range(len(vectorsSave)):
        Acts, Bcts = drone.calculateCTSABMatrix(refStates, controlRef)
        Aopt = np.eye(Acts.shape[0]+2)
        #A = Aopt + env.calculateANumerical(stateRefFull, controlRef, env.rhoNom, step=10**-5)*env.dt
        Aopt[0, 2] = drone.dt*timeMulti
        Aopt[1, 5] = refStates[0]*drone.dt*timeMulti
        Aopt[2:,2:] = Aopt[2:,2:]+Acts*drone.dt*timeMulti
        Bopt = np.zeros([6,2])
        Bopt[2:,0:1] = Bcts*drone.dt*timeMulti
        alphaEst = refStates[1] - refStates[3]
        Bopt[2, 1] = 1/drone.m*np.cos(alphaEst)*drone.dt*timeMulti
        Bopt[5, 1] = 1/drone.m*np.sin(alphaEst)/refStates[0]*drone.dt*timeMulti

        path_pts = np.zeros((len(vectorsSave[vi]) - 1, 2, 2))
        for i in range(len(vectorsSave[vi]) - 1):
            path_pts[i] = np.array([[vectorsSave[vi][i][1], vectorsSave[vi][i][0]],
                            [vectorsSave[vi][i+1][1], vectorsSave[vi][i+1][0]]])
        referencePoints, referenceVels = calculateReferencePoints(math.floor(tEnd/timeMulti)+1, path_pts)
        referencePointsDyn, referenceVelsDyn = calculateReferencePoints(math.floor(tEnd/timeMulti)+1, path_pts)
        for i in range(len(referencePointsDyn)):
            referencePointsDyn[i][0] = referencePointsDyn[i][0]-refStates[0]*np.sin(refStates[3])*i*drone.dt*timeMulti-offset
            referencePointsDyn[i][1] = referencePointsDyn[i][1]-refStates[0]*np.cos(refStates[3])*i*drone.dt*timeMulti-orignX

        xsol, usol, cost, infeasbility = localTrajOpt(Aopt, Bopt, math.floor(tEnd/timeMulti), grid[midpoint-offset:, 0:80], referencePoints, referencePointsDyn, xstart, xgoal)
        if (infeasbility < 0.01 and cost < minCost):
            smoother = 0
            if(iter > smoother):
                thrustCommandCur = thrustCommand[iter]
                elivCommandCur = elivCommand[iter] 
                xCommandCur = xCommand[iter]
                yCommandCur = yCommand[iter]
                refCommandCur = refCommand[iter, :]
            minCost = cost
            for i in range(np.min((tEnd-timeMulti-1, iterations-iter))):
                iDrop = math.floor(i/timeMulti)
                iNext = iDrop+1
                ifrac1 = (timeMulti - i%timeMulti)/timeMulti
                ifrac2 = (i%timeMulti)/timeMulti

                thrustCommand[iter+i] = ifrac1*usol[iDrop][1] + ifrac2*usol[iNext][1]  + thrustCommandVal 
                elivCommand[iter + i] = ifrac1*usol[iDrop][0] + ifrac2*usol[iNext][0] + elivCommandRef
                xCommand[iter+i] = ifrac1*xsol[iDrop][0] + ifrac2*xsol[iNext][0] +refStates[0]*np.cos(0)*i*drone.dt+drone.plant.state[0]
                yCommand[iter+i] = ifrac1*xsol[iDrop][1] + ifrac2*xsol[iNext][1] + refStates[0]*np.sin(0)*i*drone.dt+drone.plant.state[1]
                refCommand[iter+i, :] = ifrac1*xsol[iDrop][2:6].T + ifrac2*xsol[iNext][2:6].T + np.array(refStates)
            if(iter > smoother):
                for i in range(smoother):
                    ifrac1 = (smoother - i)/smoother
                    ifrac2 = (i)/smoother

                    thrustCommand[iter+i] = ifrac1*thrustCommandCur + ifrac2*thrustCommand[iter+i]
                    elivCommand[iter + i] = ifrac1*elivCommandCur + ifrac2*elivCommand[iter + i]
                    xCommand[iter+i] = ifrac1*xCommandCur + ifrac2*xCommand[iter+i]
                    yCommand[iter+i] = ifrac1*yCommandCur + ifrac2*yCommand[iter+i]
                    refCommand[iter+i, :] = ifrac1*refCommandCur+ ifrac2*refCommand[iter+i, :]

    return thrustCommand, elivCommand, xCommand, yCommand, refCommand


# Trajectory Generation Demo rolled into a single function to make the demo cleaner
def TrajGenRRT(drone, grid, thrustCommand, elivCommand, xCommand, yCommand, refStates, refCommand, thrustCommandVal,elivCommandRef, iterations, iter):
    stateRef = np.hstack(([0,50], refStates))

    # Data
    elvActual = np.zeros(iterations)
    thrustCommand = np.zeros(iterations)+thrustCommandVal
    stateCommand = drone.coherentCommand(thrustCommand[0],0/180*np.pi)
    elivCommandRef = drone.elevatorFromAlpha(stateCommand[1]-stateCommand[3])
    elivCommand = np.zeros(iterations) + elivCommandRef
    xCommand = np.zeros(iterations)
    yCommand = np.zeros(iterations)
    refCommand = np.zeros((iterations, 4))
    refCommand[0] = refStates

    elvActual[iter] = drone.delta_e_actual
    midpoint = drone.ogOrigin[1]
    offset = 10
    lookAhead = 54
    orignX = drone.ogOrigin[0]
    #xstart = np.array([orignX, offset]) 
    #xgoal = np.array([offset, orignX+lookAhead])
    xstart = np.array([offset, orignX]) 
    xgoal = np.array([offset, orignX+lookAhead])

    n = 3000
    r_rewire = 5
    rrts = RRTStar(grid[midpoint-offset:midpoint+offset, 0:85], n, r_rewire, pbar = False) 
    T, gv = rrts.plan(xstart, xgoal)
    path = rrts.route2gv(T, gv)
    path_pts = rrts.vertices_as_ndarray(T, path)
    tEnd = 450

    referencePoints, referenceVels = calculateReferencePoints(tEnd+1, path_pts)
    for i in range(np.min((tEnd-1, iterations-iter))):
        dx = referencePoints[i+1][1] - referencePoints[i][1]
        dy = referencePoints[i+1][0] - referencePoints[i][0]

        thrustCommandTemp = thrustCommandVal
        gammaTemp = math.atan2(dy, dx)
        tempCommand = drone.coherentCommand(thrustCommandTemp, gammaTemp)
        refCommand[iter+i, :] = tempCommand
        thrustCommand[iter+i] = thrustCommandTemp
        elivCommand[iter + i] = drone.elevatorFromAlpha(refCommand[iter+i, 1])
        xCommand[iter+i] = drone.plant.state[0] + referencePoints[i][1]-orignX
        yCommand[iter+i] = drone.plant.state[1] + referencePoints[i][0]-offset

    return thrustCommand, elivCommand, xCommand, yCommand, refCommand


