# Draw plot
import matplotlib.pyplot as plt
import scipy.spatial.distance as sciDist
import numpy as np
import copy
def isSimple(tpMat):
    # simple: i.e., this can be solved with chain tree and intersection of circles/arc sects. 
    # find links and set up joint table. 
    # actuator will be noted with negative value. 
    fixParam = [1,3]
    jT = {}
    fixJ=[]
    kkcJ=[]
    chain={}
    
    # step 1, initialize, set all joints and links to unknown (0 in jointTable) and jointLinkTable. 
    for i in range(tpMat.shape[0]):
        jT[i] = 0
        chain[i] = {'from': None, 'next': []}

    # step 2, set all ground joints to known (1 to be known)
    for i in range(tpMat.shape[0]):
        if tpMat[i,i] in fixParam:
            jT[i] = 1
            fixJ.append(i)
            kkcJ.append((i, 'fixed', i))
            chain[i]['from'] = i
    
    
    # step 3, set joints in the kinematic chain to known
    pivotJ = fixJ
    while True: 
        prevCtr = len(kkcJ)
        newJ = []
        for i in pivotJ:
            for j in range(tpMat.shape[1]):
                if tpMat[i,j] < 0 and jT[j] == 0:
                    jT[j] = 1
                    newJ.append(j)
                    kkcJ.append((j, 'chain', i))
                    chain[i]['next'].append(j)
                    chain[j] = {'from': i, 'next': []}
                
        if len(kkcJ) == prevCtr:
            break
        else:
            pivotJ = newJ # This is based on the idea of tree node expansion

    if len(kkcJ) == tpMat.shape[0]:
        print(jT)
        return kkcJ, chain, True
    
    # step 4, set joints that can be solved through the intersection of circles to known
    while True:
        foundNew = False
        for k in jT:
            if jT[k] == 0:
                for i, _, _ in kkcJ:
                    for j, _, _ in kkcJ:
                        if i<j and tpMat[i,k] * tpMat[j,k] != 0 and not foundNew:
                            foundNew = True
                            jT[k] = 1
                            kkcJ.append((k, 'arcSect', (i,j)))
        if not foundNew:
            break
    
    # return chain and isSimple (meaning you can solve this with direct chain) 
    return kkcJ, chain, len(kkcJ) == tpMat.shape[0]
# Direct kinematics: 
def computeChainByStep(step, rMat, pos_init, unitConvert = np.pi/180):
    pos_new = copy.copy(pos_init)
    dest, _ , root = step 
    pos_new[dest, 2] = rMat[root, dest]*unitConvert + pos_new[root, 2]
    c = np.cos(pos_new[dest,2])
    s = np.sin(pos_new[dest,2])
    posVect= pos_init[dest, 0:2] - pos_init[root, 0:2]
    pos_new[dest, 0] = posVect[0]*c-posVect[1]*s+pos_new[root,0]
    pos_new[dest, 1] = posVect[0]*s+posVect[1]*c+pos_new[root,1]
    return pos_new


# Inverse kinematics: 
def computeArcSectByStep(step, posOld, distMat):
    posNew = copy.copy(posOld)
    ptSect, _, centers = step
    cntr1, cntr2 = centers
    r1s = distMat[cntr1, ptSect]
    r2s = distMat[cntr2, ptSect]
    if r1s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr1,0:2]
    elif r2s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr2,0:2]
    else:
        ptOld  = posOld[ptSect,0:2] 
        ptCen1 = posOld[cntr1,0:2]
        ptCen2 = posOld[cntr2,0:2]
        d12 = np.linalg.norm(ptCen1 - ptCen2)
        if d12 > r1s + r2s or d12 < np.absolute(r1s-r2s):
            return posOld, False
        elif np.absolute(r1s + r2s - d12) <10e-12 or np.absolute(np.absolute(r1s - r2s) - d12) < 10e-12: # Singular point
            posNew[ptSect, 0:2] = (ptCen1 + ptCen2)/2
        else: 
            # a means the LENGTH from cntr1 to the mid point between two intersection points. 
            # h means the LENGTH from the mid point to either of the two intersection points. 
            # v means the Vector from cntr1 to the mid point between two intersection points. 
            # vT 90 deg rotation of v
            a = (r1s**2 - r2s**2 + d12**2)/(d12*2)
            h = np.sqrt(r1s**2 - a**2)
            v = ptCen2 - ptCen1
            vT= np.array([-v[1], v[0]])
            r1= a/d12
            r2= h/d12
            ptMid= ptCen1 + v*r1
            sol1 = ptMid + vT*r2
            sol2 = ptMid - vT*r2
            if np.linalg.norm(sol1 - ptOld) > np.linalg.norm(sol2 - ptOld):
                posNew[ptSect, 0:2] = sol2
            else:
                posNew[ptSect, 0:2] = sol1
        return posNew, True
# Basic data for computing a mechanism. 
def computeDistMat(tpMat, pos):
    cdist = sciDist.cdist
    tpMat = copy.copy(np.absolute(tpMat))
    tpMat[list(range(0,tpMat.shape[0])), list(range(0,tpMat.shape[1]))] = 0
    return np.multiply(cdist(pos[:,0:2], pos[:,0:2]), tpMat)


def computeCurveSimple(tpMat, pos_init, rMat, distMat = None, maxTicks = 360, baseSpeed= 1):
    # preps
    kkcJ, chain, isReallySimple = isSimple(tpMat) 
    if distMat == None:
        distMat = computeDistMat(tpMat, pos_init)
    poses = np.zeros((pos_init.shape[0], maxTicks, 3))
    
    # Set first tick
    poses[:,0,0:pos_init.shape[1]] = pos_init
    # Compute others by step. 
    meetAnEnd = False
    meetTwoEnds=False
    tick = 0
    offset=0
    while not meetTwoEnds:
        # get tick 
        tick = tick + 1
        if tick >= maxTicks:
            break
        # define "time"
        if not meetAnEnd:
            time = tick * baseSpeed
        else: 
            time = (tick - offset) * baseSpeed * (-1)
            #print(time)
        # step-wise switch solution 
        for step in kkcJ:
            if step[1] == 'fixed':
                joint = step[0]
                poses[joint,tick,:] = poses[joint,0,:]
                notMeetEnd = True
            elif step[1]=='chain':
                poses[:,tick,:] = computeChainByStep(step, rMat*time, poses[:,0,:])
                notMeetEnd = True
            elif step[1]=='arcSect':
                #print('tick', tick, ' for joint', step[0], 'stats', poses[:,tick,:], '\n')
                poses[step[0],tick,:] = poses[step[0],tick-1,:]
                poses[:,tick,:], notMeetEnd = computeArcSectByStep(step, poses[:,tick,:], distMat)
                # direction control. 
                if not notMeetEnd and not meetAnEnd:
                    #print('flip tick')
                    meetAnEnd = True
                    poses1 = np.flip(poses[:,0:tick,:], axis = 1)
                    poses2 = np.zeros_like(poses[:,tick:,:])
                    poses = np.concatenate([poses1, poses2], axis = 1)
                    tick = tick - 1
                    offset = tick
                    break
                elif not notMeetEnd and meetAnEnd:
                    #print('meet both ends, flip again')
                    poses = poses[:, 0:tick, :]
                    poses = np.flip(poses, axis = 1)
                    meetTwoEnds = True
                    break
            else:
                print('Unexpected step:, '+ step[1])
                break
    return poses, meetAnEnd, isReallySimple
# Inverse kinematics: 
def computeArcSectByStep(step, posOld, distMat, threshold = 0.1):
    threshold = np.max(distMat) * threshold
    posNew = copy.copy(posOld)
    ptSect, _, centers = step
    cntr1, cntr2 = centers
    r1s = distMat[cntr1, ptSect]
    r2s = distMat[cntr2, ptSect]
    if r1s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr1,0:2]
    elif r2s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr2,0:2]
    else:
        ptOld  = posOld[ptSect,0:2] 
        ptCen1 = posOld[cntr1,0:2]
        ptCen2 = posOld[cntr2,0:2]
        d12 = np.linalg.norm(ptCen1 - ptCen2)
        if d12 > r1s + r2s or d12 < np.absolute(r1s-r2s):
            #print('impossible \n')
            return posOld, False
        elif np.absolute(r1s + r2s - d12) <10e-12 or np.absolute(np.absolute(r1s - r2s) - d12) < 10e-12: # Singular point
            posNew[ptSect, 0:2] = (ptCen1 + ptCen2)/2
        else: 
            # a means the LENGTH from cntr1 to the mid point between two intersection points. 
            # h means the LENGTH from the mid point to either of the two intersection points. 
            # v means the Vector from cntr1 to the mid point between two intersection points. 
            # vT 90 deg rotation of v
            a = (r1s**2 - r2s**2 + d12**2)/(d12*2)
            h = np.sqrt(r1s**2 - a**2)
            v = ptCen2 - ptCen1
            vT= np.array([-v[1], v[0]])
            r1= a/d12
            r2= h/d12
            ptMid= ptCen1 + v*r1
            sol1 = ptMid + vT*r2
            sol2 = ptMid - vT*r2
            #print(ptOld, sol1, np.linalg.norm(sol1 - ptOld), sol2, np.linalg.norm(sol2 - ptOld))
            if np.linalg.norm(sol1 - ptOld) > np.linalg.norm(sol2 - ptOld):
                posNew[ptSect, 0:2] = sol2
                #print('sol2 selected \n')
            else:
                posNew[ptSect, 0:2] = sol1
            # detect if there's an abrupt change: 
            if np.max(np.linalg.norm(posNew - posOld, axis = 1)) > threshold:
                return posOld, False
            
        return posNew, True
# very typical 4-bar linkage mechanism 
tp4 = np.matrix([
    [1,-1, 1, 0, 0], 
    [1, 2, 0, 1, 1],
    [1, 0, 1, 1, 0],
    [0, 1, 1, 2, 1], 
    [0, 1, 0, 1, 2]
])


# Watt I 6-bar linkage mechanism
tp6 = np.matrix([
    [1,-1, 1, 0, 0, 1, 0],
    [1, 2, 0, 1, 1, 0, 0],
    [1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 2, 1, 0, 0], 
    [0, 1, 0, 1, 2, 0, 1],
    [1, 0, 1, 0, 0, 1, 1],
    [0, 0, 0, 0, 1, 1, 2]
])

tpTest = tp6
rMatTest = np.array([[0, 1.0, 0, 0, 0, 0, 0], 
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0]])

# rMatTest = np.array([[0, 1.0, 0, 0, 0], 
#                      [0, 0, 0, 0, 0],
#                      [0, 0, 0, 0, 0],
#                      [0, 0, 0, 0, 0],
#                      [0, 0, 0, 0, 0]])
tp8=np.matrix([
[1,-1,1,0,0,1,0,0,0,0],
[1,2,0,1,1,0,0,0,0,0],
[1,0,1,1,0,1,0,0,0,0],
[0,1,1,2,1,0,0,0,0,0],
[0,1,0,1,2,0,1,1,0,0],
[0,0,1,0,0,1,1,0,1,0],
[0,0,0,0,1,1,2,1,1,0],
[0,0,0,0,1,0,1,2,0,1],
[0,0,0,0,0,1,1,0,2,1],
[0,0,0,0,0,0,0,1,1,2]])

# This does full rotation. 
posInitTest = np.array([[-4.899366, -2.01123],
                        [-3.924365, 0.172104],
                        [2.200635, -2.319563],
                        [1.133968, 2.205437],
                        [0.725635, 3.063771],
                        [7.008334, -1.024999],
                        [5.600636, 5.355437]])


jdxTest1, meetAnEnd1, isReallySimple1 = computeCurveSimple(tpTest, posInitTest, rMatTest, distMat = None)
print('final', jdxTest1.shape, meetAnEnd1, isReallySimple1)

# Draw the first mechanism (fully rotatable)
plt.plot(jdxTest1[4,:,0], jdxTest1[4,:,1], color = 'black')
plt.axis('equal')


# plot something to show it is working fine. 
begin=0
duration = 100 + begin
step = 100
drawSequence = list(range(begin, duration, step))
connect = [0, 1, 4, 1, 3, 4, 6, 5, 6, 4, 3, 2]

for frame in drawSequence: 
    if frame >= jdxTest1.shape[1]:
        break
    plt.plot(jdxTest1[connect,frame,0], jdxTest1[connect,frame,1])
