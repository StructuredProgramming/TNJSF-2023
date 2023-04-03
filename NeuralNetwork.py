from __future__ import print_function
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from skimage.io import imread
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import torch
import torch.utils.data
from torch import nn, optim
from torch.autograd import Variable
from torch.nn import functional as F
from torchvision import datasets, transforms
from torchvision.utils import save_image
from PIL import Image
import os
import ast
import numpy as np
import pywt
import random
import matplotlib.pyplot as plt
import scipy.spatial.distance as sciDist
import copy
import itertools
import numpy as np
import sys
import time
import math
from itertools import combinations
from PIL import Image as PImage
np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)
device="cuda"
def calculateloss(a,b):
    return nn.MSELoss(a,b)
def is_simple(tpMat):
    # simple: i.e., this can be solved with chain tree and intersection of circles/arc sects.
    # find links and set up joint table.
    # actuator will be noted with negative value.
    fixParam = [1, 3]
    jT = {}
    fixJ = []
    kkcJ = []
    chain = {}

    # step 1, initialize, set all joints and links to unknown (0 in jointTable) and jointLinkTable.
    for i in range(tpMat.shape[0]):
        jT[i] = 0
        chain[i] = {'from': None, 'next': []}

    # step 2, set all ground joints to known (1 to be known)
    for i in range(tpMat.shape[0]):
        if tpMat[i, i] in fixParam:
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
                if tpMat[i, j] < 0 and jT[j] == 0:
                    jT[j] = 1
                    newJ.append(j)
                    kkcJ.append((j, 'chain', i))
                    chain[i]['next'].append(j)
                    chain[j] = {'from': i, 'next': []}

        if len(kkcJ) == prevCtr:
            break
        else:
            pivotJ = newJ  # This is based on the idea of tree node expansion

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
                        if i < j and tpMat[i, k] * tpMat[j, k] != 0 and not foundNew:
                            foundNew = True
                            jT[k] = 1
                            kkcJ.append((k, 'arcSect', (i, j)))
        if not foundNew:
            break

    # return chain and isSimple (meaning you can solve this with direct chain)
    return kkcJ, chain, len(kkcJ) == tpMat.shape[0]


# Direct kinematics:
def compute_chain_by_step(step, rMat, pos_init, unitConvert=np.pi / 180):
    pos_new = copy.copy(pos_init)
    dest, _, root = step
    pos_new[dest, 2] = rMat[root, dest] * unitConvert + pos_new[root, 2]
    c = np.cos(pos_new[dest, 2])
    s = np.sin(pos_new[dest, 2])
    posVect = pos_init[dest, 0:2] - pos_init[root, 0:2]
    pos_new[dest, 0] = posVect[0] * c - posVect[1] * s + pos_new[root, 0]
    pos_new[dest, 1] = posVect[0] * s + posVect[1] * c + pos_new[root, 1]
    return pos_new


# Inverse kinematics:
def compute_arc_sect_by_step(step, posOld, distMat, Ppp=None, threshold=0.1, timefactor=0.1):
    global is_impossible

    threshold = np.max(distMat) * threshold
    posNew = copy.copy(posOld)
    ptSect, _, centers = step
    cntr1, cntr2 = centers
    r1s = distMat[cntr1, ptSect]
    r2s = distMat[cntr2, ptSect]
    if r1s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr1, 0:2]
    elif r2s < 10e-12:
        posNew[ptSect, 0:2] = posOld[cntr2, 0:2]
    else:
        ptOld = posOld[ptSect, 0:2]
        ptCen1 = posOld[cntr1, 0:2]
        ptCen2 = posOld[cntr2, 0:2]
        d12 = np.linalg.norm(ptCen1 - ptCen2)
        if d12 > r1s + r2s or d12 < np.absolute(r1s - r2s):
            # print('impossible \n')
            return posOld, False
        elif d12 < 10e-12:  # incidence joint
            # print('illegal \n')
            return posOld, False
        else:
            # print('legal')
            # a means the LENGTH from cntr1 to the mid point between two intersection points.
            # h means the LENGTH from the mid point to either of the two intersection points.
            # v means the Vector from cntr1 to the mid point between two intersection points.
            # vT 90 deg rotation of v
            a = (r1s ** 2 - r2s ** 2 + d12 ** 2) / (d12 * 2)
            h = np.sqrt(r1s ** 2 - a ** 2)
            v = ptCen2 - ptCen1
            vT = np.array([-v[1], v[0]])
            r1 = a / d12
            r2 = h / d12
            ptMid = ptCen1 + v * r1
            sol1 = ptMid + vT * r2
            sol2 = ptMid - vT * r2
            # print(ptOld, sol1, np.linalg.norm(sol1 - ptOld), sol2, np.linalg.norm(sol2 - ptOld))
            # compute ref point
            refPoint = ptOld
            if type(Ppp) != type(None):
                refPoint += (Ppp[ptSect, 0:2] - ptOld) * timefactor
            if np.linalg.norm(sol1 - refPoint) > np.linalg.norm(sol2 - refPoint):
                posNew[ptSect, 0:2] = sol2
                # print('sol2 selected \n')
            else:
                posNew[ptSect, 0:2] = sol1
            # detect if there's an abrupt change:
            if np.max(np.linalg.norm(posNew - posOld, axis=1)) > threshold:
                # print('thresholded', posNew, posOld)
                return posOld, False

        return posNew, True


# Basic data for computing a mechanism.
def compute_dist_mat(tpMat, pos):
    cdist = sciDist.cdist
    tpMat = copy.copy(np.absolute(tpMat))
    tpMat[list(range(0, tpMat.shape[0])), list(range(0, tpMat.shape[1]))] = 0
    return np.multiply(cdist(pos[:, 0:2], pos[:, 0:2]), tpMat)


def compute_curve_simple(tpMat, pos_init, rMat, distMat=None, maxTicks=360, baseSpeed=1):
    # preps
    kkcJ, chain, isReallySimple = is_simple(tpMat)
    if distMat is None:
        distMat = compute_dist_mat(tpMat, pos_init)
    poses1 = np.zeros((pos_init.shape[0], maxTicks, 3))
    poses2 = np.zeros((pos_init.shape[0], maxTicks, 3))
    # Set first tick
    poses1[:, 0, 0:pos_init.shape[1]] = pos_init
    poses2[:, 0, 0:pos_init.shape[1]] = pos_init
    # Compute others by step.
    meetAnEnd = False
    meetTwoEnds = False
    tick = 0
    offset = 0
    while not meetTwoEnds:
        # get tick
        tick += 1
        if tick + offset >= maxTicks:
            poses = poses1  # never flips
            break
        # decide which direction to compute.
        if not meetAnEnd:
            time = 1 * baseSpeed
            pos = poses1[:, tick - 1, :]
            posp = poses1[:, tick - 1, :]
            if tick - 2 < 0:
                Ppp = None
            else:
                Ppp = poses1[:, tick - 2, :]
        else:
            time = 1 * baseSpeed * (-1)
            pos = poses2[:, tick - 1, :]
            posp = poses2[:, tick - 1, :]
            if tick - 2 < 0:
                Ppp = None
            else:
                Ppp = poses2[:, tick - 2, :]
        # step-wise switch solution
        for step in kkcJ:
            if step[1] == 'fixed':
                pos[step[0], 0:2] = pos_init[step[0], :]
                notMeetEnd = True
            elif step[1] == 'chain':
                pos = compute_chain_by_step(step, rMat * time, posp[:, :])
                notMeetEnd = True
            elif step[1] == 'arcSect':
                if not meetAnEnd:
                    pos[step[0], :] = poses1[step[0], tick - 1, :]
                else:
                    pos[step[0], :] = poses2[step[0], tick - 1, :]
                pos, notMeetEnd = compute_arc_sect_by_step(step, pos, distMat, Ppp)
                if notMeetEnd and not meetAnEnd:  # never met an end -> to poses1
                    poses1[:, tick, :] = pos
                elif not notMeetEnd and not meetAnEnd:  # meet end like right now. This tick is not a solution.
                    poses1 = poses1[:, 0:tick, :]
                    offset = tick - 1  # the number of valid ticks
                    meetAnEnd = True
                    tick = 0  # reset tick to zero for time pos
                    break
                elif notMeetEnd and meetAnEnd:  # met an end. -> to poses2
                    poses2[:, tick, :] = pos
                else:  # not notMeetEnd and meetAnEnd. met both ends right now, this tick is not a solution.
                    poses2 = poses2[:, 1:tick, :]  # poses2 (<-) poses1(->). First pose of poses2 is pos_init.
                    poses2 = np.flip(poses2, axis=1)  # make poses2 (->)
                    poses = np.concatenate([poses2, poses1], axis=1)
                    meetTwoEnds = True
                    break
            else:
                print('Unexpected step:, ' + step[1])
                break
    return poses, meetAnEnd, isReallySimple


def get_pca_inclination(qx, qy):
    cx = np.mean(qx)
    cy = np.mean(qy)
    inf = False
    covar_xx = np.sum((qx - cx) * (qx - cx)) / len(qx)
    covar_xy = np.sum((qx - cx) * (qy - cy)) / len(qx)
    covar_yx = np.sum((qy - cy) * (qx - cx)) / len(qx)
    covar_yy = np.sum((qy - cy) * (qy - cy)) / len(qx)
    if np.isnan(covar_xx) or np.isnan(covar_yy) or np.isnan(covar_yx) or np.isnan(covar_xy)\
            or np.isinf(covar_xx) or np.isinf(covar_yy) or np.isinf(covar_yx) or np.isinf(covar_xy):
        inf = True
        phi = 0
    else:
        covar = np.array([[covar_xx, covar_xy], [covar_yx, covar_yy]])

        eig_val, eig_vec = np.linalg.eig(covar)

        # Inclination of major principal axis w.r.t. x axis
        if eig_val[0] > eig_val[1]:
            phi = np.arctan2(eig_vec[1, 0], eig_vec[0, 0])
        else:
            phi = np.arctan2(eig_vec[1, 1], eig_vec[0, 1])

    return phi, inf


def rotate_curve(x, y, theta):
    cpx = x * np.cos(theta) - y * np.sin(theta)
    cpy = x * np.sin(theta) + y * np.cos(theta)
    return cpx, cpy
start = time.time()

def weights_init(m):
    if isinstance(m, nn.Linear):
      torch.nn.init.kaiming_uniform_(m.weight)
    
#Plan to run twelve versions and examine the loss: 22->720, 22->360, 22->180, 22->90, 36->720, 36->360, 36->180, 36->90, 60->720, 60->360, 60->180, 60->90, this will determine which weights will be used for the Neural Network
class VAE(nn.Module):
    def __init__(self, z_dim):
        super(VAE, self).__init__()

        # Encoder
        self.encode=nn.Sequential(
        nn.Linear(22,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120)
        )
        self.fc_mu = nn.Linear(120, z_dim)
        self.fc_logvar = nn.Linear(120, z_dim)
        # Decoder
        self.decode=nn.Sequential(
        nn.Linear(z_dim, 120),
        nn.ReLU(),
        nn.Linear(120, 120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,120),
        nn.ReLU(),
        nn.Linear(120,22)
        )
        nn.init.kaiming_uniform_(self.fc_mu.weight)
        nn.init.kaiming_uniform_(self.fc_logvar.weight)
        self.encode.apply(weights_init)
        self.decode.apply(weights_init)

        
    def encoder(self, x):
        a=self.encode(x)
        return self.fc_mu(a),self.fc_logvar(a)     

    def sampling(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return eps.mul(std).add_(mu) # return z sample

    def decoder(self, z):
        return self.decode(z)
       
    def forward(self, x):
        mu, logvar = self.encoder(x)
        z = self.sampling(mu, logvar)
        return self.decoder(z), mu, logvar
#Will test 9 different values: 3000, 1536, 1000, 768, 500, 384, 192, 120, 96
class NeuralNetwork(nn.Module):
    def __init__(self, hidden_layer,latentdimensions):
        super(NeuralNetwork,self).__init__()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(latentdimensions,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,hidden_layer),
            nn.ReLU(),
            nn.Linear(hidden_layer,8)
            )            
    def forward(self,x):
        output=self.linear_relu_stack(x)
        return output
        
def coords(m):
    for i in range(2, len(m) - 3):
        if m[i] == ',':
            return m[2:i],m[(i+2):(len(m)-2)]
trainloss=0
testloss=0
vae = VAE(z_dim=5).double().to(device)
myfinaltrainloss=[]
myfinaltestloss=[]
vae.load_state_dict(torch.load("Weights120-22.txt", map_location=torch.device('cpu')))
loss_function2=nn.MSELoss()
model=NeuralNetwork(hidden_layer=1280, latentdimensions=5).double().to(device)   
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
batch_size=128
with open('file_new.txt', 'r') as f: 
    lines = f.readlines()
completeloss=0
for epoch in range (200):
    validcurves=0
    print("Epoch number " + str(epoch))
    count=0
    trainloss=0
    testloss=0
    itertrain=0
    itertest=0
    runningnum=0
    #random.shuffle(lines)
    total360pointloss=0
    numiterations=0
    epochloss=0
    for line in lines:
        numiterations+=1
        if(numiterations%1000==0):
            print("Coupler curve number "+str(numiterations))
        count+=1
        x, y = line.split('=')[0], line.split('=')[1]
        w=line.split('=')
        if len(w)==6:
            movingjoint1=w[2]
            movingjoint2=w[3]
            movingjoint3=w[4]
            movingjoint4=w[5]
            x1,y1=coords(movingjoint1)
            x2,y2=coords(movingjoint2)
            x3,y3=coords(movingjoint3)
            x4,y4=coords(movingjoint4)
        else:
            movingjoint1=w[3]
            movingjoint2=w[5]
            movingjoint3=w[6]
            movingjoint4=(0,0)
            x1,y1=coords(movingjoint1)
            x2,y2=coords(movingjoint2)
            x3,y3=coords(movingjoint3)
            x4=0
            y4=0
        x, y = x.split(' '), y.split(' ')
        x = [i for i in x if i]
        y = [i for i in y if i]
        x[0] = x[0][1:]
        y[0] = y[0][1:]
        x[-1] = x[-1][:-1]
        y[-1] = y[-1][:-1]

        x = [float(i) for i in x if i]
        y = [float(i) for i in y if i]
        if(math.isnan(x[30])):
                continue
        x=torch.tensor(x)
        y=torch.tensor(y)
        #Fourier Descriptor extraction will depend on which VAE ends up being the best (the number of inputs needed could vary as a result)
        S=np.zeros(360, dtype='complex_')
        i=0
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=359
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=1
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=358
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=2
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=357
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=3
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=356
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=4
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=355
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        i=5
        for k in range(360):
            a=x[k]
            b=y[k]
            tmp = ((-2j*np.pi*i*k)) /360
            S[i] += (complex(a,b)) * np.exp(tmp)
        S[i]=S[i]/360
        #Input representation will differ based on VAE used
        input_list=[float(np.real(S[355])),float(np.real(S[356])),float(np.real(S[357])),float(np.real(S[358])), float(np.real(S[359])), float(np.real(S[0])), float(np.real(S[1])),float(np.real(S[2])),float(np.real(S[3])),float(np.real(S[4])),float(np.real(S[5])), float(np.imag(S[355])),float(np.imag(S[356])),float(np.imag(S[357])), float(np.imag(S[358])), float(np.imag(S[359])), float(np.imag(S[0])), float(np.imag(S[1])), float(np.imag(S[2])), float(np.imag(S[3])),float(np.imag(S[4])),float(np.imag(S[5]))]
        input_tensor=torch.tensor(input_list).double().to(device)
        latent_vector=vae.encoder(input_tensor)
        prediction=model(latent_vector[0])
        #print(latent_vector[0])
        #print(prediction)
        output_list=[float(x1),float(x2),float(x3),float(x4),float(y1),float(y2),float(y3),float(y4)]
        output_tensor=torch.tensor(output_list).double().to(device)
        loss_function=nn.MSELoss()
        loss=loss_function(prediction,output_tensor)
        #print(prediction)
        #print(output_tensor)
        runningnum+=1
        #print(total360pointloss)
        #if(runningnum%100==0 and runningnum>0):
         #   print(runningnum)
          #  print(total360pointloss)
        if(runningnum<40000):
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            myloss=loss.item()
            if(count%100==0):
                count=0
                #print(myloss)
            itertrain+=1
            trainloss+=loss
            #print(prediction)
            #print(output_tensor)
        elif(runningnum>=40000):
            count=0
            testloss+=loss
            itertest+=1
        if(len(w)==6):
            #print("Entered")
            predictions=prediction.cpu().detach().numpy()
            j_0=(0,0)
            j_1=(predictions[0],predictions[4])
            j_2=(predictions[1],predictions[5])
            j_3=(1,0)
            j_4=(predictions[2],predictions[6])
            j_5=(predictions[3],predictions[7])
            j_6=(1.5,0)
            
            l_0 = math.dist(np.array(j_3),np.array(j_0))
            l_1 = math.dist(np.array(j_1),np.array(j_0))
            l_2 = math.dist(np.array(j_2),np.array(j_1))
            l_3 = math.dist(np.array(j_3),np.array(j_2))
            total = l_0 + l_1 + l_2 + l_3
            shortest = min(l_0, l_1, l_2, l_3)
            longest = max(l_0, l_1, l_2, l_3)
            pq = total - shortest - longest
            if shortest + longest <= pq:
                if min(l_0, l_1, l_2, l_3) == l_0 or min(l_0, l_1, l_2, l_3) == l_2:
                    continue

                elif min(l_0, l_1, l_2, l_3) == l_1:
                    tp_steph_3 = np.matrix([
                [1, -1, 0, 1, 0, 0, 1],
                [1, 2, 1, 0, 1, 0, 0],
                [0, 1, 2, 1, 1, 0, 0],
                [1, 0, 1, 1, 0, 0, 1],
                [0, 1, 1, 0, 2, 1, 0],
                [0, 0, 0, 0, 1, 2, 1],
                [1, 0, 0, 1, 0, 1, 1]
            ])

                    rMatTest = np.array([[0, 1.0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0]])
                else:
                    tp_steph_3 = np.matrix([
                [1, 1, 0, 1, 0, 0, 1],
                [1, 2, 1, 0, 1, 0, 0],
                [0, 1, 2, 1, 1, 0, 0],
                [1, 0, -1, 1, 0, 0, 1],
                [0, 1, 1, 0, 2, 1, 0],
                [0, 0, 0, 0, 1, 2, 1],
                [1, 0, 0, 1, 0, 1, 1]
            ])

                    rMatTest = np.array([[0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 1.0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0]])

                posInit = np.array([j_0, j_1, j_3, j_2, j_4, j_6, j_5])
                jdxTest1, _, _ = compute_curve_simple(tp_steph_3, posInit, rMatTest)

                if len(jdxTest1[4, :, 0]) < 360:
                    continue

                points_x, points_y = np.asarray(jdxTest1[4, :, 0]), np.asarray(jdxTest1[4, :, 1])

                points_x, points_y = np.subtract(points_x, np.mean(points_x)), np.subtract(points_y, np.mean(points_y))

                points_x, points_y = np.divide(points_x, np.sqrt(np.var(points_x))), np.divide(points_y, np.sqrt(np.var(
                    points_y)))

                theta, is_inf = get_pca_inclination(np.asarray(points_x), np.asarray(points_y))

                if not is_inf:
                    validcurves+=1
                    points_x, points_y = rotate_curve(points_x, points_y, -theta)
                    #x=torch.tensor(x)
                    points_x=torch.tensor(points_x)
                    #y=torch.tensor(y)
                    points_y=torch.tensor(points_y)
                    runningloss1=loss_function(points_x,x) 
                    runningloss2=loss_function(points_y,y)
                    #print(runningloss1)
                    #print(runningloss2)
                    total360pointloss=(runningloss1+runningloss2)/2
                    epochloss+=total360pointloss
                    #print(total360pointloss)
                    #print(total360pointloss)
                   # if(validcurves%100==0):
                       # print("Total number of valid curves is "+str(validcurves)+" but total number of proposed curves is "+str(runningnum))
                   # if (runningnum%100==0):
                    #    print(runningnum)
                     #   print(total360pointloss)
                    #if(validcurves%100==0):
                     #   print("Total curves seen was "+str(runningnum)+" but total number of valid curves was "+str(validcurves))
                      #  print(total360pointloss)
                       # plt.plot(np.r_[points_x, points_x[0]], np.r_[points_y, points_y[0]], color='red')
                        #plt.plot(np.r_[x, x[0]], np.r_[y, y[0]], color='blue')
                        #plt.show()
        else:  
          #print("Entered 2")
          #print(len(w))
          predictions=prediction.cpu().detach().numpy()
          j_0=(0,0)
          j_1=(predictions[0],predictions[1])
          j_2=(1,0)
          j_3=(predictions[2],predictions[3])
          j_4=(predictions[4],predictions[5])
          l_1 = np.linalg.norm(np.array(j_1) - np.array(j_0))
          l_2 = np.linalg.norm(np.array(j_2) - np.array(j_1))
          l_3 = np.linalg.norm(np.array(j_3) - np.array(j_2))

          total = 1 + l_1 + l_2 + l_3

          shortest = min(1, l_1, l_2, l_3)
          longest = max(1, l_1, l_2, l_3)
          pq = total - shortest - longest

          if shortest + longest <= pq:
            l_1 = np.linalg.norm(np.array(j_1) - np.array(j_0))
            l_2 = np.linalg.norm(np.array(j_2) - np.array(j_1))
            l_3 = np.linalg.norm(np.array(j_3) - np.array(j_2))
    
            if min(1, l_1, l_2, l_3) == 1 or min(1, l_1, l_2, l_3) == l_2:
              continue

            elif min(1, l_1, l_2, l_3) == l_1:
              tpTest = np.matrix([
            [1, -1, 1, 0, 0],
            [1, 2, 0, 1, 1],
            [1, 0, 1, 1, 0],
            [0, 1, 1, 2, 1],
            [0, 1, 0, 1, 2]
        ])

              rMatTest = np.array([[0, 1.0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0]])
            else:
              tpTest = np.matrix([
            [1, 1, 1, 0, 0],
            [1, 2, 0, 1, 1],
            [1, 0, 1, -1, 0],
            [0, 1, 1, 2, 1],
            [0, 1, 0, 1, 2]
        ])

              rMatTest = np.array([[0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 1.0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0]])
            points_x, points_y = 0, 0

            posInit = np.array([j_0, j_1, j_3, j_2, j_4])
            jdxTest1, _, _ = compute_curve_simple(tpTest, posInit,rMatTest)

            if len(jdxTest1[4, :, 0]) < 360:
               continue

            points_x, points_y = np.asarray(jdxTest1[4, :, 0], dtype=np.float32), np.asarray(jdxTest1[4, :, 1])

            points_x, points_y = np.subtract(points_x, np.mean(points_x)), np.subtract(points_y, np.mean(points_y))

            points_x, points_y = np.divide(points_x, np.sqrt(np.var(points_x))), np.divide(points_y, np.sqrt(np.var(
                points_y)))
            points_x=torch.tensor(points_x)
            points_y=torch.tensor(points_y)
          #x=torch.tensor(x)
          #y=torch.tensor(y)
            runningloss1=loss_function2(x,points_x)
            runningloss2=loss_function2(y,points_y)
            total360pointloss=(runningloss1+runningloss2)/2
            epochloss+=total360pointloss
          #print(total360pointloss)
            #if(runningnum%100==0):
             #   print("Outputting here")
              #  print(total360pointloss)
               # print(runningnum)
                #plt.plot(points_x, points_y, color='blue')
                #plt.plot(x,y,color='red')
                #plt.axis('equal')
                #plt.show()
   # myfinaltrainloss.append(trainloss/itertrain)
   # myfinaltestloss.append(testloss/itertrain)
    if(epoch>20):
      completeloss+=epochloss/numiterations
    print(epochloss/numiterations)
print("Final average loss is "+str(completeloss/180))
#print(myfinaltrainloss)
#print(myfinaltestloss)
