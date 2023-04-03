import math
import os
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
from torchvision import datasets, transforms
from torch.autograd import Variable
from torchvision.utils import save_image
import os
from PIL import Image as PImage
device="cpu"
trainloss=0
testloss=0
def weights_init(m):
    if isinstance(m, nn.Linear):
      torch.nn.init.kaiming_uniform_(m.weight)

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
  
vae = VAE(z_dim=5).double().to(device)
optimizer = optim.Adam(vae.parameters(), lr=0.001)
def loss_function(recon_x, x, mu, logvar):
    myloss=nn.MSELoss()
    MSE=myloss(recon_x,x)
    KLD = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())
    return MSE + KLD


with open('ISEFData.txt', 'r') as f: 
    lines = f.readlines()
epoch=0
trainloss=0
testloss=0
#batch_size will be 128

batch_size=128
count=0
input_descriptors=np.zeros((batch_size,22),dtype='float64')
runningnum=0
trainloss=0
testloss=0
itertrain=0
itertest=0
totaltrainlosstracker=np.zeros(50)
totaltestlosstracker=np.zeros(50)
for epoch in range(50):
    print("Epoch number " + str(epoch))
    count=0
    trainloss=0
    testloss=0
    itertrain=0
    itertest=0
    runningnum=0
    for line in lines:
            runningnum+=1
            count+=1
            x, y = line.split('=')[0], line.split('=')[1]
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
            S=np.zeros(360, dtype='complex_')
            
            #Fourier Descriptor extraction will differ based on the VAE version
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
            #Will vary depending on what VAE architecture is used
            input_list=[float(np.real(S[355])),float(np.real(S[356])),float(np.real(S[357])),float(np.real(S[358])), float(np.real(S[359])), float(np.real(S[0])), float(np.real(S[1])),float(np.real(S[2])),float(np.real(S[3])),float(np.real(S[4])),float(np.real(S[5])), float(np.imag(S[355])),float(np.imag(S[356])),float(np.imag(S[357])), float(np.imag(S[358])), float(np.imag(S[359])), float(np.imag(S[0])), float(np.imag(S[1])), float(np.imag(S[2])), float(np.imag(S[3])),float(np.imag(S[4])),float(np.imag(S[5]))]
            for i in range (len(input_list)):
                input_list[i]=input_list[i]*1000
            input_tensor=torch.tensor(input_list)
            input_descriptors[count-1]=np.asarray(input_tensor)
            if(count==batch_size and runningnum<2554):
                input_descriptors=torch.from_numpy(input_descriptors).to(device)
                recon_batch, mu, logvar = vae(input_descriptors)
                print(input_descriptors)
                print(recon_batch)
                loss = loss_function(recon_batch, input_descriptors, mu, logvar)
                optimizer.zero_grad()
                loss.backward()
                myloss=loss.item()
                trainloss+=myloss
                itertrain+=1
                optimizer.step()
                count=0
                print(myloss)
                input_descriptors=np.zeros((batch_size,22),dtype='float64')
            elif(count==batch_size and runningnum>=2554):
                input_descriptors=torch.from_numpy(input_descriptors).to(device)
                recon_batch, mu, logvar = vae(input_descriptors)
                loss = loss_function(recon_batch, input_descriptors, mu, logvar)
                myloss=loss.item()
                count=0
                input_descriptors=np.zeros((batch_size,22),dtype='float64')
                testloss+=myloss
                itertest+=1
    print("Train loss for epoch number "+str(epoch))
    print(trainloss/itertrain)
    totaltrainlosstracker[epoch]=trainloss/itertrain
    print("Test loss for epoch number " + str(epoch))
    print(testloss/itertrain)
    totaltestlosstracker[epoch]=testloss/itertrain
print(totaltrainlosstracker)
print(totaltestlosstracker)
torch.save(vae.state_dict(), 'D:/abhay/wavelets/ISEF_VAE_Weights/Weights120-22.txt')
