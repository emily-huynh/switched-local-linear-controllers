import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.optimize as opt
import scipy.linalg as linalg
import scipy.interpolate as interpolate
import math
import pickle

exec(open("emFunctions.py").read())
from emFunctions import import_data, calc_XFromCentre, centreLineOfRoad, calc_xFromCentreMode


''' Variables '''
global mu_dim, zeta_dim, K, roadGeometries, pollFrequency_sec
pollFrequency_sec = 0.1
mu_dim = 3; zeta_dim = 5 

firstK = True
fullLap = True

if fullLap: # position based segmenting
    secondsToTrim = 4.7
    secondsToTrimEnd = 55
elif firstK: # time based segmenting 
    secondsToTrim = 3
    secondsToTrimEnd = 30 #22
else:
    secondsToTrim = 22
    secondsToTrimEnd = 30
    
lengthToTrim = 1 # is irrelevant when timeBasedSegment = True
length_actual = 300

headingAngleMode = False
xFromCentreMode = True

initGuess_Ones = False
straightsMode = True
timeBasedSegment = False 
postProcess_steering = False


ID_List = [17] #[8,11,12]#[1,2,3,6,7,8,11,12] #,9,10,13,14]

if headingAngleMode:
    zeta_dim = 4

''' 
    Import Data
'''

# Import the data to be segmented
All_mu = np.ones((1,mu_dim+1))
All_zeta = np.ones((1,zeta_dim+1))

for ID in ID_List:  
    inputFile = f'Saves/straightsAndcircles_{ID}.csv'
    [mu, zeta, _, _, _, _, _, y_pos, _, _, _, _, _] = import_data(inputFile, headingAngleMode)
        # Trimming out first Ns of data
    indexOfTrim =  int(secondsToTrim/pollFrequency_sec)
        # Trim end data
    indexOfEndTrim = int(secondsToTrimEnd/pollFrequency_sec)
        #
    mu = mu[indexOfTrim:indexOfEndTrim,:]
    zeta = zeta[indexOfTrim:indexOfEndTrim,:]
    
    # Add to stack
    All_mu = np.row_stack((All_mu, mu))
    All_zeta = np.row_stack((All_zeta, zeta))

# Cutting out the dummy row of 1s for array creation
# as well as the 'timestamp' for each row
All_mu = All_mu[1:,1:]
All_zeta = All_zeta[1:,1:]

if timeBasedSegment == False: # proper segmenting
    y_pos = All_zeta[:,1]
    x_pos = All_zeta[:,0]

#% Import Road Dictionary for xFromCentreMode
filePath = 'straightsAndcircles_roadGeometries.pkl'
with open (filePath, 'rb') as fp:
    roadGeometries = pickle.load(fp)

if xFromCentreMode:
    totalTime = len(All_mu)
    plotMode = True
    
    [All_zeta, xFromCentre, _] = calc_xFromCentreMode(totalTime, All_zeta, ID_List, plotMode)
    
'''
 Simplest Segment:  
 Only straights and RH turns.
'''
length = lengthToTrim * length_actual

segment_straights = list()
segment_turnsRH = list()
segment_Bstraights = list()
segment_BturnsRH = list()

## Indexing: Finding the timestamps which are within the bounds
# for indexing later using the trimmed data
if timeBasedSegment:
    y_pos = All_zeta[:,1]
    x_pos = y_pos 

global c
for i, posy in enumerate(y_pos):
    posx = x_pos[i]
    
    if posx <= 25 and -75 <= posy <= 300:
        Kgain = "K1"
        segment_straights.append(i)
    elif posy >= 300 and posy >= posx + 191.144 - c:
        Kgain = "K2"
        segment_turnsRH.append(i)
    elif posx >= 175 and 0 <= posy <= 366.144 - c:
        Kgain = "K3"
        segment_Bstraights.append(i)
    elif posy <= 0 and posy <= posx - 91.144:
        Kgain = "K4"
        segment_BturnsRH.append(i)
        
## Cluster the data using the timestamps found        
# reminder: Data has been preprocessed to remove the timestamp
straights_mu = np.array(All_mu)[segment_straights, :]
straights_zeta = np.array(All_zeta)[segment_straights, :] 

turnRH_mu = np.array(All_mu)[segment_turnsRH, :]
turnRH_zeta = np.array(All_zeta)[segment_turnsRH, :] 

Bstraights_mu = np.array(All_mu)[segment_Bstraights, :]
Bstraights_zeta = np.array(All_zeta)[segment_Bstraights, :] 

BturnRH_mu = np.array(All_mu)[segment_BturnsRH, :]
BturnRH_zeta = np.array(All_zeta)[segment_BturnsRH, :] 
   
# Force K1 to be 0s to 30s instead of the location based straights
if False:
    index = int(30/pollFrequency_sec) - int(4.7/pollFrequency_sec) - 1
    straights_mu = np.array(All_mu)[:index, :]
    straights_zeta = np.array(All_zeta)[:index, :]

''' 
    Preprocessing of Data  
'''
 # Steering -> [-470, 470]
def pProcess_steering(mu):
    steeringData = mu[:,1]
    steeringProcessed = np.empty((len(mu), 1))
    for i in range(len(steeringData)):
        steeringProcessed[i] = steeringData[i] / 470
    return steeringProcessed

if postProcess_steering:
    straights_mu[:, 1:2] = pProcess_steering(straights_mu[:,1])
    turnRH_mu[:, 1:2] = pProcess_steering(turnRH_mu[:,1])
    Bstraights_mu[:, 1:2] = pProcess_steering(Bstraights_mu[:,1])
    BturnRH_mu[:, 1:2] = pProcess_steering(BturnRH_mu[:,1])
    
# steeringData_straight = straights_mu[:,1]
# steeringData_turnRH = turnRH_mu[:,1]
# steeringProcessed = np.empty((len(mu), 1))
# j = 0 
#   # scale the data by the maximum value, squashing the bounds to [-1, 1]
# for i in range(len(steeringData)):
#     steeringProcessed[i] = steeringData[i] / 470
#   # replace observed data with processed data
# if postProcess_steering:
#     straights_mu[:,1:2] = steeringProcessed

''' Optimisation '''
def costFunc(Kv, mu, zeta):
    if len(mu) != len(zeta):
        return print("mu and zeta are not the same length!?")
    
    K = np.reshape(Kv, [mu_dim,zeta_dim])
    cost = 0
    
    for t in range(len(zeta)):
        #A = np.reshape(mu[t,:], [3,1])
        #B = np.reshape(zeta[t,:], [4,1])
        inside = mu[t,:] - np.matmul(K, zeta[t,:])
        cost += np.linalg.norm(inside)**2
    
    return cost 

def optimiser(func, kV, ID, Result, printMSG, titleMSG):
    thisResult = opt.minimize(func, kV, method = 'SLSQP')
    success = thisResult.get('success')
    cost = thisResult.get('fun')
    
    print('\n')
    print(f'Optimising for {titleMSG}', f'\n   using ID {ID}')       
    print(f' success: {success} \n with cost = {cost:.2f} \n using initial guess of {int(kV[0][0])}s',
          # f'\n bearing mode: {headingAngleMode} \n distance from centreline mode: {xFromCentreMode}',
          f'\n trimmed (seconds): {secondsToTrim}s and {secondsToTrimEnd}s \n trimmed (length): {lengthToTrim}'
          )
    
    printMSG.append(f'   {Kgain} with {cost:.2f}')

    Result.append([thisResult.get('success'),thisResult.get('fun'), thisResult.get('x'), ID_List,
                   [secondsToTrim, secondsToTrimEnd], lengthToTrim,
                   headingAngleMode, xFromCentreMode, initGuess_Ones, straightsMode, timeBasedSegment])

    return np.reshape(thisResult.get('x'), [mu_dim,zeta_dim]), Result, printMSG

# Settings
if xFromCentreMode and headingAngleMode:
    zeta_dim = 3
elif xFromCentreMode:
    zeta_dim = 4    
if initGuess_Ones:   
    kV = np.ones([1,mu_dim*zeta_dim])
else:
    kV = np.zeros([1,mu_dim*zeta_dim])
if 'Result' not in globals():
        Result = list()
printMSG = []
''' 
    Compute K gain matrixes 
'''        
funcStraight = lambda K : costFunc(K, straights_mu, straights_zeta)
funcTurnRH = lambda K : costFunc(K, turnRH_mu, turnRH_zeta) 
funcBStraight = lambda K : costFunc(K, Bstraights_mu, Bstraights_zeta)
funcBTurnRH = lambda K : costFunc(K, BturnRH_mu, BturnRH_zeta)      

[K1, _, printMSG] = optimiser(funcStraight, kV, ID, Result, printMSG, 'Straights'); Kgain = 'K2'
[K2, _, printMSG] = optimiser(funcTurnRH, kV, ID, Result, printMSG, 'RH turn'); Kgain = 'K3'
[K3, _, printMSG] = optimiser(funcBStraight, kV, ID, Result, printMSG, 'Back Straight'); Kgain = 'K4'
[K4, _, printMSG] = optimiser(funcBTurnRH, kV, ID, Result, printMSG, '2nd RH turn')

print('\n'+f'Gain Results \n {printMSG[0]} \n {printMSG[1]} \n {printMSG[2]} \n {printMSG[3]}')
if timeBasedSegment:    
    func = lambda K : costFunc(K, straights_mu, straights_zeta)
                 
    if firstK:
        [Kt1, _] = optimiser(func, kV, ID, Result,'time based segmenting')
    else:
        [Kt2, _] = optimiser(func, kV, ID, Result,'time based segmenting')
#%% Save Results
columnNames = ['Success or Failure', 'Cost', 'K', 'list of CSVs',
               'Time Trim (start, end)', 'Length Trim',
               'headingAngleMode', 'xFromCentreMode', 'kV Ones', 'straightsMode', ' sillySegmenting']

Results = pd.DataFrame(np.array(Result), columns = columnNames)
outputFile = 'SimpleSegment_Results3.csv'

Results.to_csv(outputFile, index = True, lineterminator = '\r\n')


#%% Testing the calculated K
delta = np.empty(straights_mu.shape)

for i in range(len(straights_mu)):
    mu = straights_mu[i,:]
    zeta = straights_zeta[i,:]

    muCalc = np.matmul(K, zeta)

    delta[i] = abs(muCalc - mu)
    delta[i][1] = delta[i][1]/180

print(f' throttle max: {max(delta[:][0]):.3f} and min {min(delta[:][0]):.3f}',
      f'\n steering max: {max(delta[:][1]):.3f} and min {min(delta[:][1]):.3f}',
      f'\n brake max: {max(delta[:][2]):.3f} and min {min(delta[:][2]):.3f}')








