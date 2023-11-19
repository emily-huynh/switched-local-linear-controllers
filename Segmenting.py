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

ID_List = [23] #[8,11,12]#[1,2,3,6,7,8,11,12] #,9,10,13,14]

if fullLap: # position based segmenting
    if ID_List[0] == 17:
        secondsToTrim =  4.7
        secondsToTrimEnd = 55
    elif ID_List[0] == 26:
        secondsToTrim = 3.4
        secondsToTrimEnd = 46
    elif ID_List[0] == 19:         
        secondsToTrim = 12
        secondsToTrimEnd = 64
    elif ID_List[0] == 36:         
        secondsToTrim = 1
        secondsToTrimEnd = 40
    elif ID_List[0] == 23:         
        secondsToTrim = 2
        secondsToTrimEnd = 45
    elif ID_List[0] == 12:         
        secondsToTrim = 1
        secondsToTrimEnd = 32
    else:   
        secondsToTrim = 3
        secondsToTrimEnd = 44.4
        
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
postProcess_steering = True



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
    plotMode = False
    
    [All_zeta, xFromCentre, _] = calc_xFromCentreMode(totalTime, All_zeta, ID_List, roadGeometries, plotMode)
    
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

global a, b, c, d
a = 0.60 #1 
b = a * 100 #100 
c = 10 #40
for i, posy in enumerate(y_pos):
    posx = x_pos[i]
    
    if posx <= 100 and (a*posx - b) <= posy <= 300:
        Kgain = "K1"
        segment_straights.append(i)
    elif posy >= 300  and posy >= a*posx + (300-b):
        Kgain = "K2"
        segment_turnsRH.append(i)
    elif posx >= 100 and 0 <= posy <= a*posx + (300-b):
        Kgain = "K3"
        segment_Bstraights.append(i)
    elif posy <= 0 and posy <= (a*posx - b):
        Kgain = "K4"
        segment_BturnsRH.append(i)
        
## Cluster the data using the indices found        
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
        steeringProcessed[i] = -steeringData[i] / 470
    return steeringProcessed

if postProcess_steering:
    straights_mu[:, 1:2] = pProcess_steering(straights_mu)
    turnRH_mu[:, 1:2] = pProcess_steering(turnRH_mu)
    Bstraights_mu[:, 1:2] = pProcess_steering(Bstraights_mu)
    BturnRH_mu[:, 1:2] = pProcess_steering(BturnRH_mu)
    
    
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
    
    printMSG.append([f'   {Kgain} with {cost:.2f}', cost])

    Result.append([thisResult.get('success'),thisResult.get('fun'), thisResult.get('x'), ID_List,
                   [secondsToTrim, secondsToTrimEnd], lengthToTrim,
                   headingAngleMode, xFromCentreMode, initGuess_Ones,postProcess_steering, straightsMode, timeBasedSegment])

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
totalCost = 0
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

for i in printMSG:
    totalCost += i[1]

print('\n'+f'Gain Results for a = {a} \n {printMSG[0][0]} \n {printMSG[1][0]} \n {printMSG[2][0]} \n {printMSG[3][0]} \n Total = {totalCost:0.1f}')
if timeBasedSegment:    
    func = lambda K : costFunc(K, straights_mu, straights_zeta)
                 
    if firstK:
        [Kt1, _] = optimiser(func, kV, ID, Result,'time based segmenting')
    else:
        [Kt2, _] = optimiser(func, kV, ID, Result,'time based segmenting')
#%% Save Results
columnNames = ['Success or Failure', 'Cost', 'K', 'list of CSVs',
               'Time Trim (start, end)', 'Length Trim',
               'headingAngleMode', 'xFromCentreMode', 'kV Ones', 'Pre-process Steering', 'straightsMode', ' sillySegmenting']

Results = pd.DataFrame(np.array(Result), columns = columnNames)
outputFile = 'SimpleSegment_Results99.csv'

Results.to_csv(outputFile, index = True, lineterminator = '\r\n')


#%% Testing the calculated K
delta = np.empty(straights_mu.shape)

testRange = [K1, K2, K3, K4]
nameRange = ['K1', 'K2', 'K3', 'K4']

print(f' Steering setting = {postProcess_steering}')
for j in range(len(testRange)):
    K_name = nameRange[j]
    K = testRange[j]
    
    for i in range(len(straights_mu)):
        mu = straights_mu[i,:]
        zeta = straights_zeta[i,:]
    
        muCalc = np.matmul(K, zeta)
    
        delta[i] = abs(muCalc - mu)

    print(f' For the controller {K_name}',
          f'\n   throttle max: {max(delta[:][0]):.3f} and min {min(delta[:][0]):.3f}',
          f'\n   steering max: {max(delta[:][1]):.3f} and min {min(delta[:][1]):.3f}',
          f'\n   brake max: {max(delta[:][2]):.3f} and min {min(delta[:][2]):.3f}')
