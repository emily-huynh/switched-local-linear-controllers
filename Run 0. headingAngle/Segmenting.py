import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.optimize as opt
import scipy.linalg as linalg
import math

exec(open("emFunctions.py").read())

# Import the data to be segmented
All_mu = np.ones((1,4))
All_zeta = np.ones((1,5))

ID_List = [1,2,3,6,7,8,11,12,9,10,13,14]
for ID in ID_List:  
    inputFile = f'Saves/straightsAndcircles_{ID}.csv'
    [mu, zeta, _, _, _, _, _, y_pos, _, _, _, _, _] = import_data(inputFile, headingAngleMode = True)
    # Trimming out first 5s of data
    indexOfTrim =  int(5/pollFrequency_sec)
    mu = mu[indexOfTrim-1:,:]
    zeta = zeta[indexOfTrim-1:,:]
    # Add to stack
    All_mu = np.row_stack((All_mu, mu))
    All_zeta = np.row_stack((All_zeta, zeta))

# Cutting out the dummy row of 1s for array creation
# as well as the 'timestamp' for each row
All_mu = All_mu[1:,1:]
All_zeta = All_zeta[1:,1:]

#%
'''
 Simplest Segment:  
 Only straights and RH turns.
'''

# from setup_straightsAndCircles import length
length_actual = 300
length = 0.4 * length_actual
from emFunctions import import_data

segment_straights = list()
segment_turnsRH = list()

## Indexing: Finding the timestamps which are within the bounds
y_pos = All_zeta[:,1]
for index, j in enumerate(y_pos):
    if j > 0 and j < length:
        segment_straights.append(index)        
    else: 
        segment_turnsRH.append(index)
        
## Cluster the data using the timestamps found        
# Data has been pretreated to remove the timestamp
straights_mu = np.array(All_mu)[segment_straights, :]
straights_zeta = np.array(All_zeta)[segment_straights, :] 

turnRH_mu = np.array(All_mu)[segment_turnsRH, :]
turnRH_zeta = np.array(All_zeta)[segment_turnsRH, :] 

# Generalised Optimisation
def costFunc(Kv, mu, zeta):
    if len(mu) != len(zeta):
        return print("mu and zeta are not the same length!?")
    
    K = np.reshape(Kv, [3,4])
    cost = 0
    
    for t in range(len(zeta)):
        #A = np.reshape(mu[t,:], [3,1])
        #B = np.reshape(zeta[t,:], [4,1])
        inside = mu[t,:] - np.matmul(K, zeta[t,:])
        cost += np.linalg.norm(inside)**2
    
    return cost 

kV = np.zeros([1,12])
func = lambda K : costFunc(K, straights_mu, straights_zeta)
thisResult = opt.minimize(func, kV, method = 'Nelder-Mead')
success = thisResult.get('success')
cost = thisResult.get('fun')
print('\n')
print(f'success: {success} \n with cost = {cost} \n using initial guess of {kV}')

if 'Result' not in globals():
    Result = list()
Result.append([thisResult.get('success'),thisResult.get('fun'), thisResult.get('x'), kV, ID_List])

K = np.reshape(thisResult.get('x'), [3,4])
#%% Save Results
columnNames = ['Success or Failure', 'Cost', 'K', 'k0', 'list of CSVs']

Results = pd.DataFrame(np.array(Result), columns = columnNames)
outputFile = 'SimpleSegment_Results.csv'

Results.to_csv(outputFile, index = True, lineterminator = '\r\n')
#%% Obtained K
K = np.reshape(thisResult.get('x'), [3,3])

states = zeta[0,1:-1]

for t in range(t_sec):
      inputs = np.matmul(K, states)
      
      throttle = inputs[0]
      steering = inputs[1]
      brake = inputs[2]
      
      vehicle.



    






#%% Testing: Inside function 
K = np.ones([3, 3], dtype = 'f')
inside = straights_mu[10,:] - np.matmul(K, straights_zeta[10,:])
np.linalg.norm(inside)**2 

#%% Attempt 1: Get the gains (K)
    ## Test on one timestamp
tmu =  np.reshape(np.array(straights_mu[10], dtype='f'), [3,1])
tzeta = np.reshape(np.array(straights_zeta[10], dtype='f'), [3,1])

tK = np.dot(tmu, np.linalg.pinv(tzeta))
tmu == np.matmul(tK,tzeta)
    ##
    
    # K_Straight Gain

K_straights = np.zeros([(3*len(straights_mu)),3])
for i in range(len(straights_mu)):
    tmu =  np.reshape(np.array(straights_mu[i], dtype='f'), [3,1])
    tzeta = np.reshape(np.array(straights_zeta[i], dtype='f'), [3,1])
    tK = np.dot(tmu, np.linalg.pinv(tzeta))
    
    if i == 0:
        K_straights[0:3,:] = tK
    else:
        K_straights[(i*3-3):(i*3),:] = tK


K_straights.shape
    





