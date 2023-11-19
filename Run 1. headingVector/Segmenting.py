import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.optimize as opt
import scipy.linalg as linalg
import math

exec(open("emFunctions.py").read())

''' Variables '''
global mu_dim, zeta_dim, K
mu_dim = 3; zeta_dim = 5 
secondsToTrim = 3
lengthToTrim = 0.8

headingAngleMode = False
initGuess_Ones = False
straightsMode = True


if headingAngleMode:
    zeta_dim = 4
'''           '''

# Import the data to be segmented
All_mu = np.ones((1,mu_dim+1))
All_zeta = np.ones((1,zeta_dim+1))

ID_List = [1,2,3,6,7,8,11,12,9,10,13,14]
for ID in ID_List:  
    inputFile = f'Saves/straightsAndcircles_{ID}.csv'
    [mu, zeta, _, _, _, _, _, y_pos, _, _, _, _, _] = import_data(inputFile, headingAngleMode)
    # Trimming out first Ns of data
    indexOfTrim =  int(secondsToTrim/pollFrequency_sec)
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
length = lengthToTrim * length_actual

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
# reminder: Data has been preprocessed to remove the timestamp
straights_mu = np.array(All_mu)[segment_straights, :]
straights_zeta = np.array(All_zeta)[segment_straights, :] 

turnRH_mu = np.array(All_mu)[segment_turnsRH, :]
turnRH_zeta = np.array(All_zeta)[segment_turnsRH, :] 

# Generalised Optimisation
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

if initGuess_Ones:   
    kV = np.ones([1,mu_dim*zeta_dim])
else:
    kV = np.zeros([1,mu_dim*zeta_dim])
    
if straightsMode:
    func = lambda K : costFunc(K, straights_mu, straights_zeta)
else:
    func = lambda K : costFunc(K, turnRH_mu, turnRH_zeta)
    
thisResult = opt.minimize(func, kV, method = 'Nelder-Mead')
success = thisResult.get('success')
cost = thisResult.get('fun')
print('\n')

if straightsMode:
    print('Optimising for Straights')
else:
    print('Optimising for Turns_RH')
print(f'success: {success} \n with cost = {cost} \n using initial guess of {kV} \n bearing mode: {headingAngleMode}')

if 'Result' not in globals():
    Result = list()
Result.append([thisResult.get('success'),thisResult.get('fun'), thisResult.get('x'), kV, ID_List])

K = np.reshape(thisResult.get('x'), [mu_dim,zeta_dim])
#%% Save Results
columnNames = ['Success or Failure', 'Cost', 'K', 'k0', 'list of CSVs']

Results = pd.DataFrame(np.array(Result), columns = columnNames)
outputFile = 'SimpleSegment_Results.csv'

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








