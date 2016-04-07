import numpy as np
from DiscreteEnvironment import DiscreteEnvironment
DE =  DiscreteEnvironment(np.array([0.2,0.2]),np.array([0.0,0.0]),np.array([1.0,1.0]))
#print DE.ConfigurationToGridCoord(np.array([0.56,0.12]))
print DE.GridCoordToNodeId(np.array([1,3]))
