import numpy as np
from DiscreteEnvironment import DiscreteEnvironment
DE =  DiscreteEnvironment(np.array([0.2] * 7),np.array([-1] * 7),np.array([1] * 7))

grid = DE.ConfigurationToGridCoord(np.array([-0.29] * 7))
print grid
config = DE.GridCoordToConfiguration(grid)
print config

# NodeId  = DE.GridCoordToNodeId(np.array([1,2,3,4,5,6,7]))
# Coord = DE.NodeIdToGridCoord(NodeId);
# print Coord

# Coord = DE.NodeIdToGridCoord(10000);
# NodeId  = DE.GridCoordToNodeId(Coord)
# print NodeId