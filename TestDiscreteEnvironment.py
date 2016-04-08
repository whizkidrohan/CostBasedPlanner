import numpy as np
from DiscreteEnvironment import DiscreteEnvironment


DE =  DiscreteEnvironment(np.array([0.2] * 2),np.array([-5] * 2),np.array([5] * 2))
#print DE.lower_limits
#print DE.resolution

#DE_3D =  DiscreteEnvironment(np.array([0.2] * 3),np.array([0] * 3),np.array([1] * 3))

#grid = DE.ConfigurationToGridCoord(np.array([44,-2]))
#print grid
#config = DE.GridCoordToConfiguration(grid)44
#print config

NodeId  = DE.GridCoordToNodeId(np.array([44,-2]))
print NodeId
#NodeId1= -56
Coord = DE.NodeIdToGridCoord(NodeId)
print Coord

#Node_conf= DE.NodeIdToConfiguration(NodeId)
#print Node_conf
#conf_Node=DE.ConfigurationToNodeId(config)
#print conf_Node
