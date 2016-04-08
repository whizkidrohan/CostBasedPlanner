import numpy
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):
        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0
        grid = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(grid)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        grid = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(grid)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for i in range(self.dimension):
        	coord[i] = numpy.ceil((config[i]-self.lower_limits[i])/self.resolution) - 1 
        	if coord[i]==-1:
        		coord[i]=0
        return coord

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for i in range(self.dimension):
        	config[i] = self.lower_limits[i] + coord[i]*self.resolution + 0.5*self.resolution
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        for i in range(self.dimension):
        	
        	prod = 1
        	if self.dimension-i-1 != 0:
	        	for j in range(self.dimension-i-1):
	        		prod=prod*self.num_cells[j-1]
	        	node_id = node_id + prod*coord[self.dimension-i-1]
        	else:
				node_id = node_id + coord[self.dimension-i-1]

        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        sum = node_id
        for i in range(self.dimension):
            prod = 1
            if self.dimension-i-1 != 0:
                for j in range(self.dimension-i-1):
                    prod=prod*self.num_cells[j-1]
                coord[self.dimension-i-1] = numpy.floor(sum/prod)
                sum = sum % prod
            else:
                coord[self.dimension-i-1] = sum
        return coord
