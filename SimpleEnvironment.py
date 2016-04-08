 import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import copy

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        #self.lower_limits = [-5., -5.]
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)
        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        ACTIONS = [(0,1),(0,-1),(1,0),(-1,0)]
        
        for action in ACTIONS:
            successor = (map(lambda x,y: x+y, action, coord))
            if successor[0] < 0 or \
            successor[0] > self.discrete_env.num_cells[0]-1 or    \
            successor[1] < 0 or    \
            successor[1] > self.discrete_env.num_cells[1]-1 or \
            self.CollisionChecker(successor):
                continue
            successors.append(successor)
       
        for item in successors:
            successors_id.append(self.discrete_env.GridCoordToNodeId(item))

        return successors_id

    def CollisionChecker(self, node_coord):
        config = self.discrete_env.GridCoordToConfiguration(node_coord)
        config_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                   [ 0, 1,  0, config[1]], 
                                   [ 0, 0,  1, 0], 
                                   [ 0, 0,  0, 1]])

        with self.robot.GetEnv():
            self.robot.SetTransform(config_pose)

        check = self.robot.GetEnv().CheckCollision(self.robot)
        return check

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_coord = self.discrete_env.NodeIdToGridCoord(end_id)
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        dist = abs(start_coord[0] - end_coord[0]) + abs(start_coord[1] - end_coord[1])
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)
        return cost
    
    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
