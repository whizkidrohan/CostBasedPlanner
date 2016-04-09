import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import copy
import time
import random

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

        successors_id = []
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

    def CollisionChecker_RRT(self, config):
        #config = self.discrete_env.GridCoordToConfiguration(node_coord)
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

    def ComputeDistanceRRT(self, start_config, end_config):

        dist = numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)
        return cost

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        #lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        while True:
           
            config = numpy.random.uniform(self.lower_limits, self.upper_limits)
            
            check = self.CollisionChecker_RRT(config)

            if check == False:
                break
            
        
        return numpy.array(config)
    
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

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        tstart = time.time()

        while ( (time.time() - tstart) < timeout ):


            x1, x2 = sorted(random.sample(range(len(path)), 2))


            #x1 = int(numpy.random.uniform(0,len(path)-1))
            #x2 = int(numpy.random.uniform(x1+1,len(path)-1))
            start_config = path[x1]
            end_config = path[x2]

            #print (x1,x2)
            #print len(path)
            #print path

            extend_config = self.Extend(start_config, end_config)

            if extend_config != None:
                if all(extend_config ==  end_config):
                    
                    #print "successful connection"
                    new_path = []
                    for i in range(0,x1+1):
                        new_path.append(path[i])
                    for i in range(x2,len(path)):
                        new_path.append(path[i])
                    path = new_path
 
        
        return path

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
        stepSize = 0.1
        numOfInterp = int(self.ComputeDistanceRRT(start_config, end_config)/stepSize)
        #print "numOfInterp", numOfInterp
        interp_config = numpy.zeros(len(start_config))
        i=0
        
        for i in range(numOfInterp):
            for j in range(len(start_config)):
                interp_config[j] = numpy.linspace(start_config[j], end_config[j], numOfInterp)[i]

            check = self.CollisionChecker_RRT(interp_config)

            if check == False:
                extend_config = numpy.array(interp_config)
            else:
                break

        if i == 1 or i == 0:
            #print "can't extend"
            return None
        else:
            #print "---- extend ----, i-->", i 
            
            return extend_config
            
    def Extend_max(self, start_config, end_config, max_extend):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
        plan_step_size = 0.05
        unit_vec = end_config - start_config #[x - y for x, y in zip(end_config,start_config)]
        dist = numpy.linalg.norm(unit_vec)
        unit_vec = [i / dist for i in unit_vec]
        count = 0
        curr_config = start_config
        c3 = False
        while True:
          #print count
          c1 = count*plan_step_size < dist  #condition 1: end_config is not  reached
          c2 =  count*plan_step_size < max_extend #condition 2: within boundaries

          if  not c1: return end_config
          if not c2: return numpy.array(curr_config) - plan_step_size*numpy.array(unit_vec)
          c3 = not self.CollisionChecker_RRT(curr_config) #condition 3: robot is not in collision
          if not c3: break
          count += 1
          curr_config = [ x + count*plan_step_size*y for x,y in zip(start_config, unit_vec)]
        #print curr_config, start_config  
        if all([i == j for i,j in zip(curr_config,start_config)]): return None
        curr_config = [ x - plan_step_size*y for x,y in zip(start_config, unit_vec)]
        if all([i == j for i,j in zip(curr_config,start_config)]): return None
        return numpy.array(curr_config)