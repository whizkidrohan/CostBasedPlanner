import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []
        successors_id = []
        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        #for example: [1,2,3,4,5,6,7]
        new = []
        
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        for i in range(len(coord)):
            new = list(coord)
            new[i] += 1
            if new[i] >= 0 and new[i] <= self.discrete_env.num_cells[i]-1:
                if not self.CollisionCheck(new):
                    successors.append(new)

            new = list(coord)
            new[i] -= 1
            if new[i] >= 0 and new[i] <= self.discrete_env.num_cells[i]-1:
                if not self.CollisionCheck(new):
                    successors.append(new)
            
        for item in successors:
            successors_id.append(self.discrete_env.GridCoordToNodeId(item))
        return successors_id
        

    def CollisionCheck(self, node_coord):
        config = self.discrete_env.GridCoordToConfiguration(node_coord)
        with self.robot.GetEnv():
            self.robot.SetDOFValues(config, self.robot.GetActiveDOFIndices())
        
        # collision check: when there is collision, return true
        check_1 = self.robot.GetEnv().CheckCollision(self.robot) 

        # self collision check: when self collision, return true
        check_2 = self.robot.CheckSelfCollision()

        # we want both to be false
        check = check_1 or check_2
        
        #if collision, true
        return check



    def ComputeDistance(self, start_id, end_id):
        dist = 0
        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_coord = self.discrete_env.NodeIdToGridCoord(end_id)
        weights=[4,4,4,1,1,1,1]

        dist = numpy.linalg.norm(weights*(numpy.array(start_coord) - numpy.array(end_coord)))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)
        return cost


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
        stepSize = 0.1
        numOfInterp = int(self.ComputeDistance(start_config, end_config)/stepSize)
        #print "numOfInterp", numOfInterp
        interp_config = numpy.zeros(len(start_config))
        i=0
        
        for i in range(numOfInterp):
            for j in range(len(start_config)):
                interp_config[j] = numpy.linspace(start_config[j], end_config[j], numOfInterp)[i]

            check = self.CollisionChecker(interp_config)

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

    def ShortenPath(self, path, timeout=5.0):
        
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