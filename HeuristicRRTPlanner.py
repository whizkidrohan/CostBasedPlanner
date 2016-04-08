import numpy
from RRTTree import RRTTree

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space


        new_config = start_config

        

        print "Growing the tree"
        
        while(self.planning_env.ComputeDistance(new_config, goal_config) > epsilon):
            
            ## choose a new target config (p = 0.2)
            ## 20% to be the goal config, 80% to be a random config 
            target_prob = numpy.random.uniform(0, 1)

            if target_prob < 0.2:
                target_config = goal_config
            else:
                target_config = self.planning_env.GenerateRandomConfiguration()


            ## find the node in the tree that is nearest to the target config
            ## tree.vertices[vid]: the nearest node 
            ## vid: id of the node 
            vid, tree.vertices[vid] = tree.GetNearestVertex(target_config)
            
          

            ## extend the path between nearest node to the target config
            #extend_config = self.planning_env.Extend(tree.vertices[vid], target_config)
        
            extend_config = self.planning_env.Extend_max(tree.vertices[vid], target_config, 1)
            
            ## add the extend config to the tree
            if extend_config != None:
                # append extend config into the tree
                eid = tree.AddVertex(extend_config)
                # append the id mapping into the dictionary
                tree.AddEdge(vid, eid)

                # update plot #
                if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                    self.planning_env.PlotEdge(tree.vertices[vid], extend_config)
  
            new_config = tree.vertices[-1]
            #print "... add new nodes" 

        # num of vertices
        self.numOfVert = len(tree.vertices)

        print "path found"

        # tree --> plan
        while eid != 0:
            plan.append(tree.vertices[eid])
            eid = tree.edges[eid]

        plan.append(start_config)
        
        plan.reverse()
        
        return plan
