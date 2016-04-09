import numpy
from RRTTree import RRTTree
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):
        print "...start planning..."
        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        self.start_config = start_config
        self.goal_config  = goal_config
        new_config = self.start_config
        self.c_max = 0
        print "Growing the tree"
        

        while(self.planning_env.ComputeDistanceRRT(new_config, self.goal_config) > epsilon):
            
            # random sample: target_config, nearest node id in tree: vid
            if len(self.tree.vertices)  == 1:
                
                target_config = self.planning_env.GenerateRandomConfiguration()
                vid, self.tree.vertices[vid] = self.tree.GetNearestVertex(target_config)
            else:
                target_config, vid = self.SelectTarget()

            

            ## extend the path between nearest node to the target config
            #extend_config = self.planning_env.Extend(tree.vertices[vid], target_config)
        
            extend_config = self.planning_env.Extend(self.tree.vertices[vid], target_config)
            
            ## add the extend config to the tree
            if extend_config != None:
                # append extend config into the tree
                eid = self.tree.AddVertex(extend_config)
                # append the id mapping into the dictionary
                self.tree.AddEdge(vid, eid)

                # update c_max
                c_vertices = self.planning_env.ComputeDistanceRRT(self.tree.vertices[eid], self.goal_config)
                plan = self.tree2plan(eid)
                for i in range(len(plan)-1):
                    c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])

                self.c_max = max(c_vertices, self.c_max)
                print "cmax",self.c_max
                # update plot #
                if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                    self.planning_env.PlotEdge(self.tree.vertices[vid], extend_config)
                    #time.sleep(10)
  
            new_config = self.tree.vertices[-1]
            #print "... add new nodes" 
            

        # num of vertices
        self.numOfVert = len(self.tree.vertices)

        print "path found"

        # tree --> plan
        return self.tree2plan(eid)

    def tree2plan(self, eid):
        plan = []
        while eid != 0:
            plan.append(self.tree.vertices[eid])
            eid = self.tree.edges[eid]

        plan.append(self.start_config)
        plan.reverse()
        
        return plan

    def Cal_mquality(self, vid, prob_floor):

        c_vertices = self.planning_env.ComputeDistanceRRT(self.tree.vertices[vid], self.goal_config)
        plan = self.tree2plan(vid)
        for i in range(len(plan)-1):
            c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])
        print "c_vertices",c_vertices
        c_opt = self.planning_env.ComputeDistanceRRT(self.start_config, self.goal_config)
        print "c_opt",c_opt
        result = 1-(c_vertices-c_opt)/(self.c_max-c_opt)
        
        m_quality = max(result, prob_floor)

        return m_quality

    def SelectTarget(self):
        while(True):
            print "\n\n"
            target_prob = numpy.random.uniform(0, 1)

            ## choose a new target config (p = 0.2)
            ## 20% to be the goal config, 80% to be a random config 

            if target_prob < 0.4:
                target_config = self.goal_config
            else:
                target_config = self.planning_env.GenerateRandomConfiguration()


            ## find the node in the tree that is nearest to the target config
            ## tree.vertices[vid]: the nearest node 
            ## vid: id of the node 

            vid, self.tree.vertices[vid] = self.tree.GetNearestVertex(target_config)
            
            m_quality =  self.Cal_mquality(vid, 0.5)
            print "score",m_quality
            r_prob = numpy.random.uniform(0, 1)
            print "threshold",r_prob
            if r_prob > m_quality:
                print "--neglect sample: ", target_config
            if r_prob < m_quality:
                print "--good sample: ", target_config
                break

        return target_config, vid

    

    
        

        




