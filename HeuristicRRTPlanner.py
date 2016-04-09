import numpy
from RRTTree import RRTTree
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan2(self, start_config, goal_config, epsilon = 0.001):
        print "...start planning..."
        tree = RRTTree(self.planning_env, start_config)
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
            if len(tree.vertices)  == 1:
                target_config = self.planning_env.GenerateRandomConfiguration()
                vid, tree.vertices[vid] = tree.GetNearestVertex(target_config)
            else:
                target_config, vid = self.SelectTarget(tree)

            

            ## extend the path between nearest node to the target config
            #extend_config = self.planning_env.Extend(tree.vertices[vid], target_config)
        
            extend_config = self.planning_env.Extend(tree.vertices[vid], target_config)
            
            ## add the extend config to the tree
            if extend_config != None:
                # append extend config into the tree
                eid = tree.AddVertex(extend_config)
                # append the id mapping into the dictionary
                tree.AddEdge(vid, eid)

                # update c_max
                c_vertices = self.planning_env.ComputeDistanceRRT(tree.vertices[eid], self.goal_config)
                plan = self.tree2plan(eid, tree)
                for i in range(len(plan)-1):
                    c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])

                self.c_max = max(c_vertices, self.c_max)
                print "cmax",self.c_max
                # update plot #
                if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                    self.planning_env.PlotEdge(tree.vertices[vid], extend_config)
                    #time.sleep(10)
  
            new_config = tree.vertices[-1]
            #print "... add new nodes" 
            

        # num of vertices
        self.numOfVert = len(tree.vertices)

        print "path found"

        # tree --> plan
        return self.tree2plan(eid)

    def Plan(self, start_config, goal_config, epsilon = 0.01):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        self.C_max_f = 0
        self.C_max_r = 0
        self.start_config = start_config
        self.goal_config  = goal_config

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        

        print "Growing the tree"

        while True:

            #### Step 1: In the rear tree, sample a new config, extend, and add to the tree ####
        

            # generate new config for the rear tree
            #rconfig = self.planning_env.GenerateRandomConfiguration()
            self.c_max = self.C_max_r
            rconfig, rvid = self.SelectTarget(rtree)

            # find the node in the rear tree that is nearest to the target config
            #rvid, rtree.vertices[rvid] = rtree.GetNearestVertex(rconfig)

            # extend the tree to the target config
            #rext_config = self.planning_env.Extend_max(rtree.vertices[rvid], rconfig, 1)
            rext_config = self.planning_env.Extend_max(rtree.vertices[rvid], rconfig, 1)

            # add the extend config to the tree
            if rext_config != None:
                # append extend config into the tree
                reid = rtree.AddVertex(rext_config)
                # append the id mapping into the dictionary
                rtree.AddEdge(rvid, reid)

                # update plot #
                if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                    self.planning_env.PlotEdge(rtree.vertices[rvid], rext_config)
                

            #### Step 2: if p < 0.4, then rext_config above will be the new goal 
            #### try to connect rext_config with the front tree

            target_prob = numpy.random.uniform(0, 1)

            if target_prob < 0.4:   # the target is rext_config
            
                if rext_config != None:
                    # find the node in the front tree that is nearest to rext_config
                    nvid, ftree.vertices[nvid] = ftree.GetNearestVertex(rext_config)
                
                    # extend the nearest node in the front tree to rext_config
                    next_config = self.planning_env.Extend_max(ftree.vertices[nvid], rext_config, 1)

                    # add the extend config to the tree
                    
                    if next_config != None:
                        # append extend config into the tree
                        neid = ftree.AddVertex(next_config)
                        # append the id mapping into the dictionary
                        ftree.AddEdge(nvid, neid)

                        # update max_f
                        c_vertices = self.planning_env.ComputeDistanceRRT(ftree.vertices[neid], self.goal_config)
                        plan = self.tree2plan(neid, ftree)
                        for i in range(len(plan)-1):
                            c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])

                        self.C_max_f = max(c_vertices, self.C_max_f)

                        print "self.C_max_f",self.C_max_f

                        # update plot #
                        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                            self.planning_env.PlotEdge(ftree.vertices[nvid], next_config)

                        # stop condition
                        if self.planning_env.ComputeDistanceRRT(next_config, rext_config) < epsilon:
                            break

            #### Step 3: if p > 0.4, do a new sample to the front tree ####

            else: # random sample in the front tree

                # generate new config for the front tree
                #fconfig = self.planning_env.GenerateRandomConfiguration()
                self.c_max = self.C_max_f
                fconfig, fvid = self.SelectTarget(ftree)

                # find the node in the front tree that is nearest to the target config
                #fvid, ftree.vertices[fvid] = ftree.GetNearestVertex(fconfig)

                # extend the tree to the target config
                fext_config = self.planning_env.Extend_max(ftree.vertices[fvid], fconfig, 1)

                if fext_config != None:
                    # append extend config into the tree
                    feid = ftree.AddVertex(fext_config)
                    # append the id mapping into the dictionary
                    ftree.AddEdge(fvid, feid)

                    # update c_max_r

                    c_vertices = self.planning_env.ComputeDistanceRRT(rtree.vertices[feid], self.goal_config)
                    plan = self.tree2plan(feid, rtree)
                    for i in range(len(plan)-1):
                        c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])

                    self.C_max_r = max(c_vertices, self.C_max_r)

                    # update plot #
                    if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                        self.planning_env.PlotEdge(ftree.vertices[fvid], fext_config)

        self.numOfVert = len(ftree.vertices) + len(rtree.vertices)   
        print "path found" 
        # tree --> plan
        
        # ftree
        id1 = neid
        id2 = reid

        while id1 != 0:
            plan.append(ftree.vertices[id1])
            id1 = ftree.edges[id1]

        plan.append(start_config)
        plan.reverse()
        
        # rtree
        while id2 != 0:
            id2 = rtree.edges[id2]
            plan.append(rtree.vertices[id2])
            
        return plan

    def tree2plan(self, eid, tree):
        plan = []
        while eid != 0:
            plan.append(tree.vertices[eid])
            eid = tree.edges[eid]

        plan.append(self.start_config)
        plan.reverse()
        
        return plan

    def Cal_mquality(self, vid, prob_floor, tree):

        #should compute differently for each tree
        c_vertices = self.planning_env.ComputeDistanceRRT(tree.vertices[vid], self.goal_config)
        plan = self.tree2plan(vid, tree)
        for i in range(len(plan)-1):
            c_vertices += self.planning_env.ComputeDistanceRRT(plan[i+1], plan[i])
        print "c_vertices",c_vertices
        c_opt = self.planning_env.ComputeDistanceRRT(self.start_config, self.goal_config)
        print "c_opt",c_opt

        result = 1-(c_vertices-c_opt)/(self.c_max-c_opt)
        
        m_quality = max(result, prob_floor)

        return m_quality

    def SelectTarget(self, tree):
        while(True):
            print "\n\n"
            target_prob = numpy.random.uniform(0, 1)

            ## choose a new target config (p = 0.2)
            ## 20% to be the goal config, 80% to be a random config 

            #if target_prob < 0.4:
            #    target_config = self.goal_config
            #else:
            target_config = self.planning_env.GenerateRandomConfiguration()


            ## find the node in the tree that is nearest to the target config
            ## tree.vertices[vid]: the nearest node 
            ## vid: id of the node 

            vid, tree.vertices[vid] = tree.GetNearestVertex(target_config)
            
            m_quality =  self.Cal_mquality(vid, 0.2, tree)
            print "score",m_quality
            r_prob = numpy.random.uniform(0, 1)
            print "threshold",r_prob
            if r_prob > m_quality:
                print "--neglect sample: ", target_config
            if r_prob < m_quality:
                print "--good sample: ", target_config
                break

        return target_config, vid

    

    
        

        




