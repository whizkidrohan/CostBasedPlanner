import Queue

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        self.planning_env.InitializePlot(goal_config)
        plan = []
        plan_config = []
        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        print start_config
        start_nid = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_nid =  self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        
        open_list = Queue.PriorityQueue()
        closed_list = []        
        open_list.put((0,-1,start_nid))
        
        while open_list:
            N = open_list.get_nowait()
            closed_list.append((N[1],N[2]));
            if N[2] == goal_nid:
                current_node = N[2]
                while current_node != start_nid:
                    plan.append(current_node)
                    current_node = self.get_parent(closed_list,current_node)
                break
            successors = self.planning_env.GetSuccessors(N[2])
            for successor in successors:
                if not self.in_closed_list(closed_list, successor):
                    if not (N[0]+1,N[2],successor) in open_list.queue:
                        open_list.put((N[0]+1,N[2],successor))
                        #self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(N[2]), \
                         #   self.planning_env.discrete_env.NodeIdToConfiguration(successor))
        plan.append(start_nid)
       # print "Start:" , start_nid , " Goal:" , goal_nid , " Plan: " , plan[::-1]
	print "nodes:",len(closed_list)
        plan = plan[::-1]
        for nodeId in plan:
            plan_config.append(self.planning_env.discrete_env.NodeIdToConfiguration(nodeId))
        for i in range(len(plan_config)-1):
            self.planning_env.PlotEdge(plan_config[i],plan_config[i+1])
    
        return plan_config

    def in_closed_list(self,list_of_lists, element):
        for list_ in list_of_lists:
            if element == list_[1]:
                return True
        return False

    def get_parent(self,list_of_lists,element):
        for list_ in list_of_lists:
            if element == list_[1]:
                return list_[0]
