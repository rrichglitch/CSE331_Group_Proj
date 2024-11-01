from Traversals import bfs_path
import heapq
from collections import deque
from Simulator import Simulator
import sys

class Solution:

    def __init__(self, problem, isp, graph, info):
        self.problem = problem
        self.isp = isp
        self.graph = graph
        self.info = info
        self.scores = [-1]*len(self.graph)
        self.connecteced_component = None
        self.MST = None 
        

    def will_pay(self,node):
        return 1

    def local_bfs_path(self, graph, isp, list_clients):

        paths = {}

        graph_size = len(graph)
        priors = [-1]*graph_size
        search_queue = deque()
        search_queue.append(isp)

        while search_queue:
            node = search_queue.popleft()
            for neighbor in graph[node]:
                if (priors[neighbor] == -1 and neighbor != isp):
                    priors[neighbor] = node
                    search_queue.append(neighbor)

        for client in list_clients:
            path = []
            current_node = client
            while (current_node != -1):
                path.append(current_node)
                current_node = priors[current_node]
            path = path[::-1]
            paths[client] = path

        return paths
    

    def score(self, node, path_bandwidth, x):
        return 0
        if scores[node] >= 0:
            return scores[node]

        score = node.pmt * self.will_pay(node)

        child_scores = [ self.score(child,path_bandwidth-1,)*(1-self.connecteced_component[child]) for child in self.graph[node]]

        child_score.sort(reverse=True)

        for i in range(node.bandwidth):
            score += child_scores[i]
            
        scores[node] = score
        return score
    
    def under_thresh(self,delay,node,info):

        if self.pathLens is None:

            paths = self.local_bfs_path(self.graph, self.isp, self.info["list_clients"])

            self.path_lens = [len(path) for path in paths.values()]
            
        if node in self.info["alphas"]:
            return delay <= self.pathLens[node] * self.info["alphas"][node]
        else:
            return False
        
        


    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        self.connecteced_component = [0]*len(self.graph)

        self.connecteced_component = set()

        possibleNext = set(self.graph[self.isp])
        
        print(f"len of pos next is {len(self.graph)}")

        path_bandwidth = 0      # TODO

        while(len(possibleNext)):

            childScores = sorted(possibleNext, reverse= True, key=lambda x: self.score(x,path_bandwidth,self.connecteced_component))

            self.connecteced_component.add(childScores[0])

            possibleNext.remove(childScores[0])

            possibleNext.update(self.graph[childScores[0]])

            possibleNext.difference_update(self.connecteced_component)  #this sucks, we can do better but we lazy rn        


        paths, bandwidths, priorities = {}, {}, {}
        # Note: You do not need to modify all of the above. For Problem 1, only the paths variable needs to be modified. If you do modify a variable you are not supposed to, you might notice different revenues outputted by the Driver locally since the autograder will ignore the variables not relevant for the problem.
        # WARNING: DO NOT MODIFY THE LINE BELOW, OR BAD THINGS WILL HAPPEN
        return (paths, bandwidths, priorities)
