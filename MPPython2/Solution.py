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

    def will_pay(self,node):
        return 1

    def score(self, node, path_bandwidth):
        if scores[node] >= 0:
            return scores[node]

        score = node.pmt * self.will_pay(node)

        child_scores = [ self.score(child,path_bandwidth-1,)*(1-self.connecteced_component[child]) for child in self.graph[node]]

        child_score.sort(reverse=True)

        for i in range(node.bandwidth):
            score += child_scores[i]
            
        scores[node] = score
        return score

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        self.connecteced_component = [0]*len(self.graph)

        possibleNext = set(self.graph[self.isp])
        
        print(f"len of pos next is {len(self.graph)}")

        path_bandwidth = 0      # TODO

        while(len(possibleNext)):

            childScores = sorted(possibleNext, reverse= True, key=lambda x: self.score(x,path_bandwidth,connecteced_component))

            connecteced_component.add(childScores[0])

            possibleNext.remove(childScores[0])

            possibleNext.update(self.graph[childScores[0]])

            possibleNext.difference_update(connecteced_component)  #this sucks, we can do better but we lazy rn        


        paths, bandwidths, priorities = {}, {}, {}
        # Note: You do not need to modify all of the above. For Problem 1, only the paths variable needs to be modified. If you do modify a variable you are not supposed to, you might notice different revenues outputted by the Driver locally since the autograder will ignore the variables not relevant for the problem.
        # WARNING: DO NOT MODIFY THE LINE BELOW, OR BAD THINGS WILL HAPPEN
        return (paths, bandwidths, priorities)
