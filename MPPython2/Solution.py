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

    def score(self,node):
        return (0,node)

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        connecteced_component = set([self.isp])

        possibleNext = set(self.graph[self.isp])
        
        print(f"len of pos next is {len(self.graph)}")

        while(len(possibleNext)):

            childScores = sorted(possibleNext, reverse= True, key=lambda x: self.score(x))

            connecteced_component.add(childScores[0])

            possibleNext.remove(childScores[0])

            possibleNext.update(self.graph[childScores[0]])

            possibleNext.difference_update(connecteced_component)  #this sucks, we can do better but we lazy rn


        


        paths, bandwidths, priorities = {}, {}, {}
        # Note: You do not need to modify all of the above. For Problem 1, only the paths variable needs to be modified. If you do modify a variable you are not supposed to, you might notice different revenues outputted by the Driver locally since the autograder will ignore the variables not relevant for the problem.
        # WARNING: DO NOT MODIFY THE LINE BELOW, OR BAD THINGS WILL HAPPEN
        return (paths, bandwidths, priorities)
