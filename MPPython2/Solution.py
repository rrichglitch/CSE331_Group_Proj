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
        self.node2downStream = [ set() for i in range(len(self.graph)) ]
        self.current_path_acceptable = [-1]*len(self.graph)
        # self.is_client = [False]*len(self.graph)
        # for c in self.info['list_clients']:
        #     self.is_client[c] = True

        # TUNE THIS MAGIC NUMBER [0,1]:
        # higher means faster; at 0 total algo time is O(n^3) at 1 its O(n^2)
        # the lower this is, the more precisely the graph is formation is guided by score_path
        self.top_x_percent = .8

    def extra_path_delay(self,node): # O(1)
        return len(self.node2downStream[node])//self.info["bandwidths"][node]

    # node2downStream must be populated before calling this method
    def dijkstras(self, grumps): # grumps are the nodes to return paths for
        distances = [sys.maxsize]*len(self.graph)
        distances[self.isp] = 0
        path_src = [-1]*len(self.graph)
        
        pq = [(0, self.isp)]
        visited = [False]*len(self.graph) # unfortunately need this bc distances are updated before visit
        
        while len(pq):
            cur_dist, cur_node = heapq.heappop(pq)
            
            # If we've already processed this vertex, skip it
            if visited[cur_node]: continue  
            else: visited[cur_node] = True
            
            for child in self.graph[cur_node]:
                weight = 1+self.extra_path_delay(child)

                distance = cur_dist + weight
                
                # If we found a shorter path, update it
                if distance < distances[child]:
                    distances[child] = distance
                    path_src[child] = cur_node
                    heapq.heappush(pq, (distance, child))

        # extract paths
        grump_paths = {}
        for grump in grumps:
            path = [grump]
            while path[len(path)-1] != self.isp:
                path.append(path_src[path[len(path)-1]])

            path.reverse()
            grump_paths[grump] = path

        return grump_paths

    # This gives a heuristic score based on pmt, path_len, delay, and acceptability   O(n)
    def score_current_path(self,path): # call only on client paths
        node = path[len(path)-1]

        base_delay = len(self.short_paths[node])-1
        extra_delay = max([ self.extra_path_delay(path[i]) for i in range(len(path)-1) ])
        cur_delay = base_delay+extra_delay
        self.current_path_acceptable[node] = cur_delay <= (base_delay * self.info["alphas"][node])

        money = self.info["payments"][node] * self.current_path_acceptable[node]

        return money/cur_delay
        
    # call only on client paths      O(n)
    def check_path_acceptable(self,path):
        node = path[len(path)-1]

        if self.current_path_acceptable[node] >= 0:
            return self.current_path_acceptable[node] # any new paths will already be cached during scoring

        base_delay = len(self.short_paths[node])-1
        extra_delay = max([ self.extra_path_delay(path[i]) for i in range(len(path)-1) ])

        return (base_delay+extra_delay) <= (base_delay * self.info["alphas"][node])


    # to begin we run our MST algo
    # next we iterate through the paths created by the MST and check for bandwidth delays
    # we sort our nodes which r unhappy with bandwidth delay by pmt and/or path len
    # find a new short path for unhappy nodes given the current graph (dijkstra) 
    def improve_MST(self):
        paths = {}

        first = True
        last_useful = False
        unhappy = self.info["list_clients"]
        last_unhappy_cnt = len(unhappy)+1

        while len(unhappy) and (len(unhappy) < last_unhappy_cnt or last_useful): # exit loop if we got everyone or we cant get anymore
            last_unhappy_cnt = len(unhappy)
            last_useful = len(unhappy) < last_unhappy_cnt # comment this out to save 1 iteration. leave in to maybe get a bit more revenue

            # run dijkstras to find the shortest known paths for our grumps with the current knowledge of delays
            new_paths = self.dijkstras(unhappy)
            if first: # on the first run, since none of the node2downStream use for weighting is filled, dijkstra is simply BFS yielding MST
                self.short_paths = new_paths
                first = False

            # add in top x% of paths sorted by payment or sorted by path_len or both
            sorted_new_paths = sorted(new_paths.values(), reverse=True, key=self.score_current_path) # O(n^2)
            new_cnt = len(new_paths)
            for i in range(min(new_cnt,int(new_cnt*self.top_x_percent)+1)): # tune this magic top_x_percent!
                path = sorted_new_paths[i]
                node = path[len(path)-1]
                paths[node] = path
                for i in range(len(path)-1):
                    self.node2downStream[path[i]].add(node)

            unhappy = []
            for c in self.info["list_clients"]:
                if c not in paths: unhappy.append(c)
                else:
                    if c not in new_paths: self.current_path_acceptable[c] = -1 # mark old paths for rechecking delay
                    if not self.check_path_acceptable(paths[c]): # O(n^2)
                        unhappy.append(c)
                        # as we find unhappy clients, pull them from their current path
                        for upstream in paths[c]: # O(n^2)
                            self.node2downStream[upstream].discard(c)

        # print(paths)
        for c in self.info["list_clients"]:
            paths.setdefault(c,[self.isp,c]) # add in default paths for the shitters that didnt make the cut

        return paths

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        return self.improve_MST(), {}, {}

    # the simulator dequeues packets for closer nodes first
