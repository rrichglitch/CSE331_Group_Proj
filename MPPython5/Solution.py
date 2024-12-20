from heapq import heapify,heappush,heappop
import sys
from math import ceil

class Solution:

    def __init__(self, problem, isp, graph, info):
        self.problem = problem
        self.isp = isp
        self.graph = graph
        self.info = info
        self.node2downStream = [set() for _ in range(len(self.graph))]
        self.m_extra_path_delay = [-1] * len(self.graph)
        self.m_upgrade_for_path_delay_improvement = [(-1,-1)] * len(self.graph)
        # self.cost_bandwidth = self.info["cost_bandwidth"]
        self.C = self.info["list_clients"]
        self.short_paths = None
        self.unhappy = None

        # TUNE THIS MAGIC NUMBER [0,1]:
        self.top_x_percent = 0.8


    def extra_node_delay(self, node): # O(1)
        return len(self.node2downStream[node]) // self.info["bandwidths"][node]

    def extra_path_delay_r(self, path, i): # O(1) per new checked node
        if i < 0: return 0

        end_node = path[i]

        if self.m_extra_path_delay[end_node] > 0: return self.m_extra_path_delay[end_node]

        self.m_extra_path_delay[end_node] = max( self.extra_node_delay(end_node), self.extra_path_delay_r(path, i-1) )

        return self.m_extra_path_delay[end_node]

    def extra_path_delay(self, path, refresh): # O(1) per new checked node
        if len(path) <= 1: return 0

        if refresh: # refresh once per call then allow the recursive tree to cache again
            for i in range(len(path)):
                self.m_extra_path_delay[path[i]] = -1

        return self.extra_path_delay_r(path, i=len(path)-2) # i is index and should exclude this end node

    # node2downStream must be populated before calling this method
    def dijkstras(self, grumps):  # grumps are the nodes to return paths for
        distances = [sys.maxsize]*len(self.graph)
        distances[self.isp] = 0
        path_src = [-1]*len(self.graph)
        
        pq = [(0, self.isp)]
        visited = [False]*len(self.graph) # unfortunately need this bc distances are updated before visit
        
        while len(pq):
            cur_dist, cur_node = heappop(pq)
            
            # If we've already processed this vertex, skip it
            if visited[cur_node]: continue  
            else: visited[cur_node] = True
            
            for child in self.graph[cur_node]:
                weight = 1+self.extra_node_delay(child)

                distance = cur_dist + weight
                
                # If we found a shorter path, update it
                if distance < distances[child]:
                    distances[child] = distance
                    path_src[child] = cur_node
                    heappush(pq, (distance, child))

        # extract paths
        grump_paths = {}
        for grump in grumps:
            path = [grump]
            while path[len(path)-1] != self.isp:
                path.append(path_src[path[len(path)-1]])

            path.reverse()
            grump_paths[grump] = path

        return grump_paths


    # call only on client paths      O(n)
    def path_is_acceptable(self,path,refresh):
        node = path[len(path)-1]

        base_delay = len(self.short_paths[node])-1
        extra_delay = self.extra_path_delay(path,refresh)
        cur_delay = base_delay + extra_delay

        return cur_delay <= (base_delay * self.info["alphas"][node])

    # This gives a heuristic score based on pmt, path_len, delay, and acceptability   O(n)
    def score_path(self,path): # call on new client paths
        node = path[len(path)-1]

        money = self.info["payments"][node] * self.path_is_acceptable(path,refresh=True)

        base_delay = len(self.short_paths[node])-1
        extra_delay = self.extra_path_delay(path,refresh=False)
        cur_delay = base_delay + extra_delay

        return money/cur_delay


    def add_band_for_complaints(self):     
        complainers = self.unhappy

        while len(complainers):

            node_to_appease = complainers.pop()

            extra_delay_thresh = (len(self.short_paths[node_to_appease])-1) * (self.info["alphas"][node_to_appease]-1)
            extra_delay_thresh = int(extra_delay_thresh)

            path = self.paths[node_to_appease]

            for i in range(len(path)-1):
                upgrade_node = path[i]
                extra_delay = self.extra_node_delay(upgrade_node)
                delay_to_drop = extra_delay - extra_delay_thresh

                if delay_to_drop > 0:
                    # drop the delay all at once with multiplication
                    amnt_to_upgrade = len(self.node2downStream[upgrade_node]) % self.info["bandwidths"][upgrade_node]
                    amnt_to_upgrade += len(self.node2downStream[upgrade_node]) * (delay_to_drop-bool(amnt_to_upgrade))
                    self.info["bandwidths"][upgrade_node] += amnt_to_upgrade


    # to begin we run our MST algo
    # next we iterate through the paths created by the MST and check for bandwidth delays
    # we sort our nodes which are unhappy with bandwidth delay by pmt and/or path len
    # find a new short path for unhappy nodes given the current graph (dijkstra) 
    def iterative_improved_paths(self):
        paths = {}

        first = True
        last_useful = False
        unhappy = self.info["list_clients"]
        last_unhappy_cnt = len(unhappy)+1
        new_paths = None

        while len(unhappy) and (len(unhappy) < last_unhappy_cnt or last_useful): # exit loop if we got everyone or we cant get anymore
            last_unhappy_cnt = len(unhappy)
            last_useful = len(unhappy) < last_unhappy_cnt # comment this out to save 1 iteration. leave in to maybe get a bit more revenue

            # run dijkstras to find the shortest known paths for our grumps with the current knowledge of delays
            new_paths = self.dijkstras(unhappy)
            if first: # on the first run, since none of the node2downStream use for weighting is filled, dijkstra is simply BFS yielding MST
                self.short_paths = new_paths
                first = False

            # add in top x% of paths sorted by payment or sorted by path_len or both
            sorted_new_paths = sorted(new_paths.values(), reverse=True, key=self.score_path) # O(n^2)
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
                elif not self.path_is_acceptable(paths[c],refresh=(c not in new_paths)): # O(n^2)
                    unhappy.append(c)
                    # as we find unhappy clients, pull them from their current path
                    for upstream in paths[c]: # O(n^2)
                        self.node2downStream[upstream].discard(c)

        # print(paths)
        for grump in unhappy: # add in default paths for the shitters that didnt make the cut
            paths[grump] = self.short_paths[grump]

        self.unhappy = unhappy

        return paths  # output the final paths


    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        self.paths = self.iterative_improved_paths()

        # print(f"there are paths for:\n{sorted(self.graph.keys())}")

        self.add_band_for_complaints()

        # all_good = True
        # for c in self.C:
        #     path = self.paths[c]
        #     extra_delay = self.extra_path_delay(path,True)
        #     print(f"estimated extra delay for {c} is {extra_delay} ")
        #     if not self.path_is_acceptable(path,refresh=False):
        #         print(f"found an unacceptable path at {path[len(path)-1]}, \
        #             pathlen is {len(path)}, shortest path len is {len(self.short_paths[path[len(path)-1]])}")
        #         all_good = False
        #         # exit()

        # if all_good: print(f"all paths acceptable")

        priorities = {c: -(len(self.short_paths[c])-1)*self.info["alphas"][c] for c in self.C }

        # For problems 3, 4, 5, return updated bandwidths
        return self.paths, self.info["bandwidths"], priorities
