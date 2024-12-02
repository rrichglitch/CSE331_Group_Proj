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
        self.current_path_compliant = [-1] * len(self.graph)  # For complaint threshold

        # Ensure 'cost_bandwidth' is present
        if "cost_bandwidth" not in self.info:
            raise ValueError("Bandwidth increase cost 'cost_bandwidth' is missing from the input.")

        # For Problem 3, we need to consider penalties and bandwidth costs
        self.cost_bandwidth = self.info["cost_bandwidth"]
        self.rho_lawsuit = self.info.get("rho1", 0)
        self.rho_fcc = self.info.get("rho2", 0)
        self.lawsuit_amount = self.info.get("lawsuit", 0)
        self.fcc_fine = self.info.get("fcc_fine", 0)
        self.S = [c for c in self.info["list_clients"] if self.info["is_fcc"][c]]
        self.C = self.info["list_clients"]

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

    # returns the delay that would have to be reduced for this node to stop complaining
    def path_complaint_diff(self, path,refresh=False):
        node = path[len(path)-1]

        base_delay = len(self.short_paths[node])-1
        extra_delay = self.extra_path_delay(path,refresh=refresh)
        cur_delay = base_delay + extra_delay

        return cur_delay - (base_delay * self.info["alphas"][node])


    def add_band_for_complaints(self, fine_amount, thresh_beta):

        complaint_thresh = int(thresh_beta * len(self.S)) # this might be under by 1??

        complainers = [ c for c in self.C if self.current_path_compliant[c] and c in self.S ]

        band_budget = fine_amount/self.cost_bandwidth  # Maximum amount we are willing to spend in terms of bandwidth

        no_upgrades = self.info["bandwidths"].copy()

        # sort the complainers by how easy it is to keep them from complaining "complaint_diff"
        complainers = sorted( ( (self.path_complaint_diff(self.paths[node]),node) for node in complainers ), reverse=True)
        while len(complainers) > complaint_thresh and band_budget > 0 :

            _, node_to_appease = complainers.pop()

            delay_thresh = (len(self.short_paths[node_to_appease])-1) * (self.info["betas"][node_to_appease]-1)
            delay_thresh = ceil(delay_thresh)

            path = self.paths[node_to_appease]
            for i in range(len(path)-2):
                upgrade_node = path[i]
                extra_delay = self.extra_node_delay(upgrade_node)
                delay_to_drop = extra_delay - delay_thresh

                if delay_to_drop > 0:
                    # drop the delay all at once with multiplication
                    amnt_to_upgrade = len(self.node2downStream[upgrade_node]) % self.info["bandwidths"][upgrade_node]
                    amnt_to_upgrade += len(self.node2downStream[upgrade_node]) * (delay_to_drop-bool(amnt_to_upgrade))
                    self.info["bandwidths"][upgrade_node] += amnt_to_upgrade
                    band_budget -= amnt_to_upgrade

        # if we were unable to upgrade enough to avoid fine then dont upgrade at all
        if len(complainers) > complaint_thresh:
            self.info["bandwidths"] = no_upgrades


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
        for c in self.info["list_clients"]:
            paths.setdefault(c,[self.isp,c]) # add in default paths for the shitters that didnt make the cut

        return paths  # output the final paths


    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        self.paths = self.iterative_improved_paths()

        self.add_band_for_complaints(self.fcc_fine, self.rho_fcc)
        self.add_band_for_complaints(self.lawsuit_amount, self.rho_lawsuit)

        # For problems 3, 4, 5, return updated bandwidths
        return self.paths, self.info["bandwidths"], {}
