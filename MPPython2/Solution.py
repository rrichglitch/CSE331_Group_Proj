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
        self.bandwidths = [-1]*len(self.graph)
        self.node2downStream = [ set() for i in range(len(self.graph)) ]
        self.best_delays = None
        self.is_client = [False]*len(self.graph)
        for c in self.info['list_clients']:
            self.is_client[c] = True
        delay_srcs = [ [set()] for i in range(len(self.graph)) ]

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
        

    
    def check_path_acceptable(self,path): # O(n)
        extra_delay = max([ self.extra_path_delay(path[i]) for i in range(len(path)-1) ])
        base_delay = len(self.short_paths[path[len(path)-1]])-1
        return (extra_delay+base_delay) <= (base_delay * self.info["alphas"].get(path[len(path)-1],9999))


    def score_path(self,path):
        return self.info["payments"][path[len(path)-1]]/(len(path)-1) # combined pmt and path len scoring


    # packets dequeue for closer nodes first

    # to begin we run our MST algo
    # next we iterate through the paths created by the MST and check for bandwidth delays
    # we sort our nodes which r unhappy with bandwidth delay by pmt
    # find a path the node is happy with foreach
    def improve_MST(self):
        paths = {}
        # bfs is run on initialization to give us MST
        self.short_paths = self.local_bfs_path(self.graph, self.isp, self.info["list_clients"])
        new_paths = self.short_paths.copy()
        # print(self.short_paths)

        # paths = self.dijkstras(self.info["list_clients"])

        last_unhappy_cnt = -1
        while True:
            # add in top x% of paths sorted by payment or sorted by path_len or both
            new_paths = sorted(new_paths.values(), reverse=True, key=self.score_path)
            size = len(new_paths)
            for i in range(min(size,int(size*.85)+1)):
                path = new_paths[i]
                node = path[len(path)-1]
                paths[node] = path
                for i in range(len(path)-1):
                    self.node2downStream[path[i]].add(node)

            unhappy = []
            for c in self.info["list_clients"]:
                if c not in paths: unhappy.append(c)
                elif not self.check_path_acceptable(paths[c]): # O(n^2) total
                    unhappy.append(c)
                    # as we find unhappy clients, pull them from their current path
                    for upstream in paths[c]: # O(n^2) total
                        self.node2downStream[upstream].discard(c)

            if not len(unhappy) or last_unhappy_cnt == len(unhappy): break # exit loop if we got everyone or we cant get anymore

            last_unhappy_cnt = len(unhappy)

            # now run dijkstras to find the shortest known paths for our grumps with the knowledge of current delays
            new_paths = self.dijkstras(unhappy)
            # print(new_paths4unhappy)

        # print(paths)
        for c in self.info["list_clients"]:
            paths.setdefault(c,[self.isp,c])

        return paths

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        return self.improve_MST(), {}, {}
