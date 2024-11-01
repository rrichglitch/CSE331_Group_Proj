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
        self.is_client = [False]*len(self.graph)
        self.best_delays = None
        for c in self.info['list_clients']:
            self.is_client[c] = True

        self.short_paths = self.local_bfs_path(self.graph, self.isp, self.info["list_clients"])

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

    def will_pay(self,node,delay): # this is the under_threshold method
        if self.best_delays is None:
            self.best_delays = {node: len(path)-1 for node, path in self.short_paths.items()}
            
        return (delay <= (self.best_delays[node] * self.info["alphas"].get(node,0))) * self.info["payments"].get(node,0)

    def score(self,node,delay,connecteced_component):
        sum_l1_pay = self.will_pay(node,delay)
        for child in self.graph[node]:
            if self.is_client[child] and not connecteced_component[child]: sum_l1_pay += self.will_pay(child,delay+1)
        return sum_l1_pay
        


    def heuristic_scores_approach(self):
        paths = {}

        connecteced_component = [False]*len(self.graph)
        connecteced_component[self.isp] = True

        possibleNext = [ [ (0,self.isp,sys.maxsize,[self.isp]) ], [] ] # element format is ( pmt, node, bandwidth, path )
        heap_ind = 0
        
        print(f"len of graph is {len(self.graph)}")

        while len(possibleNext):
            pmt, curNode, bandwidth, path = heapq.heappop(possibleNext[heap_ind])
            # pmt, curNode, bandwidth, path = possibleNext[heap_ind].popleft()
            # print(f"at headpind {heap_ind} dequed {curNode} with pmt {pmt}")

            bandwidth = min(bandwidth, self.info["bandwidths"][curNode]) # should be curNode_band -1
            # print(f"bandwidth for {curNode} is {bandwidth}")
            
            band_expended = 1
            # for child in self.graph[curNode]: # only clients will expend bandwidth
            #     if self.is_client[child]:
            #         band_expended += 1
            # band_expended = min(bandwidth,band_expended)
            next_bandwidth = bandwidth

            for child in self.graph[curNode]:
                if not connecteced_component[child]:
                    connecteced_component[child] = True
                    nxt_path = path+[child]
                    if self.is_client[child]:
                        if band_expended > 0:
                            # band_expended -= 1
                            paths[child] = nxt_path
                            heapq.heappush( possibleNext[heap_ind+1], (-self.score(child,len(nxt_path)-1,connecteced_component),child,next_bandwidth,nxt_path) )
                            # possibleNext[heap_ind+1].append( (-self.will_pay(child,len(nxt_path)-1),child,next_bandwidth,nxt_path) )
                    else:
                        heapq.heappush( possibleNext[heap_ind+1], (0,child,next_bandwidth,nxt_path) ) # always add routers to front of q
                        # possibleNext[heap_ind+1].append( (float("-inf"),child,next_bandwidth,nxt_path) ) # always add routers to front of q

            if not len(possibleNext[heap_ind]):
                heap_ind += 1
                possibleNext.append([])
                if not len(possibleNext[heap_ind]):
                    break

        # Note: You do not need to modify all of the above. For Problem 1, only the paths variable needs to be modified. If you do modify a variable you are not supposed to, you might notice different revenues outputted by the Driver locally since the autograder will ignore the variables not relevant for the problem.
        # WARNING: DO NOT MODIFY THE LINE BELOW, OR BAD THINGS WILL HAPPEN
        print(f"len of paths: {len(paths)}")
        # print(paths[10840])
        # print([ self.info["bandwidths"][node] for node in paths[10840] ])
        for c in self.info["list_clients"]:
            if c not in paths:
                print(f"node {c} was removed due to insufficient bandwidth")

            paths.setdefault(c,[self.isp,c])

        # paths[10840] = [self.isp,10840]
        # print([ self.info["bandwidths"][node] for node in self.short_paths[6402] ])
        return paths


        # first sort the nodes by payment
        # then make a bandwidth remaining dict
        # go through ordered by payment and add in the path and update remaining bandwidth unless there isnt enough
    def rich_first(self):
        paths = {}
        sorted_clients = sorted(self.info["list_clients"], key=lambda x: self.info["payments"][x])
        remaining_band = self.info["bandwidths"].copy()

        for c in sorted_clients:
            enough = True
            for n in self.short_paths[c]:
                if not remaining_band[n]:
                    enough = False
                    break

            if enough:
                for n in self.short_paths[c]:
                    remaining_band[n] -= 1

                paths[c] = self.short_paths[c]
            else:
                paths[c] = [self.isp,c]

            return paths

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        return self.rich_first(), {}, {}
        # return ({},{},{})
