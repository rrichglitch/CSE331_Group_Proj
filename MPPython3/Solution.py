import heapq
import sys

class Solution:

    def __init__(self, problem, isp, graph, info):
        self.problem = problem
        self.isp = isp
        self.graph = graph
        self.info = info
        self.node2downStream = [set() for _ in range(len(self.graph))]
        self.current_path_acceptable = [-1] * len(self.graph)
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

    def extra_path_delay(self, node):  # O(1)
        return len(self.node2downStream[node]) // self.info["bandwidths"][node]

    # node2downStream must be populated before calling this method
    def dijkstras(self, grumps):  # grumps are the nodes to return paths for
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

    # This gives a heuristic score based on pmt, path_len, delay, acceptability, and complaint threshold O(n)
    def score_current_path(self, path):  # call only on client paths
        node = path[len(path)-1]

        base_delay = len(self.short_paths[node])-1
        extra_delay = max([ self.extra_path_delay(path[i]) for i in range(len(path)-1) ])
        cur_delay = base_delay+extra_delay

        self.current_path_acceptable[node] = cur_delay <= (base_delay * self.info["alphas"][node])
        self.current_path_compliant[node] = cur_delay <= (base_delay * self.info["betas"][node])

        money = self.info["payments"][node] * self.current_path_acceptable[node]

        # Penalize if complaint threshold is not met
        penalty = 0
        if not self.current_path_compliant[node]:
            
            penalty = money  

        return (money - penalty) / cur_delay if cur_delay != 0 else 0

    # Check if the path is acceptable based on alpha
    def check_path_acceptable(self, path):
        node = path[-1]

        if self.current_path_acceptable[node] >= 0:
            return self.current_path_acceptable[node]

        base_delay = len(self.short_paths[node]) - 1
        extra_delay = max([self.extra_path_delay(path[i]) for i in range(len(path) - 1)])

        return (base_delay + extra_delay) <= (base_delay * self.info["alphas"][node])

    # New method to check complaint threshold based on beta
    def check_complaint_threshold(self, path):
        node = path[len(path)-1]

        if self.current_path_compliant[node] >= 0:
            return self.current_path_compliant[node] # any new paths will already be cached during scoring

        base_delay = len(self.short_paths[node])-1
        extra_delay = max([ self.extra_path_delay(path[i]) for i in range(len(path)-1) ])

        return (base_delay+extra_delay) <= (base_delay * self.info["betas"][node])

    

    # Update delays and acceptability for given clients
    def update_delays(self, clients):
        for c in clients:
            path = self.paths[c]
            base_delay = len(self.short_paths[c]) - 1
            extra_delay = max([self.extra_path_delay(path[i]) for i in range(len(path) - 1)])
            cur_delay = base_delay + extra_delay

            
            self.current_path_acceptable[c] = cur_delay <= (base_delay * self.info["alphas"][c])

            
            self.current_path_compliant[c] = cur_delay <= (base_delay * self.info["betas"][c])

    # to begin we run our MST algo
    # next we iterate through the paths created by the MST and check for bandwidth delays
    # we sort our nodes which are unhappy with bandwidth delay by pmt and/or path len
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

        self.paths = paths  # Store the final paths

        
        total_bandwidth_cost = 0

        
        self.update_delays(self.C)
        complaining_clients = [c for c in self.C if not self.current_path_compliant[c]]

        # FCC penalty
        complaining_S = [c for c in complaining_clients if c in self.S]
        threshold_fcc = int(self.rho_fcc * len(self.S))
        if len(complaining_S) > threshold_fcc and self.fcc_fine > 0:
            # reduce number of complaining clients in S
            penalty_amount = self.fcc_fine
            budget = self.fcc_fine  # Maximum amount weare willing to spend
            total_spent = 0
            while len(complaining_S) > threshold_fcc and total_spent < budget:
                # try to fix thier delay
                for c in complaining_S:
                    path = self.paths[c]
                    # find nodes along path to increase bandwidth
                    nodes_to_consider = path[:-1]  # Exclude client node
                    nodes_to_consider = sorted(nodes_to_consider, key=lambda u: self.extra_path_delay(u), reverse=True)
                    for u in nodes_to_consider:
                        
                        self.info["bandwidths"][u] += 1
                        total_bandwidth_cost += self.cost_bandwidth
                        total_spent += self.cost_bandwidth
                        # Update delays for clients impacted by node u
                        impacted_clients = list(self.node2downStream[u])
                        self.update_delays(impacted_clients)
                        # calculating again complaining clients in S
                        complaining_S = [c for c in self.S if not self.current_path_compliant[c]]
                        if len(complaining_S) <= threshold_fcc or total_spent >= budget:
                            break
                    if len(complaining_S) <= threshold_fcc or total_spent >= budget:
                        break

        # lawsuit penalty
        complaining_C = [c for c in self.C if not self.current_path_compliant[c]]
        threshold_lawsuit = int(self.rho_lawsuit * len(self.C))
        if len(complaining_C) > threshold_lawsuit and self.lawsuit_amount > 0:
            # reduce number of complaining clients in C
            penalty_amount = self.lawsuit_amount
            budget = self.lawsuit_amount  # Maximum we are willing to spend
            total_spent = 0
            while len(complaining_C) > threshold_lawsuit and total_spent < budget:
                # try to fix their delay
                for c in complaining_C:
                    path = self.paths[c]
                    
                    nodes_to_consider = path[:-1]  
                    nodes_to_consider = sorted(nodes_to_consider, key=lambda u: self.extra_path_delay(u), reverse=True)
                    for u in nodes_to_consider:
                        # decide whether to increase bandwidth at node u
                        self.info["bandwidths"][u] += 1
                        total_bandwidth_cost += self.cost_bandwidth
                        total_spent += self.cost_bandwidth
                        # Update delays 
                        impacted_clients = list(self.node2downStream[u])
                        self.update_delays(impacted_clients)
                        # calcautking again complaining clients in C
                        complaining_C = [c for c in self.C if not self.current_path_compliant[c]]
                        if len(complaining_C) <= threshold_lawsuit or total_spent >= budget:
                            break
                    if len(complaining_C) <= threshold_lawsuit or total_spent >= budget:
                        break

        return self.paths

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        paths = self.improve_MST()
        # For problems 3, 4, 5, return updated bandwidths
        updated_bandwidths = self.info["bandwidths"]
        # For problems 4 and 5, return priorities if needed. Assuming this is Problem 3, return empty dict
        priorities = {}
        return paths, updated_bandwidths, priorities
