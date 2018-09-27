import networkx as nx
import math
#import random
import matplotlib.pyplot as plt
import os
import sys
import pytrans.UrbanNetworkAnalysis.visualize_graph as v
from scipy.misc import derivative
import pytrans.UrbanNetworkAnalysis.TransportationNetworks as loadProblem

class GradientProjection(object):
    def __init__(self, network, ods, use_simpy, SO, tol=1e-4):
        self.network = network
        self.ods = ods
        self.step = 0.6
        self.total_trips = 0
        self.tol = tol
        self.use_simpy = use_simpy
        self.SO = SO
        self.edges = self.getedges()

    def getedges(self):
        edges = []
        for f in self.network.nodes():
            for t in self.network[f]:
                edges.append((f, t))
        return edges

    def solve(self):
        solution = {}

        by_id_links = {}
        for (node_from, node_to) in self.edges:
            l = self.network.get_edge_data(node_from, node_to)['object']
            l.flow = 0
            by_id_links[l.link_id] = l

        od_pairs = []

        for origin in self.ods:
            paths = nx.shortest_path(self.network, origin, weight='time')
            for destination in self.ods[origin]:
                od_pairs.append((origin, destination))
                node_list = paths[destination]
                link_list = []
                for ind in range(len(node_list) - 1):
                    node_from = node_list[ind]
                    node_to = node_list[ind + 1]
                    edge_data = self.network.get_edge_data(node_from, node_to)
                    edge_data['object'].flow += self.ods[origin][destination]
                    # by_id_links[edge_data['object'].link_id].flow += self.ods[origin][destination]
                    if self.SO == True:
                        edge_data['time'] = edge_data['object'].getmarginalcost_l(edge_data['object'].flow)
                        if edge_data['object'].getmarginalcost_l(edge_data['object'].flow) < 0:
                            raise ValueError
                    else:
                        edge_data['time'] = edge_data['object'].get_time()
                    link_list.append(edge_data['object'].link_id)

                self.total_trips += self.ods[origin][destination]

                solution[(origin, destination)] = {tuple(link_list): self.ods[origin][destination]}

        for link_id in by_id_links:
            edge_data = self.network.get_edge_data(by_id_links[link_id].from_node, by_id_links[link_id].to_node)
            if self.SO == True:
                edge_data['time'] = edge_data['object'].getmarginalcost_l(edge_data['object'].flow)
                # if edge_data['object'].getmarginalcost_l(edge_data['object'].flow)<0:
                #     raise ValueError
            else:
                edge_data['time'] = edge_data['object'].get_time()

        objetive_function_history = []
        for i in range(300):
            # random.shuffle(od_pairs)
            for (origin, destination) in od_pairs:
                # for origin in self.ods:
                #     for destination in self.ods[origin]:
                paths = nx.shortest_path(self.network, origin, weight='time')
                node_list = paths[destination]

                link_sequence = []
                for ind in range(len(node_list) - 1):
                    node_from = node_list[ind]
                    node_to = node_list[ind + 1]

                    edge_data = self.network.get_edge_data(node_from, node_to)
                    link = edge_data['object']
                    link_sequence.append(link.link_id)

                link_sequence = tuple(link_sequence)
                # costs, previous = dijkstra_helper.get_shortest_paths(origin)
                # link_sequence = dijkstra_helper.get_link_sequence(origin, destination, previous)
                # link_sequence = tuple(link_sequence)
                if not link_sequence in solution[(origin, destination)]:
                    solution[(origin, destination)][link_sequence] = 0.0

                derivatives = {}
                s_paths = {}
                for path in solution[(origin, destination)]:
                    e_derivative = 0.0
                    s = 0.0

                    for link_id in path:
                        link = by_id_links[link_id]
                        if self.use_simpy == False:
                            e_derivative += link.get_time()
                        else:
                            if self.SO == True:
                                e_derivative += link.getmarginalcost_l(link.flow)
                            else:
                                e_derivative += link.actualcost
                        if link_id not in link_sequence:
                            if self.use_simpy == True:
                                if self.SO == True:
                                    diff = derivative(link.getmarginalcost_l, link.flow)
                                    if diff < 0 or diff > 10000:
                                        print("%.6f" % diff)
                                        diff = 0.0001

                                    s += diff
                                else:
                                    s += derivative(link.getbprcost, link.flow)
                                    a = math.pow(link.flow,
                                                 link.beta - 1) * link.beta * link.alpha * link.length / (
                                            link.free_speed * math.pow(link.capacity, link.beta))
                                    b = derivative(link.getbprcost, link.flow)
                                    if a - b > 0.0001:
                                        print("%.10f" % (a - b))
                            else:
                                s += math.pow(link.flow, link.beta - 1) * link.beta * link.alpha * link.length / (
                                link.free_speed * math.pow(link.capacity, link.beta))  # 1/31/2018 DS

                    derivatives[path] = e_derivative

                    if path != link_sequence:
                        for link_id in link_sequence:
                            if link_id not in path:
                                if self.use_simpy == True:
                                    if self.SO == True:
                                        s += derivative(link.getmarginalcost_l, link.flow)
                                    else:
                                        s += derivative(link.getbprcost, link.flow)
                                        a = math.pow(link.flow,
                                                     link.beta - 1) * link.beta * link.alpha * link.length / (
                                                link.free_speed * math.pow(link.capacity, link.beta))
                                        b = derivative(link.getbprcost, link.flow)
                                        if a - b > 0.0001:
                                            print("%.10f" % (a - b))
                                else:
                                    s += math.pow(link.flow, link.beta - 1) * link.beta * link.alpha * link.length / (
                                        link.free_speed * math.pow(link.capacity, link.beta))
                        s_paths[path] = s
                        # s_paths[path] = 1.0

                total_flow = 0.0
                for path in s_paths:
                    try:
                        previous_flow = solution[(origin, destination)][path]
                    except:
                        previous_flow = 0.0

                    if derivatives[path] < derivatives[link_sequence]:
                        print('-- path', path)
                        print('--shortest', link_sequence)
                        print('--od', origin, destination)
                        print(derivatives[path], derivatives[link_sequence])

                        raise Exception

                    flow = solution[(origin, destination)][path] - self.step * (
                        derivatives[path] - derivatives[link_sequence]) / s_paths[path]

                    flow = max(0.0, flow)
                    total_flow += flow

                    if flow > 0:
                        solution[(origin, destination)][path] = flow
                    else:
                        del solution[origin, destination][path]

                    for link_id in path:
                        by_id_links[link_id].flow += (flow - previous_flow)
                        # if by_id_links[link_id].flow<0:
                        #     raise ValueError
                        edge_data = self.network.get_edge_data(by_id_links[link_id].from_node,
                                                               by_id_links[link_id].to_node)
                        # if by_id_links[link_id].flow != link.flow:
                        #     link.flow =by_id_links[link_id].flow
                        if self.SO == True:
                            edge_data['time'] = by_id_links[link_id].getmarginalcost_l(by_id_links[link_id].flow)
                        else:
                            edge_data['time'] = by_id_links[link_id].get_time()

                try:
                    previous_flow = solution[(origin, destination)][link_sequence]
                except:
                    previous_flow = 0.0

                solution[(origin, destination)][link_sequence] = self.ods[origin][destination] - total_flow
                if self.ods[origin][destination] - total_flow < 0:
                    raise ValueError
                for link_id in link_sequence:
                    by_id_links[link_id].flow += (solution[(origin, destination)][link_sequence] - previous_flow)
                    # if by_id_links[link_id].flow - total_flow < 0:
                    #     raise ValueError
                    edge_data = self.network.get_edge_data(by_id_links[link_id].from_node, by_id_links[link_id].to_node)
                    # if by_id_links[link_id].flow != link.flow:
                    #     link.flow = by_id_links[link_id].flow
                    if self.SO == True:
                        edge_data['time'] = max(0, by_id_links[link_id].getmarginalcost_l(by_id_links[link_id].flow))
                    else:
                        edge_data['time'] = max(0, by_id_links[link_id].get_time())

            objetive_function_history.append(self.convergence_test(solution, by_id_links))
            if objetive_function_history[-1] < self.tol:
                break

        return solution, by_id_links, objetive_function_history

    def get_bpr_objetive_function(self, solution):
        s = 0.0
        for edge in self.edges:
            s += self.network.get_edge_data(edge[0], edge[1])['object'].get_bpr_objective_function()

        return s
    def get_total_traveltime(self, solution):
        s = 0.0
        for edge in self.edges:
            s += self.network.get_edge_data(edge[0], edge[1])['object'].get_total_travel_time_function()

        return s

    def get_objetive_function_SO(self, solution):
        s = 0.0
        for edge in self.edges:
            # print ("obj SO", edge[0], edge[1], self.network.get_edge_data(edge[0], edge[1])['object'].flow, self.network.get_edge_data(edge[0], edge[1])['object'].get_objective_function(), self.network.get_edge_data(edge[0], edge[1])['object'].get_objective_function_SO())
            s += self.network.get_edge_data(edge[0], edge[1])['object'].get_objective_function_SO()

        return s

    def convergence_test(self, solution, by_link_id):

        total_gap = 0

        for origin in self.ods:

            for destination in self.ods[origin]:

                times = {}

                min_t = 10E5
                index_min = None
                for path in solution[origin, destination]:
                    t = 0
                    for link_id in path:
                        link = by_link_id[link_id]
                        t += link.get_time()

                    times[path] = t

                    if t < min_t:
                        min_t = t
                        index_min = path

                if len(solution[origin, destination]) <= 1:
                    continue

                for path in solution[origin, destination]:
                    total_gap += solution[origin, destination][path] * (times[path] - min_t)
        print("UE Objective:", self.get_bpr_objetive_function(solution), "SO Objective:",self.get_total_traveltime(solution))

        return total_gap / (self.total_trips)




datafolder="./Data/TransportationNetworks/SiouxFalls/"
link_file = datafolder+"SiouxFalls_net.tntp"
trip_file = datafolder+"SiouxFalls_trips.tntp"
node_file =datafolder+"SiouxFalls_node.tntp"


visualization = False
bch = loadProblem.Network(link_file, trip_file, node_file)
# link_fields = {"from": 0, "to": 1, "capacity": 2, "length": 3, "t0": 4, "B": 5, "beta": 6, "V": 7}
# bch.link_fields = link_fields
bch.SO=True
use_simpy = False
if bch.SO==True:
    use_simpy=True
# bch.open_link_file()
bch.open_trip_file()

graph, ods = bch.graph, bch.od_dic

gp = GradientProjection(graph, ods, use_simpy, bch.SO, 1e-3)

solution, by_id_links, objetive_function_history_UE = gp.solve()


