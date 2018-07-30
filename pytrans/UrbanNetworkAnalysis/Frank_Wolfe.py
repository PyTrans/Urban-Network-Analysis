import networkx as nx
import scipy.integrate as integrate 
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt

from PyTrans.UrbanNetworkAnalysis import TransportationNetworks as tn

class Run:
    """
    Class of implementing Frank-Wolfe algorithm for networks privided from 
    Transportation Networks for Research Core Team (https://github.com/bstabler/TransportationNetworks)
    
    Parameters
    ----------
    link_file :     string
                    file path of network file, which containing various link information
                    
    trip_file :     string
                    file path of trip table. An Origin label and then Origin node number, followed by Destination node numders and OD flow
                    
    node_file :     string
                    file path of node file, which containing coordinates information of nodes
                    
    SO:             boolean
                    True if objective is to find system optimal solution,
                    False if objective is to find user equilibrium
                    
    Attributes
    ----------
    graph:          networkx DiGraph
                    graph of links when completing the algorithm
                    
    network:        nested dictionary
                    dictionary of links information and history of Frank-Wolfe algorithm implementation by iteration
    
    fwResult:       dictionary
                    dictionary of theta (optimal move size) and objective function value over iterations
                    
    Example
    -------
    A Quick example
    
    #Set the paths of Transportation Networks file
    
    >>> directory = ".\\Data\\TransportationNetworks\\SiouxFalls\\" 
    >>> link_file = '{}SiouxFalls_net.tntp'.format(directory) 
    >>> trip_file = '{}SiouxFalls_trips.tntp'.format(directory) 
    >>> node_file = '{}SiouxFalls_node.tntp'.format(directory)
    >>> SO = False \n
    
    #Implement Frank-Wolfe algorithm
    
    >>> fw = Run(link_file, trip_file, node_file, SO)
    >>> fw.showODFlow()
    >>> fw.showODFlowMap()
    """
    def __init__(self, link_file, trip_file, node_file, SO):
        self.SO = SO
        
        nw = tn.Network(link_file, trip_file, node_file, self.SO)

        ## initialization
        self.network = {(u,v): {'t0':d['object'].t0, 'alpha':d['object'].alpha, \
                   'beta':d['object'].beta, 'capa':d['object'].capacity, 'flow':[], \
                   'auxiliary':[], 'cost':[]} for (u, v, d) in nw.graph.edges(data=True)}
        
        self.fwResult = {'theta':[], 'z':[]}
        
        nw.all_or_nothing_assignment()
        nw.update_linkcost()
        
        for linkKey, linkVal in self.network.items():
            linkVal['cost'].append(nw.graph[linkKey[0]][linkKey[1]]['weight'])
            linkVal['auxiliary'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
            linkVal['flow'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
            
        ## iterations
        iterNum=0
        iteration = True
        while iteration:
            iterNum += 1
            nw.all_or_nothing_assignment()
            nw.update_linkcost()
            
            for linkKey, linkVal in self.network.items():
                linkVal['auxiliary'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
                
            theta = self.lineSearch()
            self.fwResult['theta'].append(theta)
            
            for linkKey, linkVal in self.network.items():
                aux = linkVal['auxiliary'][-1]
                flow = linkVal['flow'][-1]
                linkVal['flow'].append(flow + theta*(aux-flow))
                
                nw.graph[linkKey[0]][linkKey[1]]['object'].vol =  flow + theta * (aux - flow)
                nw.graph[linkKey[0]][linkKey[1]]['object'].flow = flow + theta * (aux - flow)
                
            
            nw.update_linkcost()
            
            z=0
            for linkKey, linkVal in self.network.items():
                linkVal['cost'].append(nw.graph[linkKey[0]][linkKey[1]]['weight'])
                totalcost = nw.graph[linkKey[0]][linkKey[1]]['object'].get_objective_function()
                z+=totalcost
                
            self.fwResult['z'].append(z)        
            
            if iterNum == 1:
                iteration = True
            else:
                if abs(self.fwResult['z'][-2] - self.fwResult['z'][-1]) <= 0.001 or iterNum==3000:
                    iteration = False
            
        self.graph = nw.graph
                    
    def BPR(self, t0, xa, ca, alpha, beta):
        """
        Method for calculating link travel time based on BPR function
        
        Parameters
        ----------
        t0:     float
                link travel time under free flow speed
                
        xa:     float
                traffic link flow
        
        ca:     float
                capacity of link
                
        alpha:  float
                first BPR function parameter, usually 0.15
                        
        beta:   float
                second BPR function parameter, usually 4.0
                
        Return
        ------
        ta:     float
                link travel time under the current traffic flow
        """
        ta = t0*(1+alpha*(xa/ca)**beta)
        return ta
    
    def calculateZ(self, theta):
        """
        Method for calculating objective function value
        
        Parameters
        ----------
        theta:      float
                    optimal move size
                    
        Return
        ------
        float
            objective function value
                    
        """
        z = 0
        for linkKey, linkVal in self.network.items():
            t0 = linkVal['t0']
            ca = linkVal['capa']
            beta = linkVal['beta']
            alpha = linkVal['alpha']
            aux = linkVal['auxiliary'][-1]
            flow = linkVal['flow'][-1]
            
            if SO == False:
                z += integrate.quad(lambda x: self.BPR(t0, x, ca, alpha, beta), 0, flow+theta*(aux-flow))[0]
            elif SO == True:
                z += list(map(lambda x : x * self.BPR(t0, x, ca, alpha, beta), [flow+theta*(aux-flow)]))[0]
        return z
    
    def lineSearch(self):
        """
        Method for estimating theta (optimal move size)
        
        Return
        ------
        float
            optimal move size (rate) between 0 and 1
        """
        theta = minimize_scalar(lambda x: self.calculateZ(x), bounds = (0,1), method = 'Bounded')
        return theta.x
    
    def showODFlow(self):
        """
        Method for presenting table of the optimal traffic assignment of the Frank-Wolfe algorithm procedure
        """
        for (u, v, d) in self.graph.edges(data=True):
            print(u, ' -> ', v, ': ', d['object'].vol)
    
    def showODFlowMap(self):
        """
        Method for presenting the traffic assignment result on a map
        """
        edgewidth = [d['object'].vol/5000 for (u, v, d) in self.graph.edges(data=True)]
    
        if node_file != None:
            plt.figure(num = 1, figsize=(10,10))
            plt.axis('off')
            
            pos = nx.get_node_attributes(self.graph, "pos")
        
            nx.draw_networkx_edges(self.graph, pos, width=edgewidth)
            nx.draw_networkx_edge_labels(self.graph, pos, \
                                         edge_labels={(u, v): round(d["object"].vol,0) for u, v, d in self.graph.edges(data=True)}, \
                                         font_size=8, label_pos=0.3, alpha=0.)
            nx.draw_networkx_nodes(self.graph, pos, with_labels=True)
            nx.draw_networkx_labels(self.graph, pos, font_size=10)
        
            plt.show()
            

if __name__ == "__main__":
    directory = ".\\Data\\TransportationNetworks\\SiouxFalls\\"
    link_file = '{}SiouxFalls_net.tntp'.format(directory)
    trip_file = '{}SiouxFalls_trips.tntp'.format(directory)
    node_file = '{}SiouxFalls_node.tntp'.format(directory)
    SO = False
    
    fw = Run(link_file, trip_file, node_file, SO)
    fw.showODFlow()
    fw.showODFlowMap()
