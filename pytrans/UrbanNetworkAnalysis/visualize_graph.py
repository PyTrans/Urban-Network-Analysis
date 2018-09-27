import networkx as nx
import plotly.plotly as py
import plotly
from plotly.graph_objs import *
from shapely.geometry import mapping, Polygon, Point, LineString, shape
import fiona
import random
import sympy



def NeworkX2Pyplot(graph, savefilename=None):
    import matplotlib.pyplot as plt
    plt.figure(1)
    # plt.subplot(211);
    # plt.axis('off')

    for fnode in graph:
        for enode in graph[fnode]:
            graph[fnode][enode]["volume"] = graph[fnode][enode]["object"].flow
    volumes = nx.get_edge_attributes(graph, "volume")
    volumes = list(volumes.values())
    max_volume = max(volumes)
    pos = nx.get_node_attributes(graph, "pos")
    edgewidth = 0.5
    nx.draw_networkx_edges(graph, pos, width=edgewidth, label=volumes)
    nx.draw_networkx_nodes(graph, pos, with_labels=True)
    # labels = bch.G.node.keys()
    nx.draw_networkx_labels(graph, pos, font_size=5)
    nx.draw_networkx_edge_labels(graph, pos=nx.spring_layout(graph))
    plt.show()
    if savefilename!=None:
        plt.savefig(savefilename)

def NetworkX2Plotly(graph, username='nds1027', apikey='d58YAwDrBrZlRmpnoMGe'):

    plotly.tools.set_credentials_file(username=username, api_key=apikey)

    edge_trace = Scatter(
        x=[],
        y=[],
        line=Line(width=[], color='#888'),
        text=[],
        hoverinfo='none',
        mode='lines')

    for edge in graph.edges():
        # x0, y0 = graph.node[edge[0]]['pos']
        # x1, y1 = graph.node[edge[1]]['pos']
        volumes = nx.get_edge_attributes(graph, "volume")
        volumes = list(volumes.values())
        max_volume = max(volumes)
        x0, y0 = graph[edge[0]][edge[1]]["pos_fnode"][0], graph[edge[0]][edge[1]]["pos_fnode"][1]
        x1, y1 = graph[edge[0]][edge[1]]["pos_tnode"][0], graph[edge[0]][edge[1]]["pos_tnode"][1]
        edge_trace['x'] += [x0, x1, None]
        edge_trace['y'] += [y0, y1, None]
        volume = graph[edge[0]][edge[1]]["volume"]
        edge_trace['line']['width'].append(volume / max_volume * 1000.0)
        edge_trace['text'].append(str(volume))
        print(edge)

    node_trace = Scatter(
        x=[],
        y=[],
        text=[],
        mode='markers',
        hoverinfo='text',
        marker=Marker(
            # showscale=True,
            # colorscale options
            # 'Greys' | 'Greens' | 'Bluered' | 'Hot' | 'Picnic' | 'Portland' |
            # Jet' | 'RdBu' | 'Blackbody' | 'Earth' | 'Electric' | 'YIOrRd' | 'YIGnBu'
            colorscale='YIGnBu',
            reversescale=True,
            color=[],
            size=10,
            colorbar=dict(
                thickness=15,
                title='volume',
                xanchor='left',
                titleside='right'
            ),
            line=dict(width=2)))
    for node in graph.nodes():
        x, y = graph.node[node]['pos']
        node_trace['x'].append(x)
        node_trace['y'].append(y)

    fig = Figure(data=Data([edge_trace, node_trace]),
                 layout=Layout(
                     title='<br>Network graph made with Python',
                     titlefont=dict(size=16),
                     showlegend=False,
                     hovermode='closest',
                     margin=dict(b=20, l=5, r=5, t=40),
                     annotations=[dict(
                         text="Python code: <a href='https://plot.ly/ipython-notebooks/network-graphs/'> https://plot.ly/ipython-notebooks/network-graphs/</a>",
                         showarrow=False,
                         xref="paper", yref="paper",
                         x=0.005, y=-0.002)],
                     xaxis=XAxis(showgrid=False, zeroline=False, showticklabels=False),
                     yaxis=YAxis(showgrid=False, zeroline=False, showticklabels=False)))

    py.plot(fig, filename='networkx')


def NetworkX2_Nodes_Shapefile(graph, filename=None, schema=None):

    nodeset = graph.nodes()
    if schema==None:
        properties={"id":"str"}
        for r in range(10):
            index = random.randint(0,len(nodeset)-1)
            for p in graph.node[list(nodeset)[index]]:
                if p not in properties:
                    dtype= p
                    if isinstance(dtype,int):
                        properties[p]="int"
                    elif isinstance(dtype,float):
                        properties[p] = "float"
                    elif isinstance(dtype,str):
                        properties[p] = "str"
        schema = {
            'geometry': 'Point',
            #'properties': properties,
            'properties': {"id":"int"},
        }

    # Write a new Shapefile
    if filename==None:
        nodefilename= "nodes.shp"
    else:
        nodefilename = filename+"_nodes.shp"
    with fiona.open(nodefilename, 'w', 'ESRI Shapefile', schema) as c:
        ## If there are multiple geometries, put the "for" loop here
        for n in nodeset:
            pos = graph.node[n]["pos"]
            eproperty={}
            for p in properties:
                eproperty[p]=None
            for p in graph.node[n]:
                dtype= graph.node[n][p]
                if isinstance(dtype,int):
                    eproperty[p]= graph.node[n][p]
                elif isinstance(dtype,float):
                    eproperty[p] = graph.node[n][p]
                elif isinstance(dtype, str):
                    eproperty[p] = str(graph.node[n][p])


            eproperty["id"]=n
            c.write({
                'geometry': Point([pos]),
                'properties': {'id':1},
            })
            print(n, pos)

def NetworkX2_Links_Shapefile(graph, filename=None, schema=None):
    linkset = graph.edges()
    for fnode, tnode in  graph.edges():
        graph[fnode][tnode]["volume"]=graph[fnode][tnode]["object"].flow
    volumes = nx.get_edge_attributes(graph, "volume")
    volumes = list(volumes.values())
    max_volume = max(volumes)

    if schema==None:
        properties={"id":"str"}
        for r in range(10):
            index = random.randint(0,len(linkset)-1)
            for p in graph.edges[list(linkset)[index]]:
                if p not in properties:
                    dtype = graph.edges[list(linkset)[index]][p]
                    try:
                        dtype = float(dtype)
                    except:
                        pass
                    if isinstance(dtype,int):
                        properties[p]="int"
                    elif isinstance(dtype,float):
                        properties[p] = "float"
                    elif isinstance(dtype,str):
                        properties[p] = "str"


        schema = {
            'geometry': "LineString",
            'properties': properties,
            #'properties': {"id":"int"},
        }

    # Write a new Shapefile
    if filename==None:
        linkfilename= "links.shp"
    else:
        linkfilename = filename+"_links.shp"
    with fiona.open(linkfilename, 'w', 'ESRI Shapefile', schema) as c:
        ## If there are multiple geometries, put the "for" loop here
        for (fnode, enode) in linkset:
            f_pos = graph.node[fnode]["pos"]
            e_pos = graph.node[enode]["pos"]
            x0, y0 = graph[fnode][enode]["pos_fnode"][0], graph[fnode][enode]["pos_fnode"][1]
            x1, y1 = graph[fnode][enode]["pos_tnode"][0], graph[fnode][enode]["pos_tnode"][1]
            eproperty={}
            poly1 = LineString([(0, 0), (0, 1), (1, 1)])
            poly1 = LineString([(x0, y0), (x1,y1)])
            for p in properties:
                eproperty[p]=None
            for p in graph[fnode][enode]:
                dtype= graph[fnode][enode][p]
                try:
                    dtype = float(dtype)
                except:
                    pass
                if isinstance(dtype,int):
                    eproperty[p]= dtype
                elif isinstance(dtype,float):
                    eproperty[p] = dtype
                elif isinstance(dtype, str):
                    eproperty[p] = dtype
                else:
                    pass


            eproperty["id"]=str(fnode)+"_"+str(enode)
            c.write({
                'geometry': mapping(poly1),
                'properties': eproperty,
            })

