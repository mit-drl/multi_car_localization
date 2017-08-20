import networkx as nx

def convert(con_dict):
	G = nx.DiGraph()
	for key in con_dict:
		G.add_node(int(key))
	for key in con_dict:
		connections = con_dict[key]
		for connection in connections:
			if int(key) != connection:
				G.add_edge(int(key), connection)
	return G

def prune(graph, ID):
	G = graph.copy()
	for node in graph.nodes():
		if node not in graph.neighbors(ID) and node != ID:
			G.remove_node(node)
	return G

