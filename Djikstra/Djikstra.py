import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from matplotlib.animation import FuncAnimation

class Graph:
    def __init__(self):
        self.graph = nx.Graph()

    def add_edge(self, u, v, weight):
        self.graph.add_edge(u, v, weight=weight)

    def bellman_ford(self, start_vertex):
        distances = {}
        predecessors = {}
        self.graph.nodes[start_vertex]['color'] = 'lightgreen'  # Mark the starting vertex as visited
        self.graph.nodes[start_vertex]['distance'] = 0  # Assign distance zero to the starting vertex

        # List to store the graph states during the Bellman-Ford algorithm
        graph_states = [self.graph.copy()]

        distances[start_vertex] = 0  # Add an entry for the starting vertex in the distances dictionary
        predecessors[start_vertex] = None

        # Initialize all distances with infinity except the starting vertex
        for node in self.graph.nodes():
            if node != start_vertex:
                distances[node] = float('inf')

        # Relax edges repeatedly to find the shortest paths
        for _ in range(len(self.graph.nodes()) - 1):
            for u, v, edge_data in self.graph.edges(data=True):
                weight = edge_data['weight']
                if distances[u] + weight < distances[v]:
                    distances[v] = distances[u] + weight
                    predecessors[v] = u
                    self.graph.nodes[v]['color'] = 'lightgreen'  # Mark the vertex as visited
                    self.graph.edges[(u, v)]['color'] = 'lightgreen'  # Mark the edge as visited
                    graph_states.append(self.graph.copy())  # Store the current state of the graph

        # Check for negative cycles
        for u, v, edge_data in self.graph.edges(data=True):
            weight = edge_data['weight']
            if distances[u] + weight < distances[v]:
                raise ValueError("Graph contains negative weight cycle")

        # Call the visualization function to animate the Bellman-Ford algorithm
        visualize_bellman_ford_animation(graph_states, distances, predecessors)

        return distances, predecessors

    def get_neighbors(self, vertex):
        neighbors = set()

        for u, v in self.graph.edges():
            if u == vertex:
                neighbors.add(v)
            elif v == vertex:
                neighbors.add(u)

        return neighbors

def visualize_bellman_ford_animation(graph_states, distances, predecessors):
    fig, ax = plt.subplots()

    pos = nx.spring_layout(graph_states[0])

    def update_frame(i):
        ax.clear()
        graph_state = graph_states[i]

        # Draw the vertices
        for node, node_attr in graph_state.nodes(data=True):
            color = node_attr.get('color', 'lightgray')
            nx.draw_networkx_nodes(graph_state, pos, nodelist=[node], node_color=color, ax=ax)

        # Draw the edges
        for u, v, edge_attr in graph_state.edges(data=True):
            color = edge_attr.get('color', 'lightgray')
            nx.draw_networkx_edges(graph_state, pos, edgelist=[(u, v)], edge_color=color, ax=ax)

        # Draw vertex labels with distances
        labels = {}
        for node, node_attr in graph_state.nodes(data=True):
            label = f"{node}"
            if 'distance' in node_attr:
                label += f"\n({node_attr['distance']})"
            labels[node] = label
        nx.draw_networkx_labels(graph_state, pos, labels=labels, ax=ax)

        # Draw edge labels with weights
        edge_labels = {}
        for u, v, edge_attr in graph_state.edges(data=True):
            if 'weight' in edge_attr:
                edge_labels[(u, v)] = str(edge_attr['weight'])
        nx.draw_networkx_edge_labels(graph_state, pos, edge_labels=edge_labels, ax=ax)

        ax.set_title(f"Step {i+1} of the Bellman-Ford algorithm")

    # Create the Bellman-Ford animation
    ani = FuncAnimation(fig, update_frame, frames=len(graph_states), interval=1000, repeat=False)

    plt.show()

g = Graph()

num_vertices = int(input("Enter the number of vertices: "))

print("Enter the names of the vertices:")
vertices = []
for _ in range(num_vertices):
    vertex = input()
    vertices.append(vertex)
    g.graph.add_node(vertex)

num_edges = int(input("Enter the number of edges: "))

print("Enter the edges in the format 'vertex1 vertex2 weight':")
for _ in range(num_edges):
    u, v, weight = input().split()
    weight = int(weight)
    g.add_edge(u, v, weight)
    g.graph.add_edge(u, v, weight=weight)

start_vertex = input("Enter the starting vertex for the Bellman-Ford algorithm: ")

distances, predecessors = g.bellman_ford(start_vertex)

print("Distances from the vertex", start_vertex + ":")
for vertex, distance in distances.items():
    print("Vertex:", vertex, "- Distance:", distance)

print("Predecessors:")
for vertex, predecessor in predecessors.items():
    print("Vertex:", vertex, "- Predecessor:", predecessor)
