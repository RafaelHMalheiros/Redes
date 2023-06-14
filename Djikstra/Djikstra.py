import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
from matplotlib.animation import FuncAnimation
import math

class Graph:
    def __init__(self):
        self.graph = nx.Graph()

    def add_edge(self, u, v, weight):
        self.graph.add_edge(u, v, weight=weight)

    def dijkstra(self, start_vertex):
        visited = set()
        distances = defaultdict(lambda: math.inf)
        distances[start_vertex] = 0
        self.graph.nodes[start_vertex]['color'] = 'lightgreen'  # Mark the initial vertex as visited
        self.graph.nodes[start_vertex]['distance'] = 0  # Assign distance zero to the initial vertex

        # List to store the graph states during Dijkstra's algorithm
        graph_states = [self.graph.copy()]

        while len(visited) < len(self.graph.nodes):
            # Find the vertex with the minimum distance
            min_distance = math.inf
            min_vertex = None
            for v in self.graph.nodes:
                if v not in visited and distances[v] < min_distance:
                    min_distance = distances[v]
                    min_vertex = v

            if min_vertex is None:
                break

            visited.add(min_vertex)

            for neighbor in self.get_neighbors(min_vertex):
                if neighbor not in visited:
                    edge_weight = self.graph.edges[(min_vertex, neighbor)]['weight']
                    new_distance = distances[min_vertex] + edge_weight
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        self.graph.nodes[neighbor]['color'] = 'lightgreen'  # Mark the vertex as visited
                        self.graph.edges[(min_vertex, neighbor)]['color'] = 'lightgreen'  # Mark the edge as visited
                        self.graph.nodes[neighbor]['distance'] = new_distance  # Assign the distance to the vertex
                        graph_states.append(self.graph.copy())  # Store the current state of the graph

        # Call to visualize Dijkstra's algorithm in animation
        visualize_dijkstra_animation(graph_states, distances)

        return distances

    def get_neighbors(self, vertex):
        neighbors = set()

        for u, v in self.graph.edges():
            if u == vertex:
                neighbors.add(v)
            elif v == vertex:
                neighbors.add(u)

        return neighbors

def visualize_dijkstra_animation(graph_states, distances):
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

        ax.set_title(f"Step {i+1} of Dijkstra's algorithm")

    # Create the animation of Dijkstra's algorithm
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
    g.add_edge(u, v, int(weight))
    g.graph.add_edge(u, v, weight=int(weight))

start_vertex = input("Enter the starting vertex for Dijkstra's algorithm: ")

distances = g.dijkstra(start_vertex)

print("Distances from vertex", start_vertex + ":")
for vertex, distance in distances.items():
    print("Vertex:", vertex, "- Distance:", distance)
