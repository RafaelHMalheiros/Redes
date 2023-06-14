import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict, deque
from matplotlib.animation import FuncAnimation

class Graph:
    def __init__(self):
        self.graph = nx.Graph()

    def add_edge(self, u, v):
        self.graph.add_edge(u, v)

    def bfs(self, start_vertex):
        visited = set()
        distances = {}
        queue = deque([(start_vertex, 0)])
        visited.add(start_vertex)
        distances[start_vertex] = 0
        self.graph.nodes[start_vertex]['color'] = 'lightgreen'  # Marcar o vértice inicial como visitado
        self.graph.nodes[start_vertex]['distance'] = 0  # Atribuir distância zero ao vértice inicial

        # Lista para armazenar os estados do grafo durante o BFS
        graph_states = [self.graph.copy()]

        while queue:
            vertex, distance = queue.popleft()

            for neighbor in self.get_neighbors(vertex):
                if neighbor not in visited:
                    queue.append((neighbor, distance + 1))
                    visited.add(neighbor)
                    distances[neighbor] = distance + 1
                    self.graph.nodes[neighbor]['color'] = 'lightgreen'  # Marcar o vértice como visitado
                    self.graph.edges[(vertex, neighbor)]['color'] = 'lightgreen'  # Marcar a aresta como visitada
                    self.graph.nodes[neighbor]['distance'] = distance + 1  # Atribuir a distância ao vértice
                    graph_states.append(self.graph.copy())  # Armazenar o estado atual do grafo

        # Chamada para visualizar o BFS em animação
        visualize_bfs_animation(graph_states, distances)

        return distances

    def get_neighbors(self, vertex):
        neighbors = set()

        for u, v in self.graph.edges():
            if u == vertex:
                neighbors.add(v)
            elif v == vertex:
                neighbors.add(u)

        return neighbors

def visualize_bfs_animation(graph_states, distances):
    fig, ax = plt.subplots()

    pos = nx.spring_layout(graph_states[0])

    def update_frame(i):
        ax.clear()
        graph_state = graph_states[i]

        # Desenhar os vértices
        for node, node_attr in graph_state.nodes(data=True):
            color = node_attr.get('color', 'lightgray')
            nx.draw_networkx_nodes(graph_state, pos, nodelist=[node], node_color=color, ax=ax)

        # Desenhar as arestas
        for u, v, edge_attr in graph_state.edges(data=True):
            color = edge_attr.get('color', 'lightgray')
            nx.draw_networkx_edges(graph_state, pos, edgelist=[(u, v)], edge_color=color, ax=ax)

        # Desenhar as etiquetas dos vértices com as distâncias
        labels = {}
        for node, node_attr in graph_state.nodes(data=True):
            label = f"{node}"
            if 'distance' in node_attr:
                label += f"\n({node_attr['distance']})"
            labels[node] = label
        nx.draw_networkx_labels(graph_state, pos, labels=labels, ax=ax)

        ax.set_title(f"Passo {i+1} do BFS")

    # Criação da animação do BFS
    ani = FuncAnimation(fig, update_frame, frames=len(graph_states), interval=1000, repeat=False)

    plt.show()

g = Graph()

num_vertices = int(input("Digite o número de vértices: "))

print("Digite os nomes dos vértices:")
vertices = []
for _ in range(num_vertices):
    vertex = input()
    vertices.append(vertex)
    g.graph.add_node(vertex)

num_edges = int(input("Digite o número de arestas: "))

print("Digite as arestas no formato 'vertice1 vertice2':")
for _ in range(num_edges):
    u, v = input().split()
    g.add_edge(u, v)
    g.graph.add_edge(u, v)

start_vertex = input("Digite o vértice inicial para a busca em largura: ")

distances = g.bfs(start_vertex)

print("Distâncias a partir do vértice", start_vertex + ":")
for vertex, distance in distances.items():
    print("Vértice:", vertex, "- Distância:", distance)
