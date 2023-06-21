import argparse
import trimesh
import openmesh as om
import numpy as np
import os

tolerance = 1e-1


# class for adjacency matrix representation of the graph
class Graph():
    def __init__(self, vertices):
        self.V = len(vertices)
        self.vertices = vertices
        self.graph = np.zeros((self.V, self.V))
 
    def printSolution(self, dist, path):
        print("Vertex \t Distance from Source")
        for node in range(self.V):
            print(node, "\t\t", dist[node])
 
    def minDistance(self, dist, sptSet):
        min = 1e7
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
        return min_index

    def dijkstra(self, src):
        dist = [1e7] * self.V
        path = [np.inf] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for count in range(self.V):
            u = self.minDistance(dist, sptSet)
            sptSet[u] = True
            for v in range(self.V):
                if (self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
                    path[count] = u
        self.printSolution(dist, path)

def generate_directions(min_phi=0, max_phi=np.pi, min_theta=0, max_theta=2*np.pi):
    directions = []
    for phi in np.linspace(min_phi, max_phi, 10):
        for theta in np.linspace(min_theta, max_theta, 10):
            directions.append(np.array([np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta), np.cos(phi)]))
    # return directions
    return [directions[0]]

def estimate_length(mesh, faces):
    # Dijkstra to get the shortest path
    vertices = set()
    for fh in faces:
        vertices.update([vh.idx() for vh in mesh.fv(fh)])
    g = Graph(vertices)
    vertice = list(vertices)
    adjecency = np.zeros((len(vertices), len(vertices)))
    for i, vid0 in enumerate(vertices):
        for j, vid1 in enumerate(vertices):
            for fh in faces:
                face_vertices = [vh.idx() for vh in mesh.fv(fh)]
                if vid1 in face_vertices and vid0 in face_vertices:
                    point0 = mesh.points()[vid0]
                    point1 = mesh.points()[vid1]
                    adjecency[i][j] = np.linalg.norm(point1 - point0)
                    adjecency[j][i] = np.linalg.norm(point1 - point0)
    g.graph = adjecency
    print(adjecency)
    g.dijkstra(0)
    return len(faces)

def main(file_name, visualize):
    mesh = om.read_trimesh(file_name)
    mesh.request_face_normals()
    mesh.update_normals()
    mesh.request_face_colors()
    face_normals = mesh.face_normals()

    isoline_lengths = []
    directions = generate_directions()
    for j, direction in enumerate(directions):
        perpendicular_faces = []
        for i, face in enumerate(mesh.faces()):
            if np.abs(np.dot(face_normals[i], direction)) < tolerance:
                perpendicular_faces.append(face)
        isoline_lengths.append(estimate_length(mesh, perpendicular_faces))

    min_direction = directions[isoline_lengths.index(max(isoline_lengths))]
    print(min_direction, "direction chosen")
    for i, face in enumerate(mesh.faces()):
            if np.abs(np.dot(face_normals[i], min_direction)) < tolerance:
                mesh.set_color(face, [0.6, 0.3, 0.3, 0])
            else:
                mesh.set_color(face, [0.3, 0.6, 0.3, 0])
    name, ext = os.path.splitext(file_name)
    om.write_mesh(name + '_labeled.obj', mesh, face_color=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Find shortest isocline')
    parser.add_argument('file_name',
                        help='Input file name is required. Enter it!')
    parser.add_argument('--visualize', required=False, action='store_true')
    parser.set_defaults(visualize=False)
    args = parser.parse_args()

    main(args.file_name, args.visualize)
