import argparse
import trimesh
import openmesh as om
import numpy as np
import os

tolerance = 1e-1
large_num = 1e7

# class for adjacency matrix representation of the graph and O(V^2) Dijkstra
class Graph():
    def __init__(self, n_verts):
        self.V = n_verts
        self.graph = np.zeros((self.V, self.V))
 
    def print_solution(self, dist, path):
        print("Vertex \t\t Distance from Source")
        for node in range(self.V):
            print(node, "\t\t", dist[node])
 
    def min_distance(self, dist, spt_set):
        min = large_num
        for v in range(self.V):
            if dist[v] < min and spt_set[v] == False:
                min = dist[v]
                min_index = v
        return min_index

    def dijkstra(self, src):
        dist = [large_num] * self.V
        dist[src] = 0
        path = [[]] * self.V
        spt_set = [False] * self.V
 
        for count in range(self.V):
            u = self.min_distance(dist, spt_set)
            spt_set[u] = True
            for v in range(self.V):
                new_distance = dist[u] + self.graph[u][v]
                if self.graph[u][v] > 0 and spt_set[v] == False and dist[v] > new_distance:
                    dist[v] = new_distance
                    path[count].append(u)
        # self.print_solution(dist, path)
        return dist, path

def generate_directions(min_phi=0, max_phi=np.pi, min_theta=0, max_theta=2*np.pi):
    directions = []
    for phi in np.linspace(min_phi, max_phi, 10):
        for theta in np.linspace(min_theta, max_theta, 10):
            directions.append(np.array([np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta), np.cos(phi)]))
    return directions
    # return [directions[0]]

def adjacency_matrix(mesh):
    n_verts = mesh.n_vertices()
    adjacency = np.zeros((n_verts, n_verts))
    for vh0 in mesh.vertices():
        for vh1 in mesh.vv(vh0):
            point0 = mesh.point(vh0)
            point1 = mesh.point(vh1)
            adjacency[vh0.idx()][vh1.idx()] = np.linalg.norm(point1 - point0)
            adjacency[vh1.idx()][vh0.idx()] = adjacency[vh0.idx()][vh1.idx()]
    return adjacency

def vertices_from_faces(mesh, faces):
    vertices = set()
    for fh in faces:
        vertices.update([vh.idx() for vh in mesh.fv(fh)])
    return list(vertices)

def select_from_adjacency_matrix(mesh, mesh_adjacency, faces):
    vertices = vertices_from_faces(mesh, faces)
    adjacency = np.zeros((len(vertices), len(vertices)))
    for i, vid0 in enumerate(vertices):
        for j, vid1 in enumerate(vertices):
            adjacency[i][j] = mesh_adjacency[vid0][vid1]
    return adjacency

def estimate_length(mesh, mesh_adjacency, faces):
    # Dijkstra to get the shortest path
    adjacency = select_from_adjacency_matrix(mesh, mesh_adjacency, faces)
    g = Graph(adjacency.shape[0])
    g.graph = adjacency
    g.dijkstra(0)
    # Copout
    return len(faces)

def main(file_name, export):
    mesh = om.read_trimesh(file_name)
    mesh.request_face_normals()
    mesh.update_normals()
    mesh.request_face_colors()
    mesh_adjacency = adjacency_matrix(mesh)
    face_normals = mesh.face_normals()

    isoline_lengths = []
    directions = generate_directions()
    for j, direction in enumerate(directions):
        perpendicular_faces = []
        for i, face in enumerate(mesh.faces()):
            if np.abs(np.dot(face_normals[i], direction)) < tolerance:
                perpendicular_faces.append(face)
        isoline_lengths.append(estimate_length(mesh, mesh_adjacency, perpendicular_faces))

    min_direction = directions[isoline_lengths.index(max(isoline_lengths))]
    print(min_direction, "direction chosen with length", min(isoline_lengths))
    for i, face in enumerate(mesh.faces()):
        if np.abs(np.dot(face_normals[i], min_direction)) < tolerance:
            mesh.set_color(face, [0.6, 0.3, 0.3, 0])
        else:
            mesh.set_color(face, [0.3, 0.6, 0.3, 0])
    if export:
        name, ext = os.path.splitext(file_name)
        export_filename = name + '_labeled.obj'
        om.write_mesh(export_filename, mesh, face_color=True)
        print("Saved labeled mesh to", export_filename) 

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Find shortest isocline')
    parser.add_argument('file_name',
                        help='Input file name is required. Enter it!')
    parser.add_argument('--export', required=False, action='store_true')
    parser.set_defaults(export=False)
    args = parser.parse_args()

    main(args.file_name, args.export)
