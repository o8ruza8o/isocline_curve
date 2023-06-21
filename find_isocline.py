import argparse
import trimesh
import openmesh as om
import numpy as np
import os

tolerance = 1e-1

def generate_directions(min_phi=0, max_phi=np.pi, min_theta=0, max_theta=2*np.pi):
    directions = []
    for phi in np.linspace(min_phi, max_phi, 10):
        for theta in np.linspace(min_theta, max_theta, 10):
            directions.append(np.array([np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta), np.cos(phi)]))
    return directions

def estimate_length(faces):
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
                perpendicular_faces.append(i)
        isoline_lengths.append(estimate_length(perpendicular_faces))

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
