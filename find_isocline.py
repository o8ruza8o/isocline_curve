import argparse
import trimesh
import openmesh as om
import numpy as np
import os

tolerance = 1e-1

def generate_directions():
    return [np.array([0, 0, 1])]

def main(file_name, visualize):
    mesh = om.read_trimesh(file_name)
    mesh.request_face_normals()
    mesh.update_normals()
    mesh.request_face_colors()
    face_normals = mesh.face_normals()
    for j, direction in enumerate(generate_directions()):
        perpendicular_faces = []
        for i, face in enumerate(mesh.faces()):
            if np.abs(np.dot(face_normals[i], direction)) < tolerance:
                perpendicular_faces.append(i)
                mesh.set_color(face, [0.6, 0.3, 0.3, 0])
            else:
                mesh.set_color(face, [0.3, 0.6, 0.3, 0])
        print(perpendicular_faces)


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
