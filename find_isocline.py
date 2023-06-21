import argparse
import pymesh
import numpy as np


tolerance = 1e-1

def generate_directions():
    return [np.array([0, 0, 1])]

def main(file_name, visualize):
    mesh = pymesh.load_mesh(file_name)

    mesh.add_attribute("face_normal")
    all_normals = mesh.get_attribute("face_normal")

    for j, direction in enumerate(generate_directions()):
        perpendicular_faces = []
        for i, face in enumerate(mesh.faces):
            face_normal = np.array(all_normals[3 * i : 3 * i + 3])
            if np.abs(np.dot(face_normal, direction)) < tolerance:
                perpendicular_faces.append(i)
        print(perpendicular_faces)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Find shortest isocline')
    parser.add_argument('file_name',
                        help='Input file name is required. Enter it!')
    parser.add_argument('--visualize', required=False, action='store_true')
    parser.set_defaults(visualize=False)
    args = parser.parse_args()

    main(args.file_name, args.visualize)
