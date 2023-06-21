import argparse
import pymesh


def main(file_name, visualize):
    mesh = pymesh.load_mesh(file_name);
    print(mesh.faces)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Find shortest isocline')
    parser.add_argument('file_name',
                        help='Input file name is required. Enter it!')
    parser.add_argument('--visualize', required=False, action='store_true')
    parser.set_defaults(visualize=False)
    args = parser.parse_args()

    main(args.file_name, args.visualize)
