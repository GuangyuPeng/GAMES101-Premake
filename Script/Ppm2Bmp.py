import os
import argparse
from PIL import Image

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', type=str, help='Set input .ppm file', required=True)
    args = parser.parse_args()

    ppm_path = args.input
    ppm_root, ext = os.path.splitext(ppm_path)
    if ext != '.ppm':
        raise Exception('Input file {} does not have a .ppm extension'.format(ppm_path))
    bmp_path = ppm_root + '.bmp'

    img = Image.open(ppm_path)
    img.save(bmp_path)
    img.close()
    print('Success to convert {} to {}'.format(ppm_path, bmp_path))


if __name__ == '__main__':
    main()