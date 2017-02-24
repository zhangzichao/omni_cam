#!/usr/bin/env python

import argparse

def strToFloatList(raw_str, delimiter=' '):
    str_list = raw_str.split(' ')
    return [float(s) for s in str_list if s != '']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convert Matlab format to cpp")
    parser.add_argument('input_file', type=str, help='input file name')
    parser.add_argument('output_file', type=str, help='output file name')
    args = parser.parse_args()

    try:
        input_f = open(args.input_file, 'r')
        output_f = open(args.output_file, 'w')
    except:
        print('Could not open input or output file.')

    print("Read calibration from {0}".format(args.input_file))

    lines = input_f.readlines()
    filtered = [l[:-1] for l in lines if l[0] != '\n' and l[0] != '#']

    pol = strToFloatList(filtered[0])[1:]
    assert len(pol) == 5
    invpol = strToFloatList(filtered[1])[1:]
    assert len(invpol) <= 12
    invpol += [0.0] * (12 - len(invpol))
    center = strToFloatList(filtered[2])[::-1]
    affine = strToFloatList(filtered[3])
    img_size = strToFloatList(filtered[4])[::-1]
    img_size = [int(v) for v in img_size]

    print('Parsed parameters:')
    print('polynomial: {0}\ninverse polynomial: {1}'.format(pol, invpol))
    print('image size: {0}\nimage center: {1}'.format(img_size, center))
    print('affine correction: {0}'.format(affine))

    merged = img_size + pol + center + affine + invpol
    merged_line = " ".join(str(v) for v in merged)
    output_f.writelines([merged_line])
    print("Write calibration to {0}".format(args.output_file))

    input_f.close()
    output_f.close()
