import argparse
import mitsuba as mi


parser = argparse.ArgumentParser()
parser.add_argument('input', type=str, help="Path to the exr file to conver to png")
args = parser.parse_args()

file = '.'.join(args.input.split('.')[:-1])
print(f"Converting '{file}.exr' to png")

mi.set_variant('scalar_rgb')
img_exr = mi.Bitmap(file + '.exr')
mi.util.write_bitmap(file + '.png', img_exr)
