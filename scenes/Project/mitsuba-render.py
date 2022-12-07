import argparse
import mitsuba as mi


parser = argparse.ArgumentParser()
parser.add_argument('scene', type=str, help="Path to the scene file in xml format")
args = parser.parse_args()

file = '.'.join(args.scene.split('.')[:-1])
print(f"Rendering '{file}.xml'")

mi.set_variant("scalar_rgb")
scene = mi.load_file(file + ".xml")
image = mi.render(scene)
mi.util.write_bitmap(file + ".png", image)
