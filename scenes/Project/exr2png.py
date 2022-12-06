import mitsuba as mi


mi.set_variant('scalar_rgb')

# TODO: file name as command line argument 
file = 'NormalMapping/normal-mapping-sphere'
print(f"Converting '{file}.exr' to png")
img_exr = mi.Bitmap(file + '.exr')

mi.util.write_bitmap(file + '.png', img_exr)
