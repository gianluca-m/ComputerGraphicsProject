import mitsuba as mi


mi.set_variant('scalar_rgb')

folder = 'Project/ImageTexture/'
imagename = 'image-texture'
img_exr = mi.Bitmap(folder + imagename + '.exr')

mi.util.write_bitmap(folder + imagename + '.png', img_exr)
