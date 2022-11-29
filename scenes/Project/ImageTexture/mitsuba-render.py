import mitsuba as mi
import matplotlib.pyplot as plt

mi.set_variant("scalar_rgb")

scene = mi.load_file("mitsuba-image-texture.xml")
image = mi.render(scene, spp=256)


#plt.axis("off")
#plt.imshow(image ** (1.0 / 2.2)) # approximate sRGB tonemapping
mi.util.write_bitmap("mitsuba-image-texture.png", image)