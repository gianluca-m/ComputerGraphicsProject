import mitsuba as mi
import matplotlib.pyplot as plt


file = "NormalMapping/mitsuba-normal-mapping"
print(f"Rendering '{file}.exr'")

mi.set_variant("scalar_rgb")

scene = mi.load_file(file + ".xml")
image = mi.render(scene, spp=256)


#plt.axis("off")
#plt.imshow(image ** (1.0 / 2.2)) # approximate sRGB tonemapping
mi.util.write_bitmap(file + ".png", image)