import numpy as np
import os
import rospkg
import time
from os import path
from scipy.misc import toimage

import noise


def get_abs_pkg_path(pkg_name):
    ros_pack = rospkg.RosPack()
    return ros_pack.get_path(pkg_name)


PACKAGE_NAME = "pioneer3at_simulation"
TEXTURES_FOLDER = "models/color_plane/materials/textures"
TEXTURES_FOLDER_PATH = path.join(get_abs_pkg_path(PACKAGE_NAME), TEXTURES_FOLDER)


def generate_texture(texture_shape=(1024, 1024), scale=150.0, octaves=6, persistence=0.5, lacunarity=1.0,
                     save_texture=True, texture_name=None, texture_extension=".png",
                     textures_folder=TEXTURES_FOLDER_PATH, show_texture=False, white_range=[0.3, 0.5]):
    assert len(white_range) == 2
    assert all(isinstance(i, float) for i in white_range)
    assert white_range[0] <= white_range[1]

    texture_array = np.zeros(texture_shape)

    for i in range(texture_shape[0]):
        for j in range(texture_shape[1]):
            texture_array[i][j] = noise.pnoise2(i / scale,
                                                j / scale,
                                                octaves=octaves,
                                                persistence=persistence,
                                                lacunarity=lacunarity,
                                                repeatx=1024,
                                                repeaty=1024,
                                                base=0)

            if white_range[0] <= texture_array[i][j] <= white_range[1]:
                texture_array[i][j] = 1.0  # white
            else:
                texture_array[i][j] = 0.0  # black

    if save_texture:
        if not isinstance(texture_name, str):
            texture_name = time.strftime("%Y-%m-%d-%H-%M-%S")
        if not path.exists(textures_folder):
            os.makedirs(textures_folder)
        texture_path = path.join(textures_folder, texture_name + texture_extension)
        toimage(texture_array).save(texture_path)
    if show_texture:
        toimage(texture_array).show()


if __name__ == "__main__":
    generate_texture()
