import os
import argparse
import time
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"      # https://github.com/opencv/opencv/issues/21326
import cv2
import numpy as np


EPSILON = 1e-10


def to_SRGB(src):
    # same as toSRGB() in src/nori/common.cpp
    smaller = src <= 0.0031308
    larger = ~smaller
    src[smaller] *= 12.92
    src[larger] = 1.055 * src[larger]**(1/2.4) - 0.055
    return src


def d2(up, uq, varp, varq, k):
    return ((up - uq)**2 - (varp + np.minimum(varq, varp))) / (EPSILON + k**2 * (varp + varq))


def nl_means_denoise(data: np.ndarray, data_var: np.ndarray, r=10, f=3, k=0.45):
    img_shape = data.shape
    print(f"Denoising image of size {img_shape}\n    r={r}, f={f}, k={k}")
    flt = np.zeros(img_shape, np.float32)
    wgtsum = np.zeros(img_shape, np.float32)

    box_filter_size = (2 * f + 1, 2 * f + 1)
    box_filter_size1 = (2 * f - 1, 2 * f - 1)

    for dx in range(-r, r+1):
        for dy in range(-r, r+1):
            ngb = np.roll(data, (dx, dy), axis=(1, 0))
            ngb_var = np.roll(data_var, (dx, dy), axis=(1, 0))

            d2pixel = d2(data, ngb, data_var, ngb_var, k)

            d2patch = cv2.boxFilter(d2pixel, -1, box_filter_size, borderType=cv2.BORDER_REPLICATE)

            wgt = np.exp(-np.maximum(0, d2patch))
            wgt = cv2.boxFilter(wgt, -1, box_filter_size1, borderType=cv2.BORDER_REPLICATE)

            flt += wgt * ngb
            wgtsum += wgt

    flt /= wgtsum
    return flt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=str, help="Path to the exr file to denoise")
    args = parser.parse_args()

    file = '.'.join(args.input.split('.')[:-1])
    data_file = file + ".exr"
    variance_file = file + "_variance.exr"

    data = cv2.imread(data_file, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    data_variance = cv2.imread(variance_file, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)

    print(f"Denoising '{data_file}'")
    start = time.time()
    denoised = nl_means_denoise(data, data_variance, r=20, f=9)
    execution_time = time.time() - start
    print(f"Finished denoising in {round(execution_time)} seconds ({round(execution_time / 60, 2)} minutes)")

    denoised = to_SRGB(denoised)
    denoised = denoised.clip(0, 1) * 255
    denoised = np.round(denoised).astype(np.uint8)
    cv2.imwrite(file + "_denoised.png", denoised)


if __name__ == '__main__':
    main()
