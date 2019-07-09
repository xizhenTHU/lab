import numpy as np


def PrecisionPointSol(latlng, error_latlng):
    # 输入输出均采用NumPy数组

    # percent = np.sum(error_latlng, axis=1)
    # percent = np.transpose([1 - (percent / np.sum(percent))])
    # latlng_pre = np.sum(latlng * percent, axis=0)
    percent = error_latlng / np.sum(error_latlng, axis=0)
    latlng_pre = np.sum(latlng * percent, axis=0)
    return latlng_pre


# example:
if __name__ == '__main__':
    latlng = np.array([[114.0, 5], [113.0, 2], [113.0, 2]])
    error_latlng = np.array([[2.0, 3], [4.0, 5], [4.0, 5]])
    latlng_pre = PrecisionPointSol(latlng, error_latlng)
    print(latlng_pre)
