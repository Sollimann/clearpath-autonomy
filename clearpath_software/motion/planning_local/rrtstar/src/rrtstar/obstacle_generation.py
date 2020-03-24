import uuid
import numpy as np

"""
Universally unique identifies (UUID) values are 128 bits long and “can guarantee uniqueness across space and time”.
They are useful for situations where a unique identifiers value is necessary.
"""


def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    :param obstacles:
    :return yield:
    """
    for obstacle in obstacles:
        yield uuid.uuid4(), obstacle, obstacle


def bbox(img):
    """
    :param img: given an body of rows and cols
    :return: rectangular dimensions of the provided rows and cols
    """
    img = (img > 0)
    rows = np.any(img, axis=1)
    cols = np.any(img, axis=0)
    rmin, rmax = np.argmax(rows), img.shape[0] - 1 - np.argmax(np.flipud(rows))
    cmin, cmax = np.argmax(cols), img.shape[1] - 1 - np.argmax(np.flipud(cols))
    return rmin, cmin, rmax, cmax