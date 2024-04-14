import numpy as np
from enum import Enum
from typing import List

class MapValue(Enum):
  UNKNOWN=205
  INVALID=1
  OCC=0
  FREE= 254

def get_all_motions(search_grid_points_map_pixelized: List[np.ndarray]):
    # Return points around 
    # optimization
    NUM_ITERATIONS = 3
    linear_step = 2**NUM_ITERATIONS
    for point in search_grid_points_map_pixelized:
        for i in range(NUM_ITERATIONS):
            linear_step = int(linear_step/2)
            motions += [
                # np.array([linear_step,0,0]),
                # np.array([0,linear_step,0]),
                # np.array([-linear_step,0,0]),
                # np.array([0,-linear_step,0]),
                # np.array([0,0,angle_step]),
                # np.array([0,0,-angle_step]),
            ]

def get_vector_1_pixel_away_vectorized(starts: np.ndarray, end: np.ndarray):
    # starts: np.array([[2, 1], [2, 2], [2, 4]])
    vecs = end - starts
    norms = np.linalg.norm(vecs, axis=1, keepdims=True)
    norms[norms == 0] = 1  # prevent division by zero, or handle zero vectors as needed
    unit_vecs = np.round(vecs / norms).astype(int)
    return unit_vecs

def map_pixel_to_matrix(points_for_all_thetas: List[np.ndarray], origin_px: np.ndarray, img_height: float):
    # [theta1[np.array(x,y)...], theta2 [...]]
    # [10/resolution, 10/resolution] in m. This is the pixel coord of the map origin, 
    # relative to the bottom left corner of the image
    # TODO: this actually messes up points_for_all_thetas
    return_arr = []
    for points_for_single_theta in points_for_all_thetas:
        points_for_single_theta += origin_px
        points_for_single_theta[:,1] = img_height - points_for_single_theta[:,1]
        points_for_single_theta = points_for_single_theta[:, [1,0]]
        return_arr.append(points_for_single_theta)
    return return_arr

def create_mask(p_hits, p_frees, img_width, img_height):
    xor_mask = MapValue.INVALID.value * np.ones((img_height, img_width))
    xor_mask[p_hits[:,0], p_hits[:,1]] = MapValue.OCC.value
    xor_mask[p_frees[:,0], p_frees[:,1]] = MapValue.FREE.value
    return xor_mask
    


if __name__ == "__main__":
    starts = np.array([[2, 1], [2, 2], [2, 4]])
    get_vector_1_pixel_away_vectorized(starts, np.array([0,0]))

    test_points_for_all_thetas = [np.array([[1,1], [2,2], [5,6]])]
    arr = map_pixel_to_matrix(test_points_for_all_thetas, np.array([1,1]), 10)
    #TODO Remember to remove
    print(f'Rico: {arr}')