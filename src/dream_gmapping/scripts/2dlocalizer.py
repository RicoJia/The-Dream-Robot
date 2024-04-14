#!/usr/bin/env python3
"""
TODOs:
1. **The current scan matching method is subject to local minima, and is not feasible for large spaces**
1. Need scan message angular increments. Right now it's hardcoded
2. Map saved from map_server doesn't provide correct occupancy_threshold. We are hardcoding them
3. Need laser scan effective ranges
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
import numpy as np
from typing import List, Tuple, Dict
from localizer_2d_utils import (
    get_vector_1_pixel_away_vectorized,
    map_pixel_to_matrix,
    create_mask,
    MapValue,
    matrix_to_map_pixel,
    add_pose_to_relative_poses,
    get_points_on_search_grid,
)

SEARCH_GRID_RESOLUTION = 8  # 3.2m
BEAM_SEARCH_KERNEL_SIZE = 2
BEAM_NOICE_VARIANCE = 0.1  # in meter


def load_map_and_meta_data(path):
    # Replace 'map.yaml' with the path to your uploaded YAML file
    with open(path + ".yaml", "r") as file:
        map_metadata = yaml.safe_load(file)
    # Replace 'map.pgm' with the path to your uploaded PGM file
    map_image = mpimg.imread(path + ".pgm")
    return map_metadata, map_image


def map_px_to_plot_coordinates(map_px, origin):
    return map_px - origin


def load_scan_messages(filename):
    with open(filename, "rb") as file:
        data = pickle.load(file)
    return data


def visualize_map(
    map_image, origin_px, best_laser_endbeam_xy=None, p_free_endbeams=None
):
    origin_x_px, origin_y_px = origin_px
    plt.figure(figsize=(10, 10))
    img_width = map_image.shape[1]
    img_height = map_image.shape[0]
    x_ticks = np.arange(-origin_x_px, img_width - origin_x_px)
    y_ticks = np.arange(-origin_y_px, img_height - origin_y_px)
    plt.imshow(
        map_image,
        cmap="gray",
        extent=[x_ticks[0], x_ticks[-1], y_ticks[0], y_ticks[-1]],
    )  # Maps are often best visualized in grayscale

    plt.title("Map Visualization")
    plt.xlabel("Matrix Y(plot X)")
    plt.ylabel("Matrix X(plot Y)")
    map_origin_px = np.array([0, 0, 0])
    plt.plot(map_origin_px[0], map_origin_px[1], "ro")  # Red dot
    plt.text(map_origin_px[0], map_origin_px[1], "Origin", color="red")

    # One Big Difference between matplotlib plot and np array is x and y in np array row major.
    # The visualization here is column major.
    def show_scatter_plot(points: List[np.ndarray], color: str, label: str):
        x_coords = [xy[0] for xy in points]
        y_coords = [xy[1] for xy in points]

        plt.scatter(
            x_coords, y_coords, c=color, marker="o", label=label, s=2, linewidths=0.05
        )
        # Adding a legend to identify the points
        plt.legend()

    if best_laser_endbeam_xy is not None:
        show_scatter_plot(
            points=best_laser_endbeam_xy, color="red", label="Laser Endpoints"
        )

    if p_free_endbeams is not None:
        show_scatter_plot(
            points=p_free_endbeams, color="green", label="Last Free Points"
        )

    plt.show()


def transform_laser_scan(laser_scan, pose):
    # Get laser scan (x,y) in world frame
    # Convert (x, y) to homogeneous coordinates
    angles = np.arange(0, -2 * np.pi, -2 * np.pi / len(laser_scan))
    x, y, theta = pose
    laser_beam_endpoints = [
        np.array(
            [
                int(x + range / resolution * np.cos(theta + bearing)),
                int(y + range / resolution * np.sin(theta + bearing)),
            ]
        )
        for range, bearing in zip(laser_scan, angles)
        if range < effective_range
    ]
    return laser_beam_endpoints


def get_laser_endbeam_relative_for_all_thetas(
    search_thetas: np.ndarray,
    bearings: np.ndarray,
    scan_msg: np.ndarray,
    effective_range: float,
    resolution: float,
):
    # return array doesn't have inf. And they are in pixelized map frame
    assert len(scan_msg) == len(
        bearings
    ), "number of bearings and scan_msg should be the same"
    p_hits_for_all_thetas = []
    # [theta1[(x,y)...], theta2 [...]]
    for theta in search_thetas:
        p_hit_xs_relative = [
            np.round(d * np.cos(bearing + theta) / resolution)
            for d, bearing in zip(scan_msg, bearings)
            if d <= effective_range
        ]  # x = d*cos(theta+bearing)
        p_hit_ys_relative = [
            np.round(d * np.sin(bearing + theta) / resolution)
            for d, bearing in zip(scan_msg, bearings)
            if d <= effective_range
        ]  # y = d*sin(theta+bearing)
        p_hits_for_all_thetas.append(
            np.asarray(
                [
                    np.array((x, y), dtype=int)
                    for x, y in zip(p_hit_xs_relative, p_hit_ys_relative)
                ]
            )
        )
    # combine_xy_for_all_thetas
    return p_hits_for_all_thetas


def get_p_frees_for_all_thetas(
    search_thetas: np.ndarray, p_hits_for_all_thetas: List[np.ndarray]
):
    # [theta1[np.array(x,y)...], theta2 [...]]
    p_free_relative = []
    for p_hits_for_theta in p_hits_for_all_thetas:
        unit_vecs = get_vector_1_pixel_away_vectorized(
            p_hits_for_theta, np.array([0, 0])
        )
        p_frees_for_theta = p_hits_for_theta + unit_vecs
        p_free_relative.append(p_frees_for_theta)

    return p_free_relative


if __name__ == "__main__":
    map_metadata, map_image = load_map_and_meta_data("/home/rjia/Videos/bird_world")
    all_data = load_scan_messages("/home/rjia/Videos/scan_data.pkl")
    resolution = map_metadata["resolution"]
    img_width = map_image.shape[1]
    img_height = map_image.shape[0]
    # [10/resolution, 10/resolution] in m. This is the pixel coord of the map origin,
    # relative to the bottom left corner of the image
    origin_px = (
        np.array([-map_metadata["origin"][0], -map_metadata["origin"][1]]) / resolution
    ).astype(int)

    # # Convert world coordinates to pixel coordinates
    # # Especially y, because in robotics, Y starts from bottoms up
    # # while in computer graphics, Y starts from top down
    # visualize_map(map_image, origin_px)
    beam_search_kernel = list(
        range(-BEAM_SEARCH_KERNEL_SIZE, BEAM_SEARCH_KERNEL_SIZE, 1)
    )
    # meter -> pixels
    effective_range = 10 / resolution
    resolution_squared = resolution * resolution
    # TODO Remember to remove
    print(f"Rico: {len(all_data)}")
    trial_scan_msg = all_data[-1]

    search_thetas = np.arange(0, 2 * np.pi, np.pi / 128)
    # search_thetas = [np.pi/2]
    bearings = np.arange(0, 2 * np.pi, 2 * np.pi / len(trial_scan_msg))
    # From now on, we are operating in pixelized map frame
    # [theta1[(x,y)...], theta2 [...]]
    p_hits_for_all_thetas = get_laser_endbeam_relative_for_all_thetas(
        search_thetas=search_thetas,
        bearings=bearings,
        scan_msg=trial_scan_msg,
        effective_range=effective_range,
        resolution=resolution,
    )
    p_frees_for_all_thetas = get_p_frees_for_all_thetas(
        search_thetas, p_hits_for_all_thetas
    )

    # optimization
    # 1. Search grid
    def optimize_using_grid_search(
        map_image: np.ndarray,
        top_left: np.ndarray,
        bottom_right: np.ndarray,
        search_grid_resolution: int,
        best_point_so_far: np.ndarray,
    ):
        if search_grid_resolution == 0:
            return
        search_grid_points = get_points_on_search_grid(
            map_image=map_image,
            top_left=top_left,
            bottom_right=bottom_right,
            search_grid_resolution=search_grid_resolution,
        )
        search_grid_points_map_pixelized = matrix_to_map_pixel(
            search_grid_points, origin_px, img_height=img_height
        )
        # # TODO
        # search_grid_points_map_pixelized =  np.array([[20,20], [0,8]])
        best_score = 0
        best_point = None
        best_theta_index = -1
        best_p_hits = None
        for pose in search_grid_points_map_pixelized:
            # all map coords thetas
            p_hits_for_all_thetas_for_pose = add_pose_to_relative_poses(
                p_hits_for_all_thetas, pose[:2]
            )
            p_frees_for_all_thetas_for_pose = add_pose_to_relative_poses(
                p_frees_for_all_thetas, pose[:2]
            )
            # To matrix coords
            p_hit_in_matrix_coords_for_all_thetas = map_pixel_to_matrix(
                points_for_all_thetas=p_hits_for_all_thetas_for_pose,
                origin_px=origin_px,
                img_height=img_height,
            )
            p_free_in_matrix_coords_for_all_thetas = map_pixel_to_matrix(
                points_for_all_thetas=p_frees_for_all_thetas_for_pose,
                origin_px=origin_px,
                img_height=img_height,
            )
            best_single_pose_score = 0
            best_single_pose_theta_index = -1
            best_single_pose_p_hits = None
            for theta_idx in range(len(search_thetas)):
                p_hits = p_hit_in_matrix_coords_for_all_thetas[theta_idx]
                p_frees = p_free_in_matrix_coords_for_all_thetas[theta_idx]
                try:
                    xor_mask = create_mask(p_hits, p_frees, img_width, img_height)
                except IndexError:
                    continue
                result_map = (map_image == xor_mask).astype(int)
                score = np.sum(result_map)
                if score > best_single_pose_score:
                    best_single_pose_score = score
                    best_single_pose_theta_index = theta_idx
                    best_single_pose_p_hits = p_hits_for_all_thetas_for_pose[theta_idx]
                # print(f'Theta: {search_thetas[theta_idx]} Score: {score}')
            if best_score < best_single_pose_score:
                best_score = best_single_pose_score
                best_point = pose
                best_theta_index = best_single_pose_theta_index
                best_p_hits = best_single_pose_p_hits
            print(
                f"best angle {best_single_pose_theta_index}, score: {best_single_pose_score}"
            )
        print(
            f"best score: {best_score}, best point: {best_point}, best theta: {search_thetas[best_theta_index]}"
        )
        quarter_window = np.array([search_grid_resolution, search_grid_resolution])
        if best_point_so_far is not None and best_point is not None:
            if np.array_equal(best_point, best_point_so_far):
                visualize_map(map_image, origin_px, best_laser_endbeam_xy=best_p_hits)
                # return
        optimize_using_grid_search(
            map_image=map_image,
            top_left=map_pixel_to_matrix(
                [np.array([best_point])], origin_px, img_height
            )[0][0]
            - quarter_window,
            bottom_right=map_pixel_to_matrix(
                [np.array([best_point])], origin_px, img_height
            )[0][0]
            + quarter_window,
            search_grid_resolution=int(search_grid_resolution / 2),
            best_point_so_far=best_point,
        )

    optimize_using_grid_search(
        map_image=map_image,
        top_left=np.array([0, 0]),
        bottom_right=np.asarray(map_image.shape),
        search_grid_resolution=SEARCH_GRID_RESOLUTION,
        best_point_so_far=None,
    )
    # Score:
    # Given a pose in search grid,
    #   - map_pixel of laser endpoint beam relative to the pose?
    #   - map_pixel of p_free relative to the pose (could be optimized potentially)
    # - Transform:
    #   - Indices of the laser endpoint beams in the matrix
    #   - Indices of the p_free in the matrix
    # - Masking:
    #   - Create mask with [0,1] of p_hit and p_free
    #   - score = sum(map xor mask)
