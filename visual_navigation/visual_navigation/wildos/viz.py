from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import numpy as np
import cv2

from visual_navigation.geofrontier_nav.viz import VisualizeGeoFrontierScoring
from visual_navigation.utils.viz import (
    make_subplot_grid, overlay_heatmap, draw_point, draw_path, make_colorbar, show_mask
)


class VisualizeGoalAgnosticGeoFrontierScoring(VisualizeGeoFrontierScoring):
    def __init__(self, angular_bins: np.ndarray, **kwargs):
        super().__init__(**kwargs)
        self.discretization_angle = 2 * np.pi / len(angular_bins)
        self.bin_starts = angular_bins
        self.num_bins = len(angular_bins)

        # for visualizing the score map, for each camera show the score map for this heading
        self.goal_cam_relative_headings = 0.0
        self.ring_radius = 1.0  # radius of the score ring in meters

    def visualize_model_det_front(self, nav_data, all_cam_data):
        img_grid = {}
        num_rows = 1
        num_cols = 2
        fig_resize_factor = 0.75


        rgb_img = nav_data[0]["image"]
        # rgb_img = cv2.resize(rgb_img, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
        frontier_overlay = rgb_img.copy()
        trav_overlay = rgb_img.copy()

        frontier_map = nav_data[0]["img_frontiers"].astype(np.float32)
        traversability_map = nav_data[0]["traversability"].astype(np.float32)


        frontier_map[frontier_map < 0.6] = 0.0
        frontier_map[traversability_map < 0.9] = 0.0

        # Morphological opening to remove small noisy frontier regions
        kernel = np.ones((20, 20), np.uint8)
        valid = cv2.morphologyEx((frontier_map>0).astype(np.uint8), cv2.MORPH_OPEN, kernel)
        frontier_map[valid==0] = 0.0

        frontier_hm = overlay_heatmap(rgb_img, frontier_map, alpha=1.0)
        valid_frontier = (frontier_map > 0)
        valid_mask_3d = np.stack([valid_frontier] * 3, axis=-1)
        frontier_overlay[valid_mask_3d] = frontier_hm[valid_mask_3d]
        frontier_overlay = cv2.resize(frontier_overlay, (0,0), fx=fig_resize_factor, fy=fig_resize_factor)
        img_grid[(0, 0)] = (frontier_overlay, "Frontiers Overlay")
        # return frontier_overlay

        # overlay traversability on the image
        traversability_map[traversability_map < 0.9] = 0.0

        trav_hm = overlay_heatmap(rgb_img, 1-traversability_map, alpha=0.5)
        valid_traversability = (traversability_map > 0)
        valid_mask_3d = np.stack([valid_traversability] * 3, axis=-1)
        trav_overlay[valid_mask_3d] = trav_hm[valid_mask_3d]
        trav_overlay = cv2.resize(trav_overlay, (0,0), fx=fig_resize_factor, fy=fig_resize_factor)
        img_grid[(0, 1)] = (trav_overlay, "Traversability Overlay")
        # return trav_overlay

        fin_img_chosen = None
        if "object_mask" in nav_data[0] and nav_data[0]["object_mask"] is not None:
            max_obj_pix = -1
            for i in range(self.num_cameras):
                img = nav_data[i]["image"]
                H, W, C = img.shape
                
                obj_mask_2d = nav_data[i]["object_mask"].squeeze() 
                valid_mask_2d = obj_mask_2d > 0
                num_obj_pixels = np.sum(valid_mask_2d)

                if num_obj_pixels <= max_obj_pix:
                    continue

                fin_img = show_mask(img, obj_mask_2d)

                DARK_GRAY = (50, 50, 50) 
                WHITE = (255, 255, 255)
                FONT = cv2.FONT_HERSHEY_DUPLEX 
                FONT_SCALE = 0.7
                FONT_THICKNESS = 1
                PADDING = 10 # Padding around the text
                text_to_display = f"Current View: {self.camera_mapping[i].title()} Camera"
                (text_w, text_h), baseline = cv2.getTextSize(
                    text_to_display, FONT, FONT_SCALE, FONT_THICKNESS
                )

                text_x = PADDING
                text_y = PADDING + text_h

                p1 = (PADDING, PADDING) # Top-left corner of the background
                p2 = (PADDING + text_w + PADDING, PADDING + text_h + baseline + PADDING) # Bottom-right corner

                cv2.rectangle(fin_img, p1, p2, DARK_GRAY, -1) # -1 fills the rectangle

                cv2.putText(
                    fin_img, 
                    text_to_display, 
                    (text_x, text_y), 
                    FONT, 
                    FONT_SCALE, 
                    WHITE, 
                    FONT_THICKNESS, 
                    cv2.LINE_AA
                )

                max_obj_pix = num_obj_pixels
                fin_img_chosen = fin_img

            if fin_img_chosen is not None:
                fin_img_chosen = cv2.resize(fin_img_chosen, (0,0), fx=fig_resize_factor, fy=fig_resize_factor)
                img_grid[(0, 2)] = (fin_img_chosen, "Object Detection Overlay")
                num_cols = 3

        grid = make_subplot_grid(img_grid, (num_rows, num_cols), pad=15)

        return grid

    def visualize_model_det(self, nav_data, all_cam_data):
        img_grid = {}
        num_rows = 4
        num_cols = self.num_cameras

        for i in range(self.num_cameras):
            plt_idx = self.cam_order[i]

            rgb_img = nav_data[i]["image"]
            rgb_img = cv2.resize(rgb_img, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
            img_grid[(0, plt_idx)] = (rgb_img, f"Image {self.camera_mapping[i]}")

            has_object_mask = False
            if "object_mask" in nav_data[i] and nav_data[i]["object_mask"] is not None:
                obj_mask = nav_data[i]["object_mask"].astype(np.float32)
                obj_mask = cv2.resize(obj_mask, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
                if np.any(obj_mask > 0):
                    rgb_img = show_mask(rgb_img, obj_mask)
                    has_object_mask = True

            # overlay frontiers on the image
            frontier_map = nav_data[i]["img_frontiers"].astype(np.float32)
            frontier_map = cv2.resize(frontier_map, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
            frontier_overlay = overlay_heatmap(rgb_img, frontier_map)
            img_grid[(1, plt_idx)] = (frontier_overlay, "Frontier Conf.")

            # overlay traversability on the image
            traversability_map = nav_data[i]["traversability"].astype(np.float32)
            traversability_map = cv2.resize(traversability_map, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
            trav_overlay = overlay_heatmap(rgb_img, traversability_map)
            img_grid[(2, plt_idx)] = (trav_overlay, "Traversability Conf.")

            # Show projected geo_frontiers and paths to straight-line goal from camera
            path_overlay = rgb_img.copy()
            if "geo_frontiers" not in nav_data[i]:
                img_grid[(3, plt_idx)] = (path_overlay, "Frontier Nodes: none")
                continue

            geo_frontiers = nav_data[i]["geo_frontiers"] * self.fig_resize_factor
            cam_heading = all_cam_data[i]["R_wc"].astype(np.float32) @ np.array([0, 0, 1], dtype=np.float32)
            cam_heading = cam_heading[:2]
            cam_heading = cam_heading / np.linalg.norm(cam_heading)
            cam_angle = np.arctan2(cam_heading[1], cam_heading[0])
            goal_heading = cam_angle + self.goal_cam_relative_headings
            heading_bin = int(goal_heading / self.discretization_angle) % len(self.bin_starts)

            scores = nav_data[i]["scores"]
            paths = nav_data[i]["paths"]
            score_map = nav_data[i]["score_map"][0][:,:,heading_bin].astype(np.float32)
            score_map = np.clip(score_map, 0.0, 1.0)
            score_map = cv2.resize(score_map, (0,0), fx=self.fig_resize_factor, fy=self.fig_resize_factor)
            path_overlay_hm = overlay_heatmap(path_overlay, score_map, alpha=0.5)
            valid_map = (score_map > 0)
            valid_mask_3d = np.stack([valid_map] * 3, axis=-1)
            path_overlay[valid_mask_3d] = path_overlay_hm[valid_mask_3d]

            for ((y,x), score, path) in zip(geo_frontiers, scores, paths):
                path = np.array(path[heading_bin]) * self.fig_resize_factor
                graph_color = (0, 255, 0)

                # Use green projected graph cues to match the paper video overlay
                draw_path(path_overlay, path, graph_color)
                draw_point(path_overlay, (y,x), graph_color, radius=7)
                draw_path(rgb_img, path, graph_color)
                draw_point(rgb_img, (y,x), graph_color, radius=7)
                # draw_point(path_overlay, path[-1], (255,255,255), radius=2)  # goal point
                # draw_text(path_overlay, (y,x), f"{score[heading_bin]:.2f}", color=(255,255,255))
            image_title = f"Image {self.camera_mapping[i]} + graph"
            if has_object_mask:
                image_title += " + Obj Mask"
            img_grid[(0, plt_idx)] = (rgb_img, image_title)
            img_grid[(3, plt_idx)] = (path_overlay, "Frontier Nodes")


        grid = make_subplot_grid(img_grid, (num_rows, num_cols), pad=15)
        cbar = make_colorbar(
            height=grid.shape[0] - 100,
            width=20,
            vmin=0,
            vmax=1,
            cmap=cv2.COLORMAP_JET,
            num_ticks=10,
            font_scale=0.5,
            pad=50
        )
        grid = cv2.hconcat([grid, cbar])

        return grid
