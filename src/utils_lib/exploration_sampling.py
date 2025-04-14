import numpy as np
import random
import math



def sample_candidate_goals(origin, resolution, map_shape, num_samples=200, min_clearance=0.3):
    """Randomly sample points within the known map area and filter close-to-obstacles."""
    goals = []
    min_x = origin[0]
    max_x = origin[0] + map_shape[0] * resolution
    min_y = origin[1]
    max_y = origin[1] + map_shape[1] * resolution

    for _ in range(num_samples * 3):  # oversample to improve diversity
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        goals.append([x, y])
        if len(goals) >= num_samples:
            break

    return goals


def compute_info_gain(map_data, goal, origin, resolution, radius=2.5):
    """Estimate how much unknown space is visible from the goal pose."""
    info_gain = 0
    radius_cells = int(radius / resolution)
    gx, gy = goal_to_map_coords(goal, origin, resolution)

    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            mx = gx + dx
            my = gy + dy
            if 0 <= mx < map_data.shape[0] and 0 <= my < map_data.shape[1]:
                if map_data[mx, my] == -1:
                    info_gain += 1
    return info_gain * (resolution ** 2)  # in square meters


def goal_to_map_coords(goal, origin, resolution):
    """Convert world coordinates to map grid indices."""
    mx = int((goal[0] - origin[0]) / resolution)
    my = int((goal[1] - origin[1]) / resolution)
    return mx, my


def score_goals(goals, robot_pose, info_gains, weight_gain=1.0, weight_dist=0.3):
    """Score goals based on info gain and distance. Filter 0-gain goals before scoring."""
    scored = []

    for i, goal in enumerate(goals):
        if info_gains[i] < 0.01:  # filter zero-gain goals
            continue

        dist = np.linalg.norm(np.array(goal) - np.array(robot_pose[:2]))
        score = weight_gain * info_gains[i] - weight_dist * dist
        scored.append((score, i))

    if not scored:
        return 0  # fallback

    # Return index of goal with best score
    _, best_idx = max(scored, key=lambda x: x[0])
    return best_idx
