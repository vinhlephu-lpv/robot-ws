import time
import numpy as np
from my_robot_controller.interfaces import PlannerInterface
from my_robot_controller.path_utils import Path

class RRTStar:
    def __init__(self, start, goal, obstacles, x_bounds, y_bounds, 
                 step_size=0.5, max_iter=300, search_radius=1.2, robot_radius=0.38):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = np.array(obstacles)
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.robot_radius = robot_radius
        
        class Node:
            def __init__(self, pt):
                self.pt = np.array(pt)
                self.parent = None
                self.cost = 0.0
        self.Node = Node
        self.nodes = [self.Node(self.start)]

    def is_collision_free(self, p1, p2):
        dist = np.linalg.norm(p2 - p1)
        if dist < 1e-6:
            return True
        steps = int(np.ceil(dist / 0.1))
        for step in range(steps + 1):
            t = step / max(steps, 1)
            pt = p1 * (1 - t) + p2 * t
            if len(self.obstacles) > 0:
                dists = np.linalg.norm(self.obstacles - pt, axis=1)
                if np.any(dists < self.robot_radius):
                    return False
        return True

    def plan(self):
        # Seed random generator for repeatability
        np.random.seed(42)
        for _ in range(self.max_iter):
            if np.random.rand() < 0.15:
                q_rand = self.goal
            else:
                q_rand = np.array([
                    np.random.uniform(self.x_bounds[0], self.x_bounds[1]),
                    np.random.uniform(self.y_bounds[0], self.y_bounds[1])
                ])
            
            nearest_idx = np.argmin([np.linalg.norm(n.pt - q_rand) for n in self.nodes])
            q_nearest = self.nodes[nearest_idx]
            
            dir_vec = q_rand - q_nearest.pt
            dist = np.linalg.norm(dir_vec)
            if dist < 1e-6:
                continue
            
            step = min(self.step_size, dist)
            q_new_pt = q_nearest.pt + (dir_vec / dist) * step
            
            if not self.is_collision_free(q_nearest.pt, q_new_pt):
                continue
                
            q_new = self.Node(q_new_pt)
            
            # Find near nodes
            near_indices = []
            for idx, n in enumerate(self.nodes):
                if np.linalg.norm(n.pt - q_new.pt) < self.search_radius:
                    near_indices.append(idx)
                    
            # Choose parent
            min_cost = q_nearest.cost + np.linalg.norm(q_new.pt - q_nearest.pt)
            best_parent = q_nearest
            
            for idx in near_indices:
                n_near = self.nodes[idx]
                cost = n_near.cost + np.linalg.norm(q_new.pt - n_near.pt)
                if cost < min_cost:
                    if self.is_collision_free(n_near.pt, q_new.pt):
                        min_cost = cost
                        best_parent = n_near
                        
            q_new.parent = best_parent
            q_new.cost = min_cost
            self.nodes.append(q_new)
            
            # Rewire
            for idx in near_indices:
                n_near = self.nodes[idx]
                cost = q_new.cost + np.linalg.norm(n_near.pt - q_new.pt)
                if cost < n_near.cost:
                    if self.is_collision_free(q_new.pt, n_near.pt):
                        n_near.parent = q_new
                        n_near.cost = cost
                        
        best_goal_node = None
        min_dist_to_goal = float('inf')
        
        for n in self.nodes:
            d = np.linalg.norm(n.pt - self.goal)
            if d < min_dist_to_goal:
                min_dist_to_goal = d
                best_goal_node = n
                
        if best_goal_node is None or min_dist_to_goal > 1.2:
            return None
            
        path = []
        curr = best_goal_node
        while curr is not None:
            path.append(curr.pt.tolist())
            curr = curr.parent
        path.reverse()
        
        if np.linalg.norm(np.array(path[-1]) - self.goal) > 1e-3:
            path.append(self.goal.tolist())
            
        return path


class RRTStarPlanner(PlannerInterface):
    """
    Standard wrapper for RRTStar Planner implementing PlannerInterface.
    """
    def __init__(self):
        self.step_size = 0.5
        self.max_iter = 300
        self.search_radius = 1.2
        self.robot_radius = 0.38
        self.status = "IDLE"

    def initialize(self, step_size=0.5, max_iter=300, search_radius=1.2, robot_radius=0.38):
        self.step_size = step_size
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.robot_radius = robot_radius
        self.status = "INITIALIZED"

    def plan(self, start_pose, goal_pose, obstacles=None, safety_margin=0.38, planning_mode="AVOIDANCE"):
        if obstacles is None:
            obstacles = []
            
        self.status = "PLANNING"
        start_time = time.time()
        
        start = start_pose[:2]
        goal = goal_pose[:2]
        
        # Define search bounds based on start and goal
        min_x = min(start[0], goal[0]) - 3.0
        max_x = max(start[0], goal[0]) + 3.0
        min_y = min(start[1], goal[1]) - 3.0
        max_y = max(start[1], goal[1]) + 3.0
        
        # Use safety_margin as the robot_radius for collision checking
        planner = RRTStar(
            start=start,
            goal=goal,
            obstacles=obstacles,
            x_bounds=(min_x, max_x),
            y_bounds=(min_y, max_y),
            step_size=self.step_size,
            max_iter=self.max_iter,
            search_radius=self.search_radius,
            robot_radius=safety_margin
        )
        
        path = planner.plan()
        duration = time.time() - start_time
        
        if path is not None:
            self.status = "SUCCESS"
            path_obj = Path(path, planner_type="RRTStar")
            return {
                "path": path_obj,
                "status": "SUCCESS",
                "cost": path_obj.length,
                "duration": duration,
                "num_nodes": len(planner.nodes)
            }
        else:
            self.status = "FAILED"
            return {
                "path": None,
                "status": "FAILED",
                "cost": float('inf'),
                "duration": duration,
                "num_nodes": len(planner.nodes)
            }

    def cancel(self):
        self.status = "CANCELLED"

    def get_status(self):
        return self.status

    def reset(self):
        self.status = "IDLE"
