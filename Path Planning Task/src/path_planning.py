from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D
from scipy.interpolate import CubicSpline
import numpy as np
from scipy import interpolate
import random




class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        num_points = 25
        step = 0.1
        cx = self.car_pose.x
        cy = self.car_pose.y
        leftSiders = []
        rightSiders = []
        yaw = self.car_pose.yaw
        yaws = [yaw]
        offset = 0.8
        path_smoothed = False
        import math

        def path_smoother(points : list) -> list:
            if len(points) >= 3: 
                path_smoothed = True   
                np_criticalPoints = np.array(points)
                x = np_criticalPoints[:, 0]
                y = np_criticalPoints[:, 1]

                tck, _ = interpolate.splprep([x, y], s=0.01, k=2)

                u_fine = np.linspace(0, 1, 200)
                path_x, path_y = interpolate.splev(u_fine, tck)
                dx, dy = interpolate.splev(u_fine, tck, der=1)
                yaws = np.arctan2(dy, dx)
                yaw = yaws[-1]

                path_ = list(zip(path_x, path_y))
                cx = path_points[-1][0]
                cy = path_points[-1][1]
                return path_
            return points
        def Prohibit_Zones() ->list:
            prohibited_zones = []
            for i in range(len(rightSiders)):
                prohibited_zones.append(((rightSiders[i][0], -1.0),(6.0,rightSiders[i][1] )))
            for i in range(len(leftSiders)):
                prohibited_zones.append(((-1.0, leftSiders[i][1]),(leftSiders[i][0],6.0 )))

            return prohibited_zones
        def candidate_is_valid(candidate, prohibited_zones, margin=0.2):
            x, y = map(float, np.ravel(candidate))
            for (p_min, p_max) in prohibited_zones:
                x_min, y_min = p_min; x_max, y_max = p_max
                x_min, x_max = sorted([x_min, x_max]); y_min, y_max = sorted([y_min, y_max])
                if (x_min - margin) <= x <= (x_max + margin) and (y_min - margin) <= y <= (y_max + margin):
                    return False
            return -0.7 <= x <= 5.99 and -0.7 <= y <= 5.99
        def validity_checker(points : list, prohibited_zones : list) -> bool:
            for p in points:
                x, y = map(float, np.ravel(p))
                for (p_min, p_max) in prohibited_zones:
                    x_min, y_min = p_min
                    x_max, y_max = p_max
                    margin = 0.2
                    x_min -= margin
                    x_max += margin
                    y_min -= margin
                    y_max += margin
                    if x_min <= x  <= x_max and y_min <= y  <= y_max:
                        return False
                if x >= 5.99 or x <= -0.7:
                    print("hit boundary")
                    return False
                if y >= 5.99 or y <= -0.99:
                    print("hit boundary")
                    return False
            return True
        def path_length(points : list):
            diffs = np.diff(np.array(points), axis=0)
            return np.sum(np.linalg.norm(diffs, axis=1))
        def random_target(points : list, prohibited_zones : list, x=cx, y=cy, yaw=yaw ,tolerance =10 )->list:
            iterations = 0
            min_dist = 5
            while True:
                if iterations >= 100:
                    iterations = 0
                    min_dist -= 1
                    tolerance += 45
                if min_dist <= 0:
                    return path_smoother(points)
                    break
                
                dist = random.uniform(min_dist, 6)
                angle_offset = np.deg2rad(random.uniform(-tolerance, -1))
                direction = np.array([
                    np.cos(yaw + angle_offset),
                    np.sin(yaw + angle_offset)
                ])
                target = (x + direction[0] * dist, y  + direction[1] * dist) 
                
                print(points)
                points.append(target)
                print(points)
                virtual = path_smoother(points)
                if validity_checker(virtual, prohibited_zones):
                    return virtual
                else:
                    points.pop()
                iterations +=1
                
        def make_unsmooth_path(points : list, prohibited_zones : list,yaw, x=cx, y=cy ,tolerance =20 )->list:
            base_yaw = yaw 
            iterations = 0
            min_dist = 0.5
            
            while path_length(points) < 5:
                if iterations >= 1000:
                    return points
                
                dist = random.uniform(min_dist, 1)
                angle_offset = np.deg2rad(random.uniform(-tolerance, tolerance))
                new_yaw = base_yaw + angle_offset
                direction = np.array([
                    np.cos(new_yaw),
                    np.sin(new_yaw)
                ])
                target = (x + direction[0] * dist, y  + direction[1] * dist) 

                """ dir_to_target = np.array([target[0] - x, target[1] - y])
                dir_to_target /= np.linalg.norm(dir_to_target) + 1e-6  
                heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
                alignment = np.dot(dir_to_target, heading_vec)

                if alignment <= 0.5:
                    print("DAAAAAAAAAAAAAAAAAAAA")
                    iterations += 1
                    continue
                """
                
                points.append(target)
                if candidate_is_valid(target, prohibited_zones):
                    prev_x = x
                    prev_y = y
                    x = target[0]
                    y = target[1]

                    yaw = math.atan2(y - prev_y, x - prev_x)
                else:
                    points.pop()
                iterations +=1
            return points

        def continue_forward(lookahead_dist : float, x = cx, y=cy, yaw=yaw ,step=step ):

            
            direction = np.array([np.cos(yaw), np.sin(yaw)])
            num_steps = int(lookahead_dist / step)
            print(lookahead_dist)
            
            for i in range(1, num_points + 1):
                dx = math.cos(yaw) * step * i
                dy = math.sin(yaw) * step * i
                
                if not candidate_is_valid((dx, dy), prohibited_zones):
                    break

                critical_points.append((cx + dx, cy + dy))
        
        path: Path2D = []
        
        #I will separate the yellow and blue cones for simplicity only
        #I will also add a distance parameter to compute path from the nearest to the farthest cone
        
        for cone in self.cones:
            distance = math.sqrt((cone.x - cx)**2 + (cone.y - cy)**2)
            if cone.color == 1:
                leftSiders.append((cone.x, cone.y, distance))
            else:
                rightSiders.append((cone.x, cone.y, distance))
        
        leftSiders = sorted(leftSiders, key= lambda c: c[2])
        rightSiders = sorted(rightSiders, key= lambda c: c[2])

        prohibited_zones = Prohibit_Zones()

        minimum = 0
        left_is_shorter = True

        initial_lookahead = (cx + math.cos(yaw)*0.4, cy + math.sin(yaw)*0.4)
        critical_points = [(cx, cy), (initial_lookahead)]
        path_points = [(cx, cy), (initial_lookahead)]

        #Now I will look in three scenarios, 1: there are two cones, 2: there is only one cone, 3: there are no cones
        if len(leftSiders) != 0 and len(rightSiders) != 0:
            minimum = min(len(leftSiders), len(rightSiders))
            if minimum == len(leftSiders):
                left_is_shorter = True
            else:
                left_is_shorter = False
            
            for i in range (minimum):
                real_cone = (
                (leftSiders[i][0] + rightSiders[i][0]) / 2.0, 
                (leftSiders[i][1] + rightSiders[i][1]) / 2.0)
                                        

                critical_points.append(real_cone)
                dir_x = math.cos(yaw)
                dir_y = math.sin(yaw)

                if i < len(leftSiders)-1 and i < len(rightSiders)-1:
                    next_mid = ((leftSiders[i+1][0] + rightSiders[i+1][0]) / 2.0,
                                (leftSiders[i+1][1] + rightSiders[i+1][1]) / 2.0)
                    dir_x = next_mid[0] - real_cone[0]
                    dir_y = next_mid[1] - real_cone[1]
                    norm = math.hypot(dir_x, dir_y)
                    dir_x, dir_y = dir_x / norm, dir_y / norm
                
                elif len(rightSiders) > minimum:
                    target_cone = rightSiders[minimum]
                    dir_x = target_cone[0] - real_cone[0]
                    dir_y = target_cone[1] - real_cone[1]
                
                elif len(leftSiders) > minimum:
                    target_cone = leftSiders[minimum]
                    dir_x = target_cone[0] - real_cone[0]
                    dir_y = target_cone[1] - real_cone[1]
                
                else:
                    dir_x, dir_y = math.cos(yaw), math.sin(yaw)

                # Add a "lookahead" control point ahead of the current midpoint
                lookahead_distance = 0.25  # tune 0.5–2.0 m
                lookahead_point = (real_cone[0] + lookahead_distance * dir_x * 0.5,
                                real_cone[1] + lookahead_distance * dir_y * 0.5)

                critical_points.append(lookahead_point)

                # Now I will start smoothening the path
                path_points = path_smoother(critical_points)               
        #This whole code does the same thing as the previous loop, except that it check whether it will go above or to the left of the yellow cone
        if len(leftSiders) <= minimum and len(rightSiders) > minimum:
            for x in range(minimum , len(rightSiders)):
                cx, cy = path_points[-1]
                yaw = yaws[-1]
                heading = np.array([math.cos(yaw), math.sin(yaw)])


                # Two possible ways to pass a yellow cone
                left_candidate  = np.array([rightSiders[x][0] - offset, rightSiders[x][1]])
                above_candidate = np.array([rightSiders[x][0], rightSiders[x][1] + offset])

                # Alignment: choose the one most aligned with heading
                to_left  = left_candidate - np.array([cx, cy])
                to_above = above_candidate - np.array([cx, cy])
                dot_left  = np.dot(heading, to_left)
                dot_above = np.dot(heading, to_above)

                if dot_above >= dot_left:
                    chosen_candidate = above_candidate
                else:
                    chosen_candidate = left_candidate
                
                critical_points.append(chosen_candidate)
                path_points = path_smoother(critical_points)
                
                virtual_path = path_smoother(critical_points)
                
                if validity_checker(virtual_path, prohibited_zones):
                    path_points = virtual_path
                
                else:
                    critical_points.pop()
                    if np.all(chosen_candidate == left_candidate):
                        chosen_candidate = above_candidate
                    else:
                        chosen_candidate = left_candidate
                    critical_points.append(chosen_candidate)
                
                    virtual_path = path_smoother(critical_points)
                    
                    if validity_checker(virtual_path, prohibited_zones):
                        path_points = virtual_path
                    
                    else: 
                        critical_points.pop()
                        if(x == minimum):
                            critical_points.pop()
                        break
                                   
        if len(rightSiders) <= minimum and len(leftSiders) > minimum:
            for x in range(minimum , len(leftSiders)):
                right_candidate = (leftSiders[x][0] + offset, leftSiders[x][1])
                below_candidate = (leftSiders[x][0], leftSiders[x][1] - offset)

                yaw = yaws[-1]
                heading = np.array([math.cos(yaw), math.sin(yaw)])
                
                cx = path_points[-1][0]
                cy = path_points[-1][1]

                # Two possible ways to pass a blue cone
                right_candidate = np.array([leftSiders[x][0] + offset, leftSiders[x][1]])    # to the right
                below_candidate = np.array([leftSiders[x][0], leftSiders[x][1] - offset])    # below

                # Alignment: choose the one most aligned with current heading
                to_right = right_candidate - np.array([cx, cy])
                to_below = below_candidate - np.array([cx, cy])
                dot_right = np.dot(heading, to_right)
                dot_below = np.dot(heading, to_below)

                # Choose the one better aligned with heading
                if dot_right >= dot_below:
                    chosen_candidate = right_candidate
                else:
                    chosen_candidate = below_candidate

                critical_points.append(chosen_candidate)
                
                virtual_path = path_smoother(critical_points)
                
                if validity_checker(virtual_path, prohibited_zones):
                    path_points = virtual_path
                
                else:
                    critical_points.pop()
                    if np.all(chosen_candidate == right_candidate):
                        chosen_candidate = below_candidate
                    else:
                        chosen_candidate = right_candidate
                    critical_points.append(chosen_candidate)
                
                    virtual_path = path_smoother(critical_points)
                    
                    if validity_checker(virtual_path, prohibited_zones):
                        path_points = virtual_path
                    
                    else: 
                        critical_points.pop()
                        if(x == minimum):
                            critical_points.pop()
                        break
        
        """if path_length(path_points) < 5:
            continue_forward(5 - path_length(path_points))
            print("Done")
            path_points = path_smoother(critical_points)
        """
        trials = 0
        copy_critical_points = critical_points
        while path_length(path_points) < 5.0:
            if(trials >= 5):
                critical_points.append(initial_lookahead)
                cx = path_points[-1][0]
                cy = path_points[-1][1]
                if not path_smoothed:
                    prevx = critical_points[-2][0]
                    prevy = critical_points[-2][1]
                    new_yaw = math.atan2(cy - prevy, cx - prevx) 
                    
                path_points = make_unsmooth_path(critical_points, prohibited_zones, new_yaw)
                break
            path_points = random_target(copy_critical_points, prohibited_zones)
            trials += 1

        path = path_points
        return path
