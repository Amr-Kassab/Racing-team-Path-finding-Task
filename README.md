# Racing-team-Path-finding-Task

# HERE IS A REVIEW BY AMR KASSAB

# Note: please don't mind my old repositories as I didn't create a new professional account yet and this is my highschool account Y_Y, feel free to look around it but please don't judge me based on the projects' names T_T



# Here starts a simple break down of the algorithm I used:

1. I didn't exactly know where should I go with the car as there are no end goals, so I decided to take the cones as a guide and try to folllow each of them moving in a smooth path

2. Cones were separated into blue and yellow cones,and three scenarios were set: 
    a.The cones makes pairs -> reach to move between them
    b.The yellow cones or blue cones are more than the other -> try reaching for its right side or its bottom if blue and left side or above if yellow
    c.If there are no cones or no clear path found -> Do the random smooth path generation for number of iterations then start doing manuevers with no smoothining

3. If a path between cones was not found there were some safety measures:
    a. The car continues to move in its direction if it isn't prohibited (I'll explain later on)
    b. The car chooses a random point and validate its path, if safe -> continue, if not -> look for another smoothed path till you reach a maximum number of iterations
    c. find a risky maneuver path to get out and continue your way till you reach a 5-meter long path

# Smooth paths are made by fitting a cubic curve into a set of so called critical points that I found earlier

# Prohibited zones were made in the beginning of the algorithm so that it prohibits moving through the areas that I don't want the car to be in, for example: I don't want to be on the left of a yellow cone, or to the right of a blue cone, or near the edges of the screen or returning through two cones that I already passed
