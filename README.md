# HERE IS A REVIEW BY AMR KASSAB

# Note: 
please don't mind my old repositories as I didn't create a new professional account yet and this is my highschool account Y_Y, feel free to look around it but please don't judge me based on the projects' names T_T



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

4. In the end I tested this approach with three yellow cones in scenario 22, and three blue cones in scenario 21

5. I also added more scenarios, up to 27, which some of them worked properly and others didn't

# Notes:
Smooth paths are made by fitting a cubic curve into a set of so called critical points that I found earlier

Prohibited zones were made in the beginning of the algorithm so that it prohibits moving through the areas that I don't want the car to be in, for example: I don't want to be on the left of a yellow cone, or to the right of a blue cone, or near the edges of the screen or returning through two cones that I already passed



I think this approach is the best since I keep track of all the points I don't want to be around, I take the cones as a guide to move around the world so the path will be somewhat expected and calculated, I still put a scenario when I just leave the cones if not feasible and go on my own toward either random or calculated paths that I check its validity beforehand 

# Limitations
1. I think the biggest limitation of all is that the car needs to start at (0,0) to work properly, I tried keeping the approach simple and I noticed no change in prespective in the pre-made scenarios, but it definetly is a big limitation.
2. If the path isn't found it can take really dangerous manuevers
3. As it takes the center of the paired cones as a reference, it can be dangerous if the track isn't sit to be in that form.

# Notes on limitations
more intiricate handling can be added, like:
1. Make the decisions around the car's prespective not the world's prespective (This will solve most of the problems it can face)
2. Add a mechanism to only pair the closest of cones together 
3. Some vector math can be added and an algorithm like APF can be applied, but for that to happen I would have needed some guidance and a little bit more time in a better duration than this. Also I tried keeping it simple as per the Author's notes and made sure it solves the scenarios set properly without any slacking :)
