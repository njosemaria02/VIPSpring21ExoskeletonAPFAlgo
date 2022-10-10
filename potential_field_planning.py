"""
Potential Field based path planner
author: Atsushi Sakai (@Atsushi_twi)
Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
# *** compares three positions at a time for major difference in direction
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True

# *** purpose of min xy and max xy?
# *** goal x, goal y, obstacle x, obstacle y, grid size, robot radius, start x, start y
def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy):
    # --- what exactly computing here?
    minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0 # *** min(min(obstacle x), robot x, goal x) - area width / 2 --- why? smallest x for map?
    miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0 # *** min(min(obstacle y), robot y, goal y) - area width / 2 --- why? smallest y for map?
    maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0 # *** max(max(obstacle x), robot x, goal x) + area width / 2 --- why? largest x for map?
    maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0 # *** max(max(obstacle y), robot y, goal y) + area width / 2 --- why? largest y for map?
    xw = int(round((maxx - minx) / reso)) # *** round up ((largest x (?) - smallest x (?)) / grid size) --- why? new x width?
    yw = int(round((maxy - miny) / reso)) # *** round up ((largest y (?) - smallest y (?)) / grid size) --- why? new y width?

    # +++ way of seeing xw, yw numerically
    # creates the dimensions of the illustration
    # print(xw)
    # print(yw)

    # print(minx)
    # print(miny)
    # print(maxx)
    # print(maxy)

    # calc each potential
    # *** initializes potential map with completely zero values for entire grid?
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    # *** for each x value within x range of grid
    for ix in range(xw):
        x = ix * reso + minx # *** x = x index * grid size + smallest x --- why?

    # *** for each y value within y range of grid
        for iy in range(yw):
            y = iy * reso + miny # *** y = y index * grid size + smallest y --- why?
            ug = calc_attractive_potential(x, y, gx, gy) # *** goal potential = calcAttractivePotential(x, y, goal x, goal y)
            uo = calc_repulsive_potential(x, y, ox, oy, rr) # *** obstacle potential = calcRepulsivePotential(x, y, obstacle x, obstacle y, robot radius)
            uf = ug + uo # *** potential force = goal potential + obstacle potential
            pmap[ix][iy] = uf # *** potential force gets assigned to that specific coordinate within map

    # +++ way of better seeing pmap purpose within terminal numerically
    # for a in range(xw):
    #     for b in range(yw):
    #         if (a < 20):
    #             print(str(a) + ", " + str(b) + ": " + str(pmap[a][b]))
    #         else:
    #             break

    # *** after finding potential force for every coordinate within map, return the map, the smallest x, and the smallest y
    return pmap, minx, miny

# *** meant for finding attractive potential of goal
def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)
    # *** .5 * attractive potential gain (?) * hypotenuse (x - goal x, y - goal y)
    # *** in simpler terms: constant * dist(robot, goal)

# *** remember: goal has attractive potential
#               obstacles have repulsive potential and force

# *** meant for finding repuslive potential of obstacle
def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf") # --- minimum distance = infinity
    # *** enumerate is used for looping
    # +++ for each x value within obstacle x?
    # --- example: ox = 11; loop goes 1, 2, 3, 4,...,11?
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i]) # *** distance = hypotenuse(x - obstacle x[x within ox], y - obstacle y[y within oy])
        if dmin >= d: # *** if current distance <= min distance
            dmin = d  # *** reassign min distance to be this distance
            minid = i # *** make min id this index

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])
    # *** robot distance to obstacle (?) = 
    # *** hypotenuse(x - obstacle x[smallest index for smallest distance within obstacle x], y - obstacle y[smallest index for smallest distance within obstacle y])

    # *** if robot distance to obstacle <= robot radius /// if robot close enough to hit obstacle
    if dq <= rr:
        if dq <= 0.1: # *** if robot SUPER close to obstacle and going to hit, reassign smallest distance to 0.1, assuming for easier computing
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
        # *** .5 * repulsive potential gain * (1 / robot distance to obstacle - 1 / robot radius) squared
    else: # *** else if robot not close enough to obstacle for repulsive force to matter
        return 0.0
        # *** no repulsive potential

# +++ defines how the robot is supposed to move visually in the graph
def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion

def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    # --- if start wobbling, edit the generated coordinate to keep it from wobbling?
    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set -- correlation to wobbling?
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

# *** start x, start y, goal x, goal y, obstacle x, obstacle y, grid size, robot radius
def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)

    # search path
    d = np.hypot(sx - gx, sy - gy) # *** robot x - goal x, robot y - goal y
    # +++ what is np.hypot()?
    # *** np.hypot() calculates the hypotenuse of a triangle
    # *** in this instance, finding hypotenuse of (robot x - goal x) by (robot y - goal y) triangle
    # *** this gives the shortest path to the goal regardless of obstacles in the way
    ix = round((sx - minx) / reso) # *** robot x - smallest x / grid size --- why? what is result? initial robot x?
    iy = round((sy - miny) / reso) # *** robot y - smallest y / grid size --- why? what is result? initial robot y?
    gix = round((gx - minx) / reso) # *** goal x - smallest x / grid size --- why? what is result? goal x?
    giy = round((gy - miny) / reso) # *** goal y - smallest y / grid size --- why? what is result? goal y?
    # +++ do all these calculation to acclimate the positions to the grid?
    # +++ all these calculations find the coordinate of start and goal with respect to size of grid
    # print(ix)
    # print(iy)
    # print(gix)
    # print(giy)

    # *** if showAnimation variable true
    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k") # *** black star marker for inital robot start coordinates
        plt.plot(gix, giy, "*m") # *** magenta start marker for goal coordinates // doesnt show as magenta though lol

    rx, ry = [sx], [sy] # *** robot x, robot y = start robot x, start robot y
    motion = get_motion_model() # +++ gets model for how robot is supposed to move in visualization
    previous_ids = deque() # --- what is the point of the deque here? +++ deque is for oscillations

    # +++ this entire section of code here determines the robot's next step with respect to 
    # motion model and potential
    # looking at each step available within the motion model, find the one with the smallest potential
    # convert that coordinate to grid dimensions
    print("d: " + str(d) + " | reso: " + str(reso));
    while d >= reso: # *** while distance(robot, goal) >= grid size
        minp = float("inf") # *** min potential (?) = infinity
        minix, miniy = -1, -1 # --- ?
        for i, _ in enumerate(motion): # ++ for each motion type in motion model
            inx = int(ix + motion[i][0]) # ++ the coordinate for this potential step within the motion model
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0: # *** if data out of bounds of map
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny] # *** potential = potential at coordinates inx, iny
            if minp > p: # *** if potential at these coordinates < minimum potential
                # ++ COORDINATE REASSIGNMENT LOGIC HERE
                minp = p # ** reassign min potential to be this potential
                minix = inx # ** reassign min ix to be this potential x
                miniy = iny # ** reassign min iy to be this potential y
            
            # print("i: " + str(i) + " | _:" + str(_) + " | inx: " + str(inx) + " | iny: " + str(iny) + " | p:" + str(p))
        ix = minix # *** ix = x with smallest potential
        iy = miniy # *** iy = y with smallest potential

        # +++ establish final coordinates with relation to grid dimensions
        xp = ix * reso + minx # *** x with smallest potential * grid size + smallest x = next x step?
        yp = iy * reso + miny # *** y with smallest potential * grid size + smallest y = next y step?
        d = np.hypot(gx - xp, gy - yp) # *** shortest dist(goal - next x step, goal - next y step)
            # why is this recorded lol
        # +++ add to list of robot steps with respect to original value passed in
        rx.append(xp) # *** add next x step to robot x coor
        ry.append(yp) # *** add next y step to robot y coor
        

        # +++ for testing purposes!
        # print("smallest: " + str(minp))
        print("ix: " + str(ix) + " | iy: " + str(iy) + " || x: " + str(xp) + " | y: " + str(yp))

        # if (ix != inx):
        #     print("x difference!")
        
        # if (iy != iny):
        #     print("y difference!")

    

        # +++ checks if coordinate causes wobbling
        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            plt.plot(ix, iy, ".r") # *** red point marker for path
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry # *** return robot x collection/y collection

# *** data passed in is map with updated potential forces
def draw_heatmap(data):
    data = np.array(data).T # --- converts map to array then transposes it to be ordered as it was created (makes it look like a 2D array for example)
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues) # *** assuming this makes visualization of map


def main():
    print("potential_field_planning start")

    # *** everything off by 30, both x and y, in figure
    # *** initializes robot start x/y, goal x/y, grid size, robot radius
    sx = 0.0  # start x position [m]
    sy = 10.0  # start y positon [m]
    gx = 30.0  # goal x position [m]
    gy = 30.0  # goal y position [m]
    grid_size = .5  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    # *** play around with the numbers!
    # *** initializes four obstacles with obstacle x/y
    ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
    oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]

    # *** checks if boolean var show_animation is true
    # *** shows animation by setting grid to true and axis to equal -- what does this mean?
    if show_animation:
        plt.grid(True) # -- what does this mean?
        plt.axis("equal") # -- what does this mean?

    # path generation
    # *** passes initialized values from beginning of method to geneerate path
    # --- what is "_, _" though, and why assigning the method to it?
    # *** method returns "rx, ry", so assuming robot x/y
    # --- dont know how that's a useful/valuable return here though?
    a, b = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    # returns all coordinates in x list and y list
    print(a)
    print(b)

    # *** checks if boolean var show_animation is true
    # *** shows animation AFTER path generation
    if show_animation:
        plt.show()

# *** whenever within main, execute these functions
if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")