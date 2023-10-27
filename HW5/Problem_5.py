import itertools
import math
import matplotlib.pyplot as plt

waypoints = {
    "Start": (0, 0),
    "Goal 1": (2, 2),
    "Goal 2": (5, 3),
    "Goal 3": (3, 4),
    "Goal 4": (6, 4),
}

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

distance_table = {}
for wp1, coords1 in waypoints.items():
    for wp2, coords2 in waypoints.items():
        if wp1 != wp2:
            distance_table[(wp1, wp2)] = calculate_distance(coords1, coords2)

waypoint_permutations = list(itertools.permutations(waypoints.keys(), len(waypoints)))

best_path = None
min_distance = float('inf')

for perm in waypoint_permutations:
    current_distance = 0
    for i in range(len(perm) - 1):
        current_distance += distance_table[(perm[i], perm[i+1])]
    if current_distance < min_distance:
        min_distance = current_distance
        best_path = perm

best_path = ("Start",) + best_path

path_coordinates = [waypoints[wp] for wp in best_path]

x, y = zip(*path_coordinates)
plt.plot(x, y, marker='o', linestyle='-')
plt.title("Optimal TSP Path")
plt.xlabel("X-coordinate")
plt.ylabel("Y-coordinate")
for i, wp in enumerate(best_path):
    plt.annotate(wp, (x[i], y[i]))

plt.show()