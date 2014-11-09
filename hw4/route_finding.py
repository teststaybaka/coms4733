import Image

f = open('hw4_start_goal.txt', 'r')
nums = f.readline().split()
start = [float(nums[0]), float(nums[1])]
nums = f.readline().split()
goal = [float(nums[0]), float(nums[1])]
f.close()

f = open('hw4_world_and_obstacles_convex.txt', 'r')
num_obstacles = f.readline()
print num_obstacles

f.close()