import Image

f = open('hw4_start_goal.txt', 'r')
nums = f.readline().split()
start = [float(nums[0]), float(nums[1])]
nums = f.readline().split()
goal = [float(nums[0]), float(nums[1])]
f.close()

f = open('hw4_world_and_obstacles_convex.txt', 'r')
num_obstacles = int(f.readline())
data = []
for i in range(0, num_obstacles):
	num_vertices = int(f.readline())
	vertices = []
	for j in range(0, num_vertices):
		coord = f.readline().split()
		x = float(coord[0])
		y = float(coord[1])
		vertices.append([x, y])
	data.append(vertices)
f.close()