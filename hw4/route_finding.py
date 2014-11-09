import Image, ImageDraw
import math

imlen = 400
im = Image.new('RGB', (imlen, imlen), (255, 255, 255)) 
draw = ImageDraw.Draw(im) 

def comp(x1, x2, cx):
	return math.atan2(x1[1]-cx[1], x1[0]-cx[0]) < math.atan2(x2[1]-cx[1], x2[0]-cx[0])
	
def buble_sort(l):
	cx = [0,0]
	for i in range(0, len(l)):
		cx[0] += l[i][0]
		cx[1] += l[i][1]
	cx[0] /= len(l)
	cx[1] /= len(l)
	# print cx
	
	for i in range(0, len(l)):
		for j in range(0, len(l)-i-1):
			if (not comp(l[j], l[j+1], cx)):
				temp = l[j]
				l[j] = l[j+1]
				l[j+1] = temp

f = open('hw4_start_goal.txt', 'r')
nums = f.readline().split()
start = [float(nums[0]), float(nums[1])]
nums = f.readline().split()
goal = [float(nums[0]), float(nums[1])]
f.close()

f = open('hw4_world_and_obstacles_convex.txt', 'r')
num_obstacles = int(f.readline())
data = []
max_x = -10000000.0
for i in range(0, num_obstacles):
	num_vertices = int(f.readline())
	vertices = []
	for j in range(0, num_vertices):
		coord = f.readline().split()
		x = float(coord[0])
		y = float(coord[1])
		vertices.append([x, y])
		if abs(x) > max_x:
			max_x = abs(x)
		if abs(y) > max_x:
			max_x = abs(y)
			
	data.append(vertices)
f.close()

print max_x
scale = imlen/float(max_x)
for i in range(0, num_obstacles):
	#buble_sort(data[i])
	for j in range(0, len(data[i])):
		draw.line((data[i][j][0]*scale + imlen/2.0, 
			data[i][j][1]*scale + imlen/2.0, 
			data[i][(j+1)%len(data[i])][0]*scale + imlen/2.0, 
			data[i][(j+1)%len(data[i])][1]*scale + imlen/2.0), 
			fill=int(255.0/len(data[i])*j))
im.save('pic.bmp')