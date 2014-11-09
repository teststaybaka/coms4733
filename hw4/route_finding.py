import Image, ImageDraw
import math

imlen = 1000
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

def multi_m_v(m, v):
	return [m[0][0]*v[0]+m[0][1]*v[1], m[1][0]*v[0]+m[1][1]*v[1]]

def cross(v1, v2):
	return v1[0]*v2[1] - v1[1]*v2[0]

def dot(v1, v2):
	return v1[0]*v2[0] + v1[1]*v2[1]

def sub_v_v(v1, v2):
	return [v1[0]-v2[0], v1[1]-v2[1]]

def add_v_v(v1, v2):
	return [v1[0]+v2[0], v1[1]+v2[1]]	

def multi_v_n(v, n):
	return [v[0]*n, v[1]*n]

def nomalize(v):
	l = dot(v, v)
	l = math.sqrt(l)
	return [v[0]/l, v[1]/l]

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

radius = 0.2
rotN90_matrix = [[0.0,1.0],[-1.0,0.0]]
invertal_angle = 15.0/360*2*math.pi
rotInterval_matrix = [[math.cos(invertal_angle), -math.sin(invertal_angle)], [math.sin(invertal_angle), math.cos(invertal_angle)]]
# v = [3,3]
# print multi_m_v(rotN90_matrix, v)

scale = imlen/2/float(max_x)
for i in range(0, num_obstacles):
	#buble_sort(data[i])
	vers = data[i]
	for j in range(0, len(vers)):
		draw.line((vers[j][0]*scale + imlen/2.0, 
			vers[j][1]*scale + imlen/2.0, 
			vers[(j+1)%len(vers)][0]*scale + imlen/2.0, 
			vers[(j+1)%len(vers)][1]*scale + imlen/2.0), 
			fill=int(255.0/len(vers)*j))

extended_data = []
for i in range(1, num_obstacles):
	buble_sort(data[i])
	vers = data[i]
	extended_vertices = []
	for j in range(0, len(vers)):
		draw.line((vers[j][0]*scale + imlen/2.0, 
			vers[j][1]*scale + imlen/2.0, 
			vers[(j+1)%len(vers)][0]*scale + imlen/2.0, 
			vers[(j+1)%len(vers)][1]*scale + imlen/2.0), 
			fill=int(255.0/len(vers)*j))	

		edge1 = sub_v_v(vers[j], vers[(j-1)%len(vers)])
		edge2 = sub_v_v(vers[(j+1)%len(vers)], vers[j])
		edge1 = nomalize(edge1)
		edge2 = nomalize(edge2)
		while cross(edge1, edge2) > 0.0001:
			v1 = multi_m_v(rotN90_matrix, edge1)
			v1 = multi_v_n(v1, radius)
			v1 = add_v_v(v1, vers[j])
			extended_vertices.append(v1 + [-1, -1, 1000000.0, 0])
			# print 'v1', v1#, edge1
			draw.point((v1[0]*scale + imlen/2.0, v1[1]*scale + imlen/2.0), 'blue')
			edge1 = multi_m_v(rotInterval_matrix, edge1)

		v2 = multi_m_v(rotN90_matrix, edge2)
		v2 = multi_v_n(v2, radius)
		v2 = add_v_v(v2, vers[j])
		extended_vertices.append(v2 + [-1, -1, 1000000.0, 0])
		# print 'v2', v2#, edge2
		draw.point((v2[0]*scale + imlen/2.0, v2[1]*scale + imlen/2.0), 'blue')

	extended_data.append(extended_vertices)

extended_data.append([start + [-1, -1, 0, 0], goal + [-1, -1, 10000000.0, 0]])

while True:
	minimum = 10000000.0
	for i in range(0, len(extended_data)):
		vers = extended_data[i]
		for j in range(0, len(vers)):
			if vers[j][4] < minimum and vers[j][5] == 0:
				minimum = vers[j][4]
				cur = [i, j]
	print 'find', cur
	extended_data[cur[0]][cur[1]][5] = 1
	if extended_data[cur[0]][cur[1]][0] == goal[0] and extended_data[cur[0]][cur[1]][1] == goal[1]:
		break

	for i in range(0, len(extended_data)):
		vers = extended_data[i]
		for j in range(0, len(vers)):
			if vers[j][5] == 0:
				dist = math.sqrt(dot(vers[j], vers[j]) + dot(extended_data[cur[0]][cur[1]], extended_data[cur[0]][cur[1]]))
				if dist + extended_data[cur[0]][cur[1]][4] < vers[j][4]:
					vers[j][4] = dist + extended_data[cur[0]][cur[1]][4]
					vers[j][2] = cur[0]
					vers[j][3] = cur[1]

while True:
	next_cur = [extended_data[cur[0]][cur[1]][2], extended_data[cur[0]][cur[1]][3]]
	draw.line((extended_data[cur[0]][cur[1]][0]*scale + imlen/2.0, 
		extended_data[cur[0]][cur[1]][1]*scale + imlen/2.0, 
		extended_data[next_cur[0]][next_cur[1]][0]*scale + imlen/2.0,
		extended_data[next_cur[0]][next_cur[1]][1]*scale + imlen/2.0,),
		fill=0)
	print 'res', cur, extended_data[cur[0]][cur[1]], next_cur
	cur = next_cur
	if extended_data[cur[0]][cur[1]][4] == 0:
		break

im.save('pic.bmp')