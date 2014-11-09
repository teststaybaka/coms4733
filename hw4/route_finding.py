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

def sub_v_v(v1, v2):
	return [v1[0]-v2[0], v1[1]-v2[1]]

def add_v_v(v1, v2):
	return [v1[0]+v2[0], v1[1]+v2[1]]	

def multi_v_n(v, n):
	return [v[0]*n, v[1]*n]

def nomalize(v):
	l = v[0]*v[0]+v[1]*v[1]
	l = math.sqrt(l)
	return [v[0]/l, v[1]/l]

def cross(v1, v2):
	return v1[0]*v2[1] - v1[1]*v2[0]

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

scale = imlen/float(max_x)
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
print cross([2,2],[2,2])
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
		while cross(edge1, edge2) > 0.0:
			v1 = multi_m_v(rotN90_matrix, edge1)
			v1 = multi_v_n(v1, radius)
			v1 = add_v_v(v1, vers[j])
			print 'v1', v1
			draw.point((v1[0]*scale + imlen/2.0, v1[1]*scale + imlen/2.0), 'blue')
			edge1 = multi_m_v(rotInterval_matrix, edge1)

		v2 = multi_m_v(rotN90_matrix, edge2)
		v2 = multi_v_n(v2, radius)
		v2 = add_v_v(v2, vers[j])
		print 'v2', v2
		draw.point((v2[0]*scale + imlen/2.0, v2[1]*scale + imlen/2.0), 'blue')

	extended_data.append(extended_vertices)




im.save('pic.bmp')