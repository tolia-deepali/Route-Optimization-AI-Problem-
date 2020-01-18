#!/bin/python

# put your routing program here!

import os,sys
import queue as Queue
import math
q=Queue.PriorityQueue()
q2=Queue.PriorityQueue()
q5=Queue.PriorityQueue()
q3=Queue.PriorityQueue()
q6=Queue.PriorityQueue()
q4=Queue.PriorityQueue()

# Edge class contains all the egdes with their distances as value and speed limits as speed
class edge:  
	
		
	def __init__(self,name,value,speed):
		self.name=name
		self.value=value
		self.speed=speed
#Reading the segments file and creating a bidirectional graph with each city containing its neighbours as edge object with
#its attributes name,weight(distance) and speed limit	

f1 = open('road-segments.text', 'r')
graph=dict()
cost=[]
for word in f1.readlines():
	a=list(word.split(' '))
	cost.append(a)
	b=edge(a[1],a[2],a[3])  
	c=edge(a[0],a[2],a[3])
	graph[a[0]]=graph.get(a[0],[])+[b]
	graph[a[1]]=graph.get(a[1],[])+[c]

#Reading all the Lat/Lon as key value pair in coords dictionary in line 36

f2 = open('city-gps.text','r')
coords=dict()
for word in f2.readlines():
	word=word[:-1] ##to strip \n in the end from the file
	a=list(word.split(' '))
	co=[a[1],a[2]]
	coords[a[0]]=co

#Given a list containing route, the Cal_cost function returns total miles of a route

def Cal_cost(route): ##function to check distance cost of a given path
	route_cost=0
	for i in range(len(route)-1):
		for d in cost:
			if route[i]==d[0] and route[i+1]==d[1] or route[i+1]==d[0] and route[i]==d[1]:
		  		 #to extract cost between two nodes
		  		route_cost+=int(d[2])
	return route_cost
def Cal_cost_time(route):
	route_cost=0
	for i in range(len(route)-1):
		for d in cost:
			if route[i]==d[0] and route[i+1]==d[1] or route[i+1]==d[0] and route[i]==d[1]:
		  		 #to extract cost between two nodes
		  		if(d[3].isdigit() and float(d[3])==0.0):
		  		 route_cost+=(int(d[2])/49.0)
		  		if(d[3].isdigit() and float(d[3])!=0.0):
		  		 route_cost+=(int(d[2])/float(d[3]))
	return route_cost

def BFS(graph,start,end):
	explored=[]  	##to keep a track of explored nodes already to not traverse them again in the future
	queue=[[start]] ## using a list within list to keep track of the path

	if start == end:
		return "done"
	while queue:
		path=queue.pop(0)
		
		node=path[-1]	##since every time we pop the whole path itself in the form of list, we extract the last appended node in list as in line 74
		if node not in explored:
			neighbours=graph[node]
			

			for neighbour in neighbours:
				new_path=list(path)
				new_path.append(neighbour.name)
				queue.append(new_path)
				if neighbour.name== end:
					return new_path
			explored.append(node)
	return "no path"



def DFS(graph,start,end,depth=-1): ##depth is given default value as -1 to act as flag (line 90) to check if are using IDS serch
	explored=[]
	queue=[[start]]

	if start == end:
		return "done"
	while queue:
		path=queue.pop() ##uses  a stack instead of queue as in case of BFS
		
		node=path[-1]
		
		if(depth == -1):  ##Flag to check if to run normal DFS or IDFS till certain depth specified by the User
			
		  if node not in explored:
			neighbours=graph[node]
			
			for neighbour in neighbours:
				new_path=list(path)
				new_path.append(neighbour.name)
				queue.append(new_path)
				if neighbour.name== end:
					return new_path
			explored.append(node)

		if(depth != -1):  ##Flag to check if to run normal DFS or IDFS till certain depth specified by the User
			if (len(path)<=(depth) and node not in explored):
				
				neighbours=graph[node]
			
				for neighbour in neighbours:
					new_path=list(path)
					new_path.append(neighbour.name)
					queue.append(new_path)
					if neighbour.name== end:
						return new_path
				explored.append(node)
	return "no path"


def IDS(graph,start,end,depth): 
	for i in range (depth):
		pathIDS=DFS(graph,start,end,i)
		
		if isinstance(pathIDS,list):  ##the DFS function can return no path for each iteration of depths as "no path" as in line 84,which is ignored 
			return pathIDS	
			break		  ##here and and returns only the valid path(route) returned by the user
	return "No path"


## For uniform cost search,I use a priority queue with tuple(values,[path]) which extracts path with least values through iteraions
## which is the whole goal of Uniform Cost Search,I've set the default cost function to distance here, but can be altered by exlpicity
## calling other cost measures
def UFS(graph,start,end,cost_type='distance'):
	explored=[]
	intial=0
	tup=(intial,[start])
	q.put(tup)
	if start == end:
		return "done"
	while q:
		path=q.get()
		node=path[1][-1]
		if node not in explored:
			neighbours=graph[node]
	
		
			for neighbour in neighbours:
				new_path=list(path[1])
				new_path.append(neighbour.name)
				distance_till_now=int(neighbour.value)+path[0]
				if(neighbour.speed.isdigit() and float(neighbour.speed)==0.0):
					time_till_now=float(neighbour.value)/49.0+path[0]
				if (neighbour.speed.isdigit() and float(neighbour.speed)!=0.0):
					time_till_now=float(neighbour.value)/float(neighbour.speed)+path[0]
				segments_till_now=len(path[1])+1
				
				if(cost_type=='distance'):
					tup_temp=(distance_till_now,new_path)
				if(cost_type=='time'):
					tup_temp=(time_till_now,new_path)
				if(cost_type=='segments'):
					tup_temp=(segments_till_now,new_path)


				q.put(tup_temp)
				if neighbour.name== end:
					return new_path
			explored.append(node)
	return "no path"

## Returns haversine distance bewteen two lat/lons later used in A*
def distance(origin, destination):  ## https://stackoverflow.com/questions/44743075/calculate-the-distance-between-two-coordinates-with-python
    """
    Calculate the Haversine distance.


    Examples
    --------
    >>> origin = (48.1372, 11.5756)  
    >>> destination = (52.5186, 13.4083)  
    >>> round(distance(origin, destination), 1)
    504.2
    """
    lat1, lon1 = origin
    lat2, lon2 = destination
    radius = 6371*0.62 # miles

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) * math.sin(dlon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius * c

    return d


## A star using distance as cost where we use h(n) as the haversine distance between the current node in contention to the goal node
def A(graph,start,end):
	explored=[]
	intial=0
	q1=dict()
	tup=(intial,[start])
	q.put(tup)
	
	l1=float(coords.get(end)[0])
	l2=float(coords.get(end)[1])
	destination_coords=(l1,l2) ##gets coordinates of destination to be later used in H(n) calculation
	if start == end:
		return "done"
	while q:
		path=q.get()

		node=path[1][-1]
		initial_dist=0
		##for calculating distance travelled till now
		for i in range(len(path[1])-1):
		
		 	if(coords.get(path[1][i]) and coords.get(path[1][i+1])!=None):
				l5=float(coords.get(path[1][i])[0])
				l6=float(coords.get(path[1][i])[1])
				l7=float(coords.get(path[1][i+1])[0])
				l8=float(coords.get(path[1][i+1])[0])
				initial_dist+=distance((l5,l6),(l7,l8))

			
		
		 
		neighbours=graph[node]
		for neighbour in neighbours:
				
				name_temp=neighbour.name
				if(coords.get(name_temp)!=None):
				
					l3=float(coords.get(name_temp)[0])
					l4=float(coords.get(name_temp)[1])
					current_coords=(l3,l4)
					h=distance(current_coords,destination_coords)
				else:
					h=0
				g=initial_dist+(int)(neighbour.value)
				
				f=g+h
				new_path=list(path[1])
				if((neighbour.name not in explored) or (neighbour.name in explored and f<q1.get(neighbour.name))):
				
					#new_path=list(path[1])
					new_path.append(neighbour.name)
				
					q.put((f,new_path))
					q1[neighbour.name]=f
				## I dont return the path as soon as goal node is encountered rather save it with its f and look for other possible paths
				## and then return the path with least value of f.
				if neighbour.name == end:
							q2.put((f,new_path))
				explored.append(node)
		
		if(q2.empty()!=1):
			p=q2.get()
			return p[1]
	return "no path"
## A star with segements as cost Ive used heuristic as (distance from current node to goal node/edge size=25(avg))
def A2(graph,start,end):
	explored=[]
	intial=0
	q1=dict()
	tup=(intial,[start])
	q5.put(tup)
	
	l1=float(coords.get(end)[0])
	l2=float(coords.get(end)[1])
	destination_coords=(l1,l2) ##gets coordinates of destination to be later used in H(n) calculation
	if start == end:
		return "done"
	while q5:
		path=q5.get()

		node=path[1][-1]
		initial_dist=len(path[1])
		

			
		
		 
		neighbours=graph[node]
		for neighbour in neighbours:
				
				name_temp=neighbour.name
				if(coords.get(name_temp)!=None):
				
					l3=float(coords.get(name_temp)[0])
					l4=float(coords.get(name_temp)[1])
					current_coords=(l3,l4)
					h=(float)(distance(current_coords,destination_coords)/25)
					

				else:
					h=0
				g=initial_dist+1
				
				f=g+h
				new_path=list(path[1])
				if((neighbour.name not in explored) or (neighbour.name in explored and f<q1.get(neighbour.name))):
				
					# new_path=list(path[1])
					new_path.append(neighbour.name)
				
					q5.put((f,new_path))
					q1[neighbour.name]=f
				if neighbour.name == end:
							q3.put((f,new_path))
				explored.append(node)
		# print (q2.qsize())
		if(q3.empty()!=1):
			p=q3.get()
			return p[1]
	return "no path"
# routeab=A(graph, 'Abbot_Village,_Maine','Cisco,_Texas')
# print("a2")
# print(len(routea))

## A star with segements as cost Ive used heuristic as (distance from current node to goal node/speed limit=49(avg))
def A3(graph,start,end):
	explored=[]
	intial=0
	q1=dict()
	tup=(intial,[start])
	q6.put(tup)
	
	l1=float(coords.get(end)[0])
	l2=float(coords.get(end)[1])
	destination_coords=(l1,l2) ##gets coordinates of destination to be later used in H(n) calculation
	if start == end:
		return "done"
	while q6:
		path=q6.get()

		node=path[1][-1]
		initial_dist=0
		neighbours=graph[node]
		for neighbour in neighbours:
				
				name_temp=neighbour.name
				if(coords.get(name_temp)!=None):
				
					l3=float(coords.get(name_temp)[0])
					l4=float(coords.get(name_temp)[1])
					current_coords=(l3,l4)
					temp_speed=float(neighbour.speed)
					if(temp_speed==0.0):
						temp_speed=49.0

					h=float(distance(current_coords,destination_coords)/49.0)
				else:
					h=0
					
				temp_speed=float(neighbour.speed)
				if(temp_speed==0.0):
						temp_speed=40.0
				g=path[0]+(float(neighbour.value)/49.0)
				
				f=g+h
				new_path=list(path[1])
				if((neighbour.name not in explored) or (neighbour.name in explored and f<q1.get(neighbour.name))):
				
					# new_path=list(path[1])
					new_path.append(neighbour.name)
				
					q6.put((f,new_path))
					q1[neighbour.name]=f
				if neighbour.name == end:
							q4.put((f,new_path))
				explored.append(node)
		
		if(q4.empty()!=1):
			p=q4.get()
			return p[1]
	return "no path"
start_city=sys.argv[1]
end_city=sys.argv[2]
routing_algo=sys.argv[3]
cost_f=sys.argv[4]

if routing_algo=="bfs" and (cost_f=="segments" or cost_f=="distance" or cost_f=="time"):
	if(cost_f=="segments"):
		routeBFS=BFS(graph,start_city,end_city)
		dist=Cal_cost(routeBFS)
		ti=Cal_cost_time(routeBFS)
		print "yes",dist,ti, ' '.join(routeBFS)
	if(cost_f=="time"):
		routeBFS=BFS(graph,start_city,end_city)
		dist=Cal_cost(routeBFS)
		ti=Cal_cost_time(routeBFS)
		print "no",dist,ti, ' '.join(routeBFS)
	if(cost_f=="distance"):
		routeBFS=BFS(graph,start_city,end_city)
		dist=Cal_cost(routeBFS)
		ti=Cal_cost_time(routeBFS)
		print "no",dist,ti, ' '.join(routeBFS)

if routing_algo=="dfs" and (cost_f=="segments" or cost_f=="distance" or cost_f=="time"):
	routeDFS=DFS(graph,start_city,end_city)
	dist=Cal_cost(routeDFS)
	ti=Cal_cost_time(routeDFS)
	print "no",dist,ti, ' '.join(routeDFS)
if routing_algo=="uniform":
	if(cost_f=="distance"):
		routeUFS_distance=UFS(graph,start_city,end_city)
		dist=Cal_cost(routeUFS_distance)
		ti=Cal_cost_time(routeUFS_distance)
		print "yes",dist,ti, ' '.join(routeUFS_distance)
	if(cost_f=="segments"):
		routeUFS_segments=UFS(graph,start_city,end_city,'segments')
		dist=Cal_cost(routeUFS_segments)
		ti=Cal_cost_time(routeUFS_segments)
		print "yes",dist,ti, ' '.join(routeUFS_segments)
	if(cost_f=="time"):
		routeUFS_time=UFS(graph,start_city,end_city,'time')
		dist=Cal_cost(routeUFS_time)
		ti=Cal_cost_time(routeUFS_time)
		print "yes",dist,ti, ' '.join(routeUFS_time)
if routing_algo=="ids" and (cost_f=="segments" or cost_f=="distance" or cost_f=="time"):
	routeIDS=IDS(graph,start_city,end_city,9999)
	dist=Cal_cost(routeIDS)
	ti=Cal_cost_time(routeIDS)
	print "yes",dist,ti, '  '.join(routeIDS)
if routing_algo=="astar":
	if (cost_f=="distance"):
		routeA_distance=A(graph,start_city,end_city)
		dist=Cal_cost(routeA_distance)
		ti=Cal_cost_time(routeA_distance)
		print "yes",dist,ti, '  '.join(routeA_distance)
	if(cost_f=="segments"):
		routeA_segments=A2(graph,start_city,end_city)
		dist=Cal_cost(routeA_segments)
		ti=Cal_cost_time(routeA_segments)
		print "yes",dist,ti, ' '.join(routeA_segments)
	if(cost_f=="time"):
		routeA_time=A3(graph,start_city,end_city9i)
		dist=Cal_cost(routeA_time)
		ti=Cal_cost_time(routeA_time)
		print "yes",dist,ti, '  '.join(routeA_time)







