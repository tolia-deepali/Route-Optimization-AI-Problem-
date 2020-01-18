# Route Optimization AI Problem !
@author : Deepali Tolia

I)a description of how you formulated the search problem, including precisely defning the state space, the successor function, the edge weights, the goal state, and (if applicable) the heuristic function(s) you designed, including an argument for why they are admissible;

Formulation: The user will give a to and from city for which we have to find a route which will be optimal according to the constraint given by the user.

1.Initial State: All the segment containing start city

2.Goal State: The Current hop made has the End city (Example: if my end city is B and my current hop is B to C then its my 	               goal state)

3.Successor Function and edge weight and heuristic: According to the constraint the cost of every succesor is returned

   -distance - haversine distance for next city in the hop and destination i.e the end_city is calculated and added to the         existing cost

   -segments - for every hop segment++

   -time - distance given in the data divided by speed limit given in the data i.e the road-segments.txt and added to the          existing cost

   -mpg - calculated using the formula in the Instructions for "V" in the formula used the speed limit for that hop and added      to the existing cost

4.Admissibality: As we are haversine distance it will always give optimal solution

II) a brief description of how your search algorithm works
List of Functions and Code Flow

1. Main function
  -we take command line arguments in variables

  -we check if the given constraint is correct and the given cities name are in the data set

  -call parse_document function

  -call A star function

2. parse_document()

  -parse road-segments.txt in gv_rd_seg_lst list structure

  -parse city-gps.txt in gv_cty_gps_lst list structure
3. Using haversine function of Python directly
  -to calculate the GPS distance between two location which will be used in
   A* search heuristic for distance constraint

4. dist_calc()
  -it passes values to haversine_distance()
  -the values are extracted from gv_cty_gps_lst - latitude and longitude of current and destination city

5. cost_calc()

  -for every next hop possible we need to calculate cost of that hop which is done in this function

6. a_star_srch()

  -we chack if initial state is same as goal state or not

  -if not we initialize a priority queue
   we use priority queue and the priority queue uses the cost to pop the next segment with lowest cost calculated in cost_clac

  -Queue is initialized with the initial segments based on start city

  -the while loop runs till queue is empty in this case printing "INF" or till we reach goal state

  -we pop the value with the lowest cost an duse it as the current state

  -successors for this current state are calculated if the current state is not equal to goal state

  -for every successor we calculate the next cost, distance, time, total gas gallon, segments

  -we append the route as well over here for every successor

  -when the goal state is found we print the segment, distance travelled, time required, total gas gallons, and path

III)discussion of any problems you faced, any assumptions,simplications, and/or design decisions.
1.initially both files were parsed in list structure for time optimization it was changed to dictionary after comparison of      run time list structure took less time

2.bidirectional case handling : we switched the city names  where needed in the successor states

Cities which didn't have GPS details case handling : in dist_calc() if we do not find details in the given data we return 0.
if the returned value from dist_calc is 0 we consider the distance given for that segment in the dataset

3.the haversine module is not available on SICE server - added a function for it haversine_distance()

4.For mpg "V" in the formula given assumed the speed limit for that hop
5.total gas gallons = (hop distance/mpg for the hop)

### Code Run :
```
./route.py [start-city] [end-city] [cost-function]

```

For Eg :
```
./route.py Bloomington,_Indiana Indianapolis,_Indiana distance
```
