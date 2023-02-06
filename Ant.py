import Rhino.Geometry as rg
import random
import math
import copy
import System.Drawing as drawing
import time

#https://www.youtube.com/watch?v=X-iSQQgOd1A&t=4s&ab_channel=SebastianLague
#This ant colony optimization algorithm and slim mold is based on the video above]
#The algorithm is extracted from the video, all python code is written from scratch\
#The concept of Physarum is based on https://uwe-repository.worktribe.com/output/980579

#Code: Chris Kang

class AgentSystem: #Declaring agent system class that has agents, food location, and pheromone grid

    def __init__(self, AntCount, FoodCount, size, resolution):

        self.Agents = [] #list of agents
        self.size = size #internalize size of the system
        self.resolution = resolution #internalize the resolution of the grid
        self.Grid = PheromoneGrid(self.size, self.resolution, 1) #2D grid of pheromone data
        self.Foods = Food(FoodCount) #list of food sources located randomly

        for i in range(0, AntCount): #create a number of ant agents
            agent = Agent(self.size) #create an agent at random position
            agent.System = self #Connect the agent to the system
            agent.FindTrail(self.Foods) #find the trail to the food
            self.Agents.append(agent)
        
    def Update(self): 
        for agent in self.Agents:
            agent.Update(self.Foods, self.Grid) #update the agent position and velocity
    
        self.Grid.Update(self) #update the grid of pheromone data

class Agent:

    def __init__(self, size):

        self.System = None #reference to the system
        self.Size = size #inherit the size of the system

        #random initial agent position
        self.Position = rg.Point3d(random.uniform(0.0,self.Size),random.uniform(0.0,self.Size), 0)

        #random initial agent velocity
        self.Velocity = rg.Vector3d(random.uniform(-self.Size/100,self.Size/100),random.uniform(-self.Size/100,self.Size/100),0)

        #Set the initial velocity to 1 unit
        self.Velocity.Unitize()

        #target position for path planning
        self.Target = None 

        #point list for path planning
        self.Path = [] 
        
        #counter for path planning. ie) the current point in the path list
        self.Counter = 0

        #how many loops has the ant gone through the path list
        self.RunCounter = 0

    #Trail Following Behavior
    def FindClosestPoint(self, foods): 
        #function to find the closest food position to the current position
        #this function is recursive
        
        counter = 0
        if counter == 0: #start with the current position
            currentPos = self.Position

        bestDistance = self.Size * math.sqrt(2) #set the best distance to the maximum possible distance
        ds = [] #list of distances
        
        for food in foods.Position:

            #For each food location, find the distance to the current position
            line = food - currentPos
            distance = line.Length

            #if the distance is less than the best distance, update the best distance and the closest point
            if distance < bestDistance:

                ds.append(distance)
                bestDistance = distance

                closestpoint = food

        #update the current position to the closest point
        currentPos = closestpoint
        #add the closest point to the path list
        self.Path.append(currentPos)

        #remove the closest point from the list of food locations since point has been visited
        foods.Position.remove(closestpoint)

        #if there are still food locations left, find the closest point to the current position
        if len(foods.Position) > 0:
            self.FindClosestPoint(foods)

    def FindTrail(self, foods):
        #for each agent, deep copy the food locations and find the closest point path
        foodscopy = copy.deepcopy(foods)
        #output is self.path which is a list of points that the agent will follow
        self.FindClosestPoint(foodscopy)
        
    def SteerWithTrail(self, steerStrength):
        #This function makes the agent follow the path list with a certain steering strength

        if self.Counter < len(self.Path)-1 and self.RunCounter < loopCount: #if the agent has not reached the end of the path list and has not looped through the path list more than the loop count
            if self.Counter == 0: #sets the target to the first point in the path list
                self.Target = self.Path[self.Counter]
            
            #add the oriented vector from the current position to the target position to the current velocity vector
            orientedVector = self.Target - self.Position 
            orientedVector.Unitize()
            self.Velocity += orientedVector * steerStrength

            #if the agent is within a certain distance of the target, update the target to the next point in the path list
            if self.Position.DistanceTo(self.Target) < foodProximity:
                self.Counter += 1
                self.Target = self.Path[self.Counter]
        #if the agent has reached the end of the path list, reset the counter and update the target to the first point in the path list
        else:
            self.Counter = 0
            self.Target = self.Path[self.Counter]
            self.RunCounter += 1

    #Boundary Behavior
    def ReflectAtBoundary(self):
        #if the ant is within 3 units of the boundary, rotate away from the boundary
        #angle of rotation is proportional to the distance from the boundary
        #if the ant is beyond the boundary, reflect the velocity vector
        if min((size - self.Position.X), self.Position.X) < 3: 
            if self.Position.X < 1:
                self.Position.X = 1
                self.Velocity.X = - self.Velocity.X
            elif self.Position.X > self.Size-1:
                self.Position.X = self.Size-1
                self.Velocity.X = - self.Velocity.X
            else:    
                if self.Position.X > self.Size/2: #if the ant is getting closer to the right side of the grid
                    self.Velocity.Rotate(-math.pi/2 * math.pow((1 / self.Size - self.Position.X),3), rg.Vector3d(0,0,1))

                else: #if the ant is getting closer to the left side of the grid
                    self.Velocity.Rotate(math.pi/2 * math.pow((1 / self.Position.X),3), rg.Vector3d(0,0,1))
            
        if min((size - self.Position.Y), self.Position.Y) < 3:
            if self.Position.Y < 1:
                self.Position.Y = 1
                self.Velocity.Y = - self.Velocity.Y
            elif self.Position.Y > self.Size-1:
                self.Position.Y = self.Size-1
                self.Velocity.Y = - self.Velocity.Y
            else:
                if self.Position.Y > self.Size/2: #if the ant is getting closer to the top side of the grid
                    self.Velocity.Rotate(-math.pi/2 * math.pow((1 / self.Size - self.Position.Y),3), rg.Vector3d(0,0,1))
                else: #if the ant is getting closer to the bottom side of the grid
                    self.Velocity.Rotate(math.pi/2 * math.pow((1 / self.Position.Y),3), rg.Vector3d(0,0,1))

    #Pheromone Behavior
    def Sense(self, angle):
        #This function returns a vector that is rotated by a certain angle from the current velocity vector
        if angle == 0:
            return self.Velocity
        
        else:
            self.Velocity.Rotate(angle, rg.Vector3d.ZAxis)
    
            vectorTilted = self.Velocity * 1

            self.Velocity.Rotate(-angle, rg.Vector3d.ZAxis)

            return vectorTilted

    def SteerWithPheromone(self, scale):
        angle = math.pi/4 #sensing angle is 45 degrees

        #get the points in front, to the right, and to the left of the ant
        pointForward = self.Position + self.Sense(0)
        pointRight = self.Position + self.Sense(angle)
        pointLeft = self.Position + self.Sense(-angle)

        #if the points are within the grid, get the pheromone values at those points
        if pointForward.X > 0 and pointForward.Y > 0 and pointForward.X < self.Size and pointForward.Y < self.Size \
            and pointRight.X > 0 and pointRight.Y > 0 and pointRight.X < self.Size and pointRight.Y < self.Size \
            and pointLeft.X > 0 and pointLeft.Y > 0 and pointLeft.X < self.Size and pointLeft.Y < self.Size:

            weightForward = self.System.Grid.Grid[int(pointForward.X/self.System.resolution)][int(pointForward.Y/self.System.resolution)]["Black"]
            weightRight = self.System.Grid.Grid[int(pointRight.X/self.System.resolution)][int(pointRight.Y/self.System.resolution)]["Black"]
            weightLeft = self.System.Grid.Grid[int(pointLeft.X/self.System.resolution)][int(pointLeft.Y/self.System.resolution)]["Black"]
            
            #if the pheromone value on the right is greatest, rotate the ant to the right
            if weightRight > weightForward and weightRight > weightLeft:
                self.Velocity = self.Velocity.Add(self.Velocity, self.Sense(angle) * scale)
                self.Velocity.Unitize()
            #if the pheromone value on the left is greatest, rotate the ant to the left
            elif weightLeft > weightForward and weightLeft > weightRight:
                self.Velocity = self.Velocity.Add(self.Velocity, self.Sense(-angle) * scale)
                self.Velocity.Unitize()
            #if the pheromone value in front is greatest, do not steer
            else:
                pass

    #Update the ant's position and velocity
    def Update(self, Food, grid):
        
        self.ReflectAtBoundary()
        self.SteerWithPheromone(SteerWithPheromoneScale)
        self.SteerWithTrail(SteerWithFoodScale)

        self.Velocity.Unitize()
        self.Position += self.Velocity


class Food: #Food is a list of points at random locations within the grid

    def __init__(self, Foodcount):
        self.Position = []

        for i in range(0,Foodcount):
            self.Position.append(rg.Point3d(random.uniform(0.0,size),random.uniform(0.0,size), 0))

class PheromoneGrid: #Pheromone grid is a 2D array of pheromone values, at white and black
    
    def __init__(self, size, resolution, initValue):
        #initialize 2D grid with size and resolution
        #each cell has value "white" and value "black"
        self.Num = int(size/resolution) + 1 
        self.Grid = []
        self.System = None
        self.Resolution = resolution
        self.Next = []
        
        for i in range(0,self.Num):
            self.Grid.append([])
            self.Next.append([])
            for j in range(0,self.Num):
                self.Grid[i].append({"White": initValue, "Black": 0})
                self.Next[i].append({"White": initValue, "Black": 0})

    def Update(self, AgentSystem):
        #inherit agent position from agent system
        #and release pheromone at agent position
        for ant in AgentSystem.Agents:
            self.Grid[int(ant.Position.X/self.Resolution)][int(ant.Position.Y/self.Resolution)]["Black"] += AntPheromoneReleaseRate
        #release pheromone at food position
        for food in AgentSystem.Foods.Position:
            self.Grid[int(food.X/self.Resolution)][int(food.Y/self.Resolution)]["Black"] += FoodPheromoneReleaseRate
        
        #Grid has three behaviors: Decay, Diffuse, Reaction-Diffusion
        self.Decay(decayRate)
        self.Diffuse()
        self.ReactionDiffusion(RD_DA, RD_DB, RD_F, RD_K)
        self.Swap()
  
    def Diffuse(self):
        #Diffusion is a process where a substance spreads out from a source
        #It is calucated from the average of 3 x 3 grid of cells
        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):

                    self.Next[x][y]["White"] = \
                    (self.Grid[x][y]["White"] + \
                    self.Grid[x+1][y]["White"] + \
                    self.Grid[x-1][y]["White"] + \
                    self.Grid[x][y+1]["White"] + \
                    self.Grid[x][y-1]["White"] + \
                    self.Grid[x-1][y-1]["White"] + \
                    self.Grid[x+1][y-1]["White"] + \
                    self.Grid[x+1][y+1]["White"] + \
                    self.Grid[x-1][y+1]["White"]) / 9

                    self.Next[x][y]["Black"] = \
                    (self.Grid[x][y]["Black"] + \
                    self.Grid[x+1][y]["Black"] + \
                    self.Grid[x-1][y]["Black"] + \
                    self.Grid[x][y+1]["Black"] + \
                    self.Grid[x][y-1]["Black"] + \
                    self.Grid[x-1][y-1]["Black"] + \
                    self.Grid[x+1][y-1]["Black"] + \
                    self.Grid[x+1][y+1]["Black"] + \
                    self.Grid[x-1][y+1]["Black"]) / 9 
                else:
                    self.Next[x][y]["White"] = 1
                    self.Next[x][y]["Black"] = 0

    def ReactionDiffusion(self, D_a, D_b, f, k):
        #Source : https://www.youtube.com/watch?v=BV9ny785UNc&t=983s&ab_channel=TheCodingTrain
        #Source : https://www.karlsims.com/rd.html
        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):

                        a = self.Grid[x][y]["White"]
                    
                        b = self.Grid[x][y]["Black"]

                        self.Next[x][y]["White"] = a + (D_a * self.laplace(x,y, "White")) - (a * b * b) + (f * (1 - a))
                        self.Next[x][y]["Black"] = b + (D_b * self.laplace(x,y, "Black")) + (a * b * b) - ((k + f)* b)

                        self.Next[x][y]["White"] = self.constrain(self.Next[x][y]["White"],0,1)

                        self.Next[x][y]["Black"] = self.constrain(self.Next[x][y]["Black"],0,0.5)

    def laplace(self, x, y, selector):
        #3x3 laplace filter
        #"The Laplacian is performed with a 3x3 convolution with center weight -1, adjacent neighbors .2, and diagonals .05.
        if selector == "White":
            sumA = 0
            sumA += self.Grid[x][y]["White"] * -1
            sumA += self.Grid[x-1][y]["White"] * 0.2
            sumA += self.Grid[x+1][y]["White"] * 0.2
            sumA += self.Grid[x][y+1]["White"] * 0.2
            sumA += self.Grid[x][y-1]["White"] * 0.2

            sumA += self.Grid[x-1][y-1]["White"] * 0.05
            sumA += self.Grid[x+1][y-1]["White"] * 0.05
            sumA += self.Grid[x+1][y+1]["White"] * 0.05
            sumA += self.Grid[x-1][y+1]["White"] * 0.05
            return sumA
        else:
            sumB = 0
            sumB += self.Grid[x][y]["Black"] * -1
            sumB += self.Grid[x-1][y]["Black"] * 0.2
            sumB += self.Grid[x+1][y]["Black"] * 0.2
            sumB += self.Grid[x][y+1]["Black"] * 0.2
            sumB += self.Grid[x][y-1]["Black"] * 0.2

            sumB += self.Grid[x-1][y-1]["Black"] * 0.05
            sumB += self.Grid[x+1][y-1]["Black"] * 0.05
            sumB += self.Grid[x+1][y+1]["Black"] * 0.05
            sumB += self.Grid[x-1][y+1]["Black"] * 0.05

            return sumB

    def constrain(self, val, min_val, max_val):
        #keeps values between 0 and 1
        #source : https://stackoverflow.com/questions/34837677/a-pythonic-way-to-write-a-constrain-function
        return min(max_val, max(min_val, val))
  
    def Decay(self, Rate):
        #Decay function: reduces the amount of black per frame
        for x in range(self.Num):
            for y in range(self.Num):
                self.Grid[x][y]["Black"] *= Rate
    
    def Swap(self):
        #Swaps the grid and next grid
        #Calculation is done on the next grid, then swapped
        temp = self.Grid
        self.Grid = self.Next
        self.Next = temp

    def Drawmesh(self):
        #Draws 2D mesh with colors according to the amount of white and black
        displayMesh = rg.Mesh()
        colors = []

        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):
                    #code from long's lecture
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution-1,y*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution-1,y*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution-1,(y+1)*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution-1, (y+1)*self.Resolution-1,0))

                    v = displayMesh.Vertices.Count

                    displayMesh.Faces.AddFace(v-4,v-3,v-2,v-1)

                    value = (self.Grid[x][y]["White"]-self.Grid[x][y]["Black"])* 255

                    if value > 255:
                        value = 255
                    elif value < 0:
                        value = 0

                    colors.append(drawing.Color.FromArgb(0, value,value,value))
            
        for i in range(len(displayMesh.Faces)):
            
            face = displayMesh.Faces[i]

            color = colors[i]

            displayMesh.VertexColors.SetColor(face,color)


        return displayMesh
    
    def PlotValue(self):
        #Plots the value of black to use in surface conversion
        points = []
        for x in range(self.Num):
            for y in range(self.Num):
                points.append(self.Grid[x][y]["Black"])
        return points

#Global Variables

FoodNum = 40 #number of food
antNum = 500 #number of ants
size = 100 #size of the grid
resolution = 0.5 #resolution of the grid

decayRate = 0.93 
RD_DA = 1 #DA in Reactions and Diffusion
RD_DB = 0.5 #DB in Reactions and Diffusion
RD_F = 0.055 #F in Reactions and Diffusion
RD_K = 0.062 # K in Reactions and Diffusion

SteerWithPheromoneScale = 2 #0~5 scale of rotation if ant senses pheromone in certain direction
SteerWithFoodScale = 0.7 #0~1 scale of rotation towards the food path

AntPheromoneReleaseRate = 0.5 #0~1 the amount of pheromone / "Black" released by ants
FoodPheromoneReleaseRate = 0.5 #0~1 the amount of pheromone / "Black" released by food

foodProximity = 3 #if ant is within this distance of food, it will consider arrived.
loopCount = 2 #number of loops that ants go through the path




#Main run code

FoodLocation = []
antLocation = []

if Reset: 
    #Initialize the system with the number of ants and food, size of the grid and resolution of the grid
    antSystem = AgentSystem(antNum, FoodNum, size, resolution)

else: 
    startTime = time.time() #Measure the time it takes to run the code

    #Visualize --------------------
    for ant in antSystem.Agents:
        antLocation.append(ant.Position)
    
    for f in antSystem.Foods.Position:
        FoodLocation.append(f)
    
    a = antLocation
    b = FoodLocation
    c = antSystem.Grid.Drawmesh()
    d = antSystem.Grid.PlotValue()
    #------------------------------

    antSystem.Update() #when timer is running, update the system

    print("--- %s seconds ---" % ((time.time() - startTime))) #Print the time it takes to run the code

