import Rhino.Geometry as rg
import random
import math
import copy
import System.Drawing as drawing
import time


class AgentSystem:

    def __init__(self, AntCount, FoodCount, size, resolution):

        self.Agents = [] #list of agents
        self.Grid = PheromoneGrid(size, resolution, 1) #2D grid of pheromone data
        self.Foods = Food(FoodCount) #list of food sources located randomly
        self.size = size #internalize size of the system
        self.resolution = resolution #internalize the resolution of the grid

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
        self.Velocity.Unitize()
        self.Target = None #target position
        #History of the agent's position
        self.History = [self.Position]
        
        self.Path = []
        
        self.Counter = 0
        self.RunCounter = 0
    def FindClosestPoint(self, foods):
        counter = 0
        if counter == 0:
            currentPos = self.Position


        bestDistance = 400
        ds = []
        
        for food in foods.Position:
            line = food - currentPos
        
            distance = line.Length

            if distance < bestDistance:

                ds.append(distance)
                bestDistance = distance

                closestpoint = food
        
        currentPos = closestpoint

        #print(currentPos)
        self.Path.append(currentPos)

        foods.Position.remove(closestpoint)

        if len(foods.Position) > 0:
            self.FindClosestPoint(foods)

    def FindTrail(self, foods):
        foodscopy = copy.deepcopy(foods)
        self.FindClosestPoint(foodscopy)
    
    def SteerWithTrail(self, steerStrength):
        

        if self.Counter < len(self.Path)-1 and self.RunCounter < 1:
            if self.Counter == 0:
                self.Target = self.Path[self.Counter]
            


            if self.Position.DistanceTo(self.Target) < 5:
                self.Counter += 1

                self.Target = self.Path[self.Counter]
            orientedVector = self.Target - self.Position
            orientedVector.Unitize()
            self.Velocity += orientedVector * steerStrength

        else:
            self.Counter = 0
            self.Target = self.Path[self.Counter]
            self.RunCounter += 1

        


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


    def Sense(self, angle):
        
        if angle == 0:
            return self.Velocity
        
        
        else:
            self.Velocity.Rotate(angle, rg.Vector3d.ZAxis)
    
            vectorTilted = self.Velocity * 1

            self.Velocity.Rotate(-angle, rg.Vector3d.ZAxis)

            return vectorTilted

        
    #https://www.youtube.com/watch?v=X-iSQQgOd1A&t=4s&ab_channel=SebastianLague
    def SteerWithPheromone(self, scale):
        angle = math.pi/4

        pointForward = self.Position + self.Sense(0)
        pointRight = self.Position + self.Sense(angle)
        pointLeft = self.Position + self.Sense(-angle)
        #print(pointForward, pointRight, pointLeft)
        if pointForward.X > 0 and pointForward.Y > 0 and pointForward.X < self.Size and pointForward.Y < self.Size \
            and pointRight.X > 0 and pointRight.Y > 0 and pointRight.X < self.Size and pointRight.Y < self.Size \
            and pointLeft.X > 0 and pointLeft.Y > 0 and pointLeft.X < self.Size and pointLeft.Y < self.Size:

            weightForward = self.System.Grid.Grid[int(pointForward.X/self.System.resolution)][int(pointForward.Y/self.System.resolution)]["Black"]
            weightRight = self.System.Grid.Grid[int(pointRight.X/self.System.resolution)][int(pointRight.Y/self.System.resolution)]["Black"]
            weightLeft = self.System.Grid.Grid[int(pointLeft.X/self.System.resolution)][int(pointLeft.Y/self.System.resolution)]["Black"]

            if weightRight > weightForward and weightRight > weightLeft:
                self.Velocity = self.Velocity.Add(self.Velocity, self.Sense(angle) * scale)
                self.Velocity.Unitize()
                #print(self.Sense(angle) * scale)
            elif weightLeft > weightForward and weightLeft > weightRight:
                self.Velocity = self.Velocity.Add(self.Velocity, self.Sense(-angle) * scale)
                self.Velocity.Unitize()


    def Update(self, Food, grid):
        
        self.ReflectAtBoundary()
        self.SteerWithPheromone(SteerWithPheromoneScale)
        self.SteerWithTrail(SteerWithFoodScale)
        #print(self.Velocity)
        self.Velocity.Unitize()
        self.Position += self.Velocity



class Food:

    def __init__(self, Foodcount):
        self.Position = []

        for i in range(0,Foodcount):
            self.Position.append(rg.Point3d(random.uniform(0.0,size),random.uniform(0.0,size), 0))

class PheromoneGrid:
    #grid plots value in a list of lists
    def __init__(self, size, resolution, initValue):
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
        
        for ant in AgentSystem.Agents:

            self.Grid[int(ant.Position.X/self.Resolution)][int(ant.Position.Y/self.Resolution)]["Black"] += AntPheromoneReleaseRate

        for food in AgentSystem.Foods.Position:
            self.Grid[int(food.X/self.Resolution)][int(food.Y/self.Resolution)]["Black"] += FoodPheromoneReleaseRate

        self.Decay(decayRate)
        self.Diffuse()
        self.ReactionDiffusion(RD_DA, RD_DB, RD_F, RD_K)
        self.Swap()
  
    def Diffuse(self):
        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):

                    self.Next[x][y]["White"] = \
                    \
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
                    \
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
        if selector == "White":
            #print("selector is a")
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
            #print("selector is b")
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

            # sumB += self.Grid[x][y][z+1]["Black"] * 0.01
            # sumB += self.Grid[x][y][z-1]["Black"] * 0.01

            # sumB += self.Grid[x-1][y-1][z-1]["Black"] * 0.01
            # sumB += self.Grid[x+1][y-1][z-1]["Black"] * 0.01
            # sumB += self.Grid[x+1][y+1][z-1]["Black"] * 0.01
            # sumB += self.Grid[x-1][y+1][z-1]["Black"] * 0.01

            # sumB += self.Grid[x-1][y-1][z+1]["Black"] * 0.01
            # sumB += self.Grid[x+1][y-1][z+1]["Black"] * 0.01
            # sumB += self.Grid[x+1][y+1][z+1]["Black"] * 0.01
            # sumB += self.Grid[x-1][y+1][z+1]["Black"] * 0.01



            return sumB

    #https://stackoverflow.com/questions/34837677/a-pythonic-way-to-write-a-constrain-function
    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))
  
    def Decay(self, Rate):
        for x in range(self.Num):
            for y in range(self.Num):
                #self.Grid[x][y]["White"] *= 0.95
                self.Grid[x][y]["Black"] *= Rate
    
    def Swap(self):
        temp = self.Grid
        self.Grid = self.Next
        self.Next = temp

    def Drawmesh(self):
        
        displayMesh = rg.Mesh()
        colors = []
        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):
                    #if self.Grid[x][y]["Black"] > 0.1:
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution-1,y*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution-1,y*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution-1,(y+1)*self.Resolution-1,0))
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution-1, (y+1)*self.Resolution-1,0))


                    v = displayMesh.Vertices.Count

                    # displayMesh.Faces.AddFace(v-8,v-7,v-6,v-5)
                    

                    # displayMesh.Faces.AddFace(v-8,v-7,v-3,v-4)
                    # displayMesh.Faces.AddFace(v-7,v-6,v-2,v-3)
                    # displayMesh.Faces.AddFace(v-6,v-5,v-1,v-2)
                    # displayMesh.Faces.AddFace(v-5,v-8,v-4,v-1)

                    displayMesh.Faces.AddFace(v-4,v-3,v-2,v-1)
                    #print(v)

                    value = (self.Grid[x][y]["White"]-self.Grid[x][y]["Black"])* 255

                    if value > 255:
                        value = 255
                    elif value < 0:
                        value = 0

                    colors.append(drawing.Color.FromArgb(0, value,value,value))
            
        for i in range(len(displayMesh.Faces)):
            #does not want to assign or overwrite the color of the face

            #loop through all the faces and assign the color without overwriting the previous color in each verticies
            
            face = displayMesh.Faces[i]

            color = colors[i]

            displayMesh.VertexColors.SetColor(face,color)


        return displayMesh
    
    def PlotPoint(self):
        points = []

        for x in range(self.Num):
            for y in range(self.Num):

                points.append(self.Grid[x][y]["Black"])
        return points

    def PlotSphere(self):
        points = []
        radius = 10
        for x in range(0,self.Num):
            for y in range(0,self.Num):
                x = x/10
                y = y/10
                lon = x / math.pi

                lat = math.atan(math.exp(-2 *math.pi*y))

                px = 20 * math.cos(lat) * math.cos(lon)
                py = 20 * math.cos(lat) * math.sin(lon)
                pz = 20 * math.sin(lat)
                
                points.append(rg.Point3d(px,py,pz))

        return points

#main code

FoodNum = 30
antNum = 300
size = 100
resolution = 1

decayRate = 0.98
RD_DA = 1
RD_DB = 0.5
RD_F = 0.055
RD_K = 0.062
SteerWithPheromoneScale = 5 #0~10
SteerWithFoodScale = 0.7 #0~1
AntPheromoneReleaseRate = 1 #0~1
FoodPheromoneReleaseRate = 1 #0~1


#Reset = False


FoodLocation = []
antLocation = []

if Reset: 
    #Initialize the system with the number of ants and food, size of the grid and resolution of the grid
    antSystem = AgentSystem(antNum, FoodNum, size, resolution)

    #antSystem.FindPath(targets)

    
    
    

else:

    startTime = time.time() #Measure the time it takes to run the code

    

    #Visualize
    for ant in antSystem.Agents:
        antLocation.append(ant.Position)
       
    
    for f in antSystem.Foods.Position:
        
        FoodLocation.append(f)
    
    

    a = antLocation
    b = FoodLocation
    c = antSystem.Grid.Drawmesh()
    
    d = antSystem.Grid.PlotPoint()

    antSystem.Update()

    #d = rg.NurbsSurface.CreateFromPoints(antSystem.Grid.PlotPoint(), antSystem.Grid.Num, antSystem.Grid.Num,3,3)
    print("--- %s seconds ---" % ((time.time() - startTime))) #Print the time it takes to run the code

