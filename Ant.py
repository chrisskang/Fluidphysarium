import Rhino.Geometry as rg
import random
import math
import copy
import System.Drawing as drawing
import time


class AgentSystem:

    def __init__(self, count):
        self.Agents = []
        self.Grid = []

        for i in range(0,count):
            randPos = rg.Point3d(random.uniform(0.0,size),random.uniform(0.0,size), 0)
            agent = Agent(randPos)
            agent.System = self
            self.Agents.append(agent)
        
    def Update(self, targets):
        for agent in self.Agents:

            agent.Update(targets)
    
    def FindPath(self, targets):
        for agent in self.Agents:
            agent.FindTrail(targets)
    
    





#distances.append(math.pow((1 / distance),2))

class Agent:

    def __init__(self,initialPosition):
        
        self.Position = initialPosition

        self.Velocity = rg.Vector3d(0,0,0)
        self.History = [self.Position]
        self.System = None
        self.Path = []
        
        self.Counter = 0
        self.History.remove(self.History[0])


    def FindClosestPoint(self, targets):
        counter = 0
        if counter == 0:
            currentPos = self.Position


        bestDistance = 400
        ds = []
        
        for target in targets.Position:
            line = target - currentPos
        
            distance = line.Length

            if distance < bestDistance:

                ds.append(distance)
                bestDistance = distance

                closestpoint = target
        
        currentPos = closestpoint

        #print(currentPos)
        self.Path.append(currentPos)

        targets.Position.remove(closestpoint)

        if len(targets.Position) > 0:
            self.FindClosestPoint(targets)


    def FindTrail(self, targets):
        targetscopy = copy.deepcopy(targets)
        self.FindClosestPoint(targetscopy)
        #self.Path.insert(0, self.Position)

        #trail = rg.Polyline(self.Path)
        
    
    def Update(self, targets):

        if self.Counter > 0:
            
            self.History.append(self.Position)

        if self.Counter == 0:
            self.Target = self.Path[self.Counter]
            #print("target ", self.Counter)


        if self.Position.DistanceTo(self.Target) < 0.1:
            self.Counter += 1

            self.Target = self.Path[self.Counter]

            #print("switched to target",  self.Counter)
        
        #if counter is at the end of the path, reset counter and target

        if self.Counter == len(self.Path)-1:
            self.Counter = 0
            self.Target = self.Path[self.Counter]
        #print(len(self.Path))

        orientedVector = self.Target - self.Position
        
        orientedVector.Unitize()

        orientedVector*= 0.1
        #dividedVector = orientedVector / 10

        self.Velocity = orientedVector
        self.Position += self.Velocity



class Target:

    def __init__(self, count):
        self.Position = []

        for i in range(0,count):
            self.Position.append(rg.Point3d(random.uniform(0.0,size),random.uniform(0.0,size), 0))

# class Pheromone:

#     def __init__(self):
#         Pheromone.Value = 0.0

#     def Update(self, antSystem):
#         for ant in antSystem.Agents:
#             self.Grid[int(ant.Position.X)][int(ant.Position.Y)] += 1


class Grid:
    #grid plots value in a list of lists
    def __init__(self, size, resolution, initValue):
        self.Num = int(size/resolution)
        self.Num += 1
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
        
     
    def ReactionDiffusion(self, D_a, D_b, f, k):
        for x in range(self.Num):
            for y in range(self.Num):
                if x > 0 and y > 0 and  x < (self.Num -1) and y < (self.Num-1):

                        a = self.Grid[x][y]["White"]
                    
                        b = self.Grid[x][y]["Black"]

                        self.Next[x][y]["White"] = a + (D_a * self.laplace(x,y, "White")) - (a * b * b) + (f * (1 - a))
                        self.Next[x][y]["Black"] = b + (D_b * self.laplace(x,y, "Black")) + (a * b * b) - ((k + f)* b)

                        self.Next[x][y]["White"] = self.constrain(self.Next[x][y]["White"],0,1)

                        self.Next[x][y]["Black"] = self.constrain(self.Next[x][y]["Black"],0,1)


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

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def Update(self, AgentSystem):
        self.ReactionDiffusion(1,0.5,0.055,0.062)
        self.Swap()

        #self.Decay()
        
        
        for ant in AgentSystem.Agents:

            self.Grid[int(ant.Position.X/self.Resolution)][int(ant.Position.Y/self.Resolution)]["Black"] += 0.4

    

    def Decay(self):
        for x in range(self.Num):
            for y in range(self.Num):
                #self.Grid[x][y]["White"] *= 0.95
                self.Grid[x][y]["Black"] *= 0.95
    
        

    def Swap(self):
        temp = self.Grid
        self.Grid = self.Next
        self.Next = temp

    def Drawmesh(self):
        
        displayMesh = rg.Mesh()
        colors = []
        for x in range(self.Num):
            for y in range(self.Num):
                if self.Grid[x][y]["Black"] > 0.1:
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution,y*self.Resolution,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution,y*self.Resolution,0))
                    displayMesh.Vertices.Add(rg.Point3d((x+1)*self.Resolution,(y+1)*self.Resolution,0))
                    displayMesh.Vertices.Add(rg.Point3d(x*self.Resolution, (y+1)*self.Resolution,0))


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
    
        

TargetNum = 3
antNum = 10
size = 100
resolution = 1
counter = 0
#main code

if Reset: #initialize

    targets = Target(TargetNum)

    antSystem = AgentSystem(antNum)

    antSystem.FindPath(targets)

    myGrid = Grid(size, resolution, 1)
    
    

else:

    startTime = time.time()

    antSystem.Update(targets)

    #print(myGrid.Grid[0])

    myGrid.Update(antSystem)

    d = myGrid.Drawmesh()

    #d = myGrid.Plot()
    #e = myGrid.PlotValues()
    print("--- %s seconds ---" % ((time.time() - startTime)))
    #d = myGrid.Plot()




#visualization

targetResult = []
antResult = []
pathResult = []


for t in targets.Position:
    
    targetResult.append(t)

for ant in antSystem.Agents:
    antResult.append(ant.Position)
    pathResult.append(rg.PolylineCurve(ant.History))

a = antResult
b = targetResult
#c = pathResult

