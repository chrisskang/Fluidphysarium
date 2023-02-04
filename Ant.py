import Rhino.Geometry as rg
import random
import math
import copy


class AgentSystem:

    def __init__(self, count):
        self.Agents = []

        for i in range(0,count):
            randPos = rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0)
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


        if self.Position.DistanceTo(self.Target) < 0.01:
            self.Counter += 1

            self.Target = self.Path[self.Counter]
            #print("switched to target",  self.Counter)
        
        #if counter is at the end of the path, reset counter and target

        if self.Counter == len(self.Path)-1:
            self.Counter = 0
            self.Target = self.Path[self.Counter]
        #print(len(self.Path))

        orientedVector = self.Target - self.Position

        orientedVector = orientedVector / 10
        #dividedVector = orientedVector / 10

        self.Velocity = orientedVector
        self.Position += self.Velocity



class Target:

    def __init__(self, count):
        self.Position = []

        for i in range(0,count):
            self.Position.append(rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0))
        
class Grid:
    #grid plots value in a list of lists
    def __init__(self, x, y, initValue):
        self.x = x 
        self.y = y

        self.grid = []
        for i in range(0,x):
            self.grid.append([initValue])
            for j in range(0,y):
                self.grid[i].append(initValue)

    def Plot(self):
        points = []
        for i in range(0,self.x):
            for j in range(0,self.y):
                points.append(rg.Point3d(i,j,0))








TargetNum = 30
antNum = 100
resolution = 1
#main code

if Reset: #initialize

    targets = Target(TargetNum)

    antSystem = AgentSystem(antNum)

    antSystem.FindPath(targets)

    #grid = Grid(10/resolution,10/resolution,0)
    

else:
    antSystem.Update(targets)

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
c = pathResult
