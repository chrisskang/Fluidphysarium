import Rhino.Geometry as rg
import random
import math
import ghpythonlib.components as comps


class AgentSystem:

    def __init__(self, count):
        self.Agents = []
        self.Pheromones = []

        for i in range(0,count):
            randPos = rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0)
            
            agent = Agent(randPos)
            agent.System = self
            self.Agents.append(agent)

        
        
    def Update(self,targetPos):

        for agent in self.Agents:
            agent.Update(targetPos)
            
            for pheromone in self.Pheromones:
                pheromone.UpdatePheromone(agent.History)


class Pheromone:

    def __init__(self, position,intensity):
        self.Position = position 
        self.Intensity = intensity
        self.DecayRate = 0.999
        self.System = None
        


    def UpdatePheromone(self,curve):

        self.Path = curve
        
        self.Intensity=self.Intensity*self.DecayRate



class Agent:

    def __init__(self,initialPosition):
        
        self.Position = initialPosition

        self.Velocity = rg.Vector3d(0,0,0)
        self.History = [self.Position]
        self.System = None

    
    def findAllDistance(self,targetPos):
        Desireability = []
        line = targetPos - self.Position
        distance = line.Length
        
        if distance !=0:
            for pheromone in self.System.Pheromones: 
            
                if pheromone.Position ==targetPos:
                    Desireability.append(math.pow((pheromone.Intensity),1))*(math.pow((1 /distance),1))
                
                else: 
                    Desireability.append(math.pow((1 / distance),1))

        return Desireability



    def FindNext(self, targetPose):

        dict = {}

        for count, pose in enumerate(targetPose.Position):
            dict[count] = self.findAllDistance(pose)

        sorted_dict = sorted(dict.items(), key=lambda x:x[1], reverse=True)

        n = next(iter(sorted_dict))[0]

        final = targetPose.Position[n]
        
        return final



    def Goto(self, targetList):

        targetPoint = self.FindNext(targetList) #closestpoint
        random_vector = rg.Vector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        orientedVector = targetPoint - self.Position
        
        self.Velocity += self.Velocity*0.037+orientedVector*0.5+random_vector*0.13
        self.Velocity.Unitize()
        self.Velocity *= 0.1

        
        if orientedVector.Length < 0.1 and targetPoint in targetList.Position:
            
            targetList.Position.remove(targetPoint)
            
            pheromoneHistory = []
            pheromoneHistory = self.History
            
            pheromone = Pheromone(self.Position,1)
            pheromonePath=[]
            pheromonePath = rg.NurbsCurve.CreateInterpolatedCurve(pheromoneHistory,3)
            
            
            pheromone.UpdatePheromone(pheromonePath)
            
            self.System.Pheromones.append(pheromone)
            


    def Update(self, targetList):
        
        self.Goto(targetList)

        
        self.Position += self.Velocity

        self.History.append(self.Position)
        




class Target:

    def __init__(self, count):
        self.Position = []
        for i in range(0,count):
            self.Position.append(rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0))





TargetNum = 30
antNum = 2

#main code

if Reset: #initialize

    targets = Target(TargetNum)

    antSystem = AgentSystem(antNum)
    
    

    
else:

    antSystem.Update(targets)




#visualization

tResult = []
aResult = []
cResult = []
dResult = []
eResult = []
fResult = []
gResult = []

for t in targets.Position:
    
    tResult.append(t)

for ant in antSystem.Agents:
    aResult.append(ant.Position)


for ant in antSystem.Agents:
    cResult.append(ant.History)

    
    
for pheromone in antSystem.Pheromones:
    dResult.append(pheromone.Path)
    eResult.append(pheromone.Intensity)
    fResult.append(pheromone.Position)



a = aResult
b = tResult

c= rg.NurbsCurve.CreateInterpolatedCurve(cResult[0],3)

d= rg.NurbsCurve.CreateInterpolatedCurve(cResult[1],3)


e=dResult
f=eResult
g=fResult

