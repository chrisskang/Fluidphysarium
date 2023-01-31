import Rhino.Geometry as rg
import random
import math


class Agent:

    def __init__(self,initialPosition):
        
        self.Position = initialPosition

        self.Velocity = rg.Vector3d(0,0,0)

    
    def ComputeDesirable(self,targetPose):
        Desireability = []
        for pos in targetPose.Position:
            line = pos - self.Position
            distance = line.Length
            Desireability.append(math.pow((1 / distance),2))
        return Desireability
        
    def FindNext(self, targetPose):
        self.ComputeDesirable(targetPose)

        

    
    def Update(self, Targets):
        self.FindNext(Targets)
        
        self.Position += self.Velocity



class Target:
    def __init__(self, count):
        self.Position = []
        for i in range(0,count):
            self.Position.append(rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0))
        


#main code

if Reset or not("AgentSystem" in globals()):
    myAgent = Agent(rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0))
    myTarget = Target(10)
    
else:
    myAgent.Update(myTarget)


#visualization
#paths = []
targets = []

paths = myAgent.Position

for t in myTarget.Position:
    targets.append(t)

a = paths
b = targets