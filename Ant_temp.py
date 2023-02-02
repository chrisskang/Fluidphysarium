import Rhino.Geometry as rg
import random
import math

class AgentSystem:

    def __init__(self, count):
        self.Agents = []

        for i in range(0,count):
            randPos = rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0)
            agent = Agent(randPos)
            agent.System = self
            self.Agents.append(agent)
        
    def Update(self, targetPos):
        for agent in self.Agents:
            agent.Update(targetPos)






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

        if distance != 0:
            Desireability.append(math.pow((1 / distance),2))
        return Desireability
        


    def FindNext(self, targetPose):

        dict = {}

        for count, pose in enumerate(targetPose.Position):

            dict[count] = self.findAllDistance(pose)

        sorted_dict = sorted(dict.items(), key=lambda x:x[1], reverse=True)

        n = next(iter(sorted_dict))[0]

        final = targetPose.Position[n]
        
        return final #return closest point

    #def FindTrail(self):

    def Goto(self, targetList):

        targetPoint = self.FindNext(targetList) #closestpoint

        orientedVector = targetPoint - self.Position
        self.Velocity += orientedVector
        self.Velocity.Unitize()
        self.Velocity *= 0.1
        
        
        if orientedVector.Length < 0.1:

            targetList.Position.remove(targetPoint)

        
    
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

for t in targets.Position:
    
    tResult.append(t)

for ant in antSystem.Agents:
    aResult.append(ant.Position)

a = aResult
b = tResult