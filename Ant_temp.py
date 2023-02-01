import Rhino.Geometry as rg
import random
import math

class AgentSystem:

    def __init__(self, count):
        self.Agents = []

        for i in range(0,count):
            randPos = xyz
            agent = Agent(xyz)
            agent.System = self
            self.Agents.append(agent)
        
    def Update(self):
        for agent in self.Agents:
            agent.Update()






class Agent:

    def __init__(self,initialPosition):
        
        self.Position = initialPosition

        self.Velocity = rg.Vector3d(0,0,0)
        self.History = [self.Position]

    
    def ComputeDesirable(self,targetPos):
        Desireability = []
        line = targetPos - self.Position
        distance = line.Length
        if distance != 0:
            Desireability.append(math.pow((1 / distance),2))
        return Desireability
        
    def FindNext(self, targetPose):

        dict = {}
        for count, pose in enumerate(targetPose.Position):

            dict[count] = self.ComputeDesirable(pose)

        sorted_dict = sorted(dict.items(), key=lambda x:x[1], reverse=True)

        n = next(iter(sorted_dict))[0]

        final = targetPose.Position[n]
        
        return final #return sorted list



    def Goto(self, targetList):

        targetPoint = self.FindNext(targetList)

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
        


#main code

if Reset:

    myTarget = Target(30)
    randpoint = myTarget.Position[int(random.uniform(0,9))]
    myAgent = Agent(randpoint)
    myTarget.Position.Remove(randpoint)

    #myAgentSystem = AgentSystem(10)




    
else:
    myAgent.Update(myTarget)



#visualization
#paths = []
targets = []

paths = []

paths.append(rg.PolylineCurve(myAgent.History))

for t in myTarget.Position:
    targets.append(t)

a = paths
b = targets
c = myAgent.Position