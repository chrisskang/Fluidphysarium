import Rhino.Geometry as rg
import random


class AgentSystem:

    def __init__(self,agentCount):
        self.Agents= []
        for i in range(0,agentCount):
            randomInitialPosition = rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0)
            agent = Agent(randomInitialPosition)
            agent.System = self
            self.Agents.append(agent)

    def Update(self):

        for agent in self.Agents:
            agent.Update()

class Agent:

    def __init__(self,initialPosition):
        
        self.Position = initialPosition

        self.Velocity = rg.Vector3d(0,0,0)

    
    def ComputeDesirable(self):
        self. System
        distance = myTarget.Position - self.Position
    
    def Update(self):
        self.ComputeDesireable()

        self.Position += self.Velocity



class Target:
    def __init__(self, count):
        self.Position = []
        for i in range(0,count):
            self.Position.append(rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0.0))
        



#main code

if Reset or not("AgentSystem" in globals()):
    myAgentSystem = AgentSystem(100)
    myTarget = Target(10)
    
else:
    myAgentSystem.Update()


#visualization
paths = []
targets = []

for p in myAgentSystem.Agents:
    paths.append(p.Position)

for t in myTarget.Position:
    targets.append(t)

a = paths
b = targets