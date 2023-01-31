import Rhino.Geometry as rg
import random
import math

class AgentSystem:

    def __init__(self,agentCount):
        self.Agents= []
        for i in range(0,agentCount):
            randomInitialPosition = rg.Point3d(random.uniform(0.0,10.0),random.uniform(0.0,10.0), 0)
            agent = Agent(randomInitialPosition)
            agent.System = self
            self.Agents.append(agent)

    def Update(self):
        # for agent in self.Agents:
        #     agent.ComputeDesiredVelocity()
        for agent in self.Agents:
            agent.Update()

class Agent:

    def __init__(self,initialPosition):
        self.Position = initialPosition
        alpha = random.uniform(0,6.18)
        self.Velocity = 0.1* rg.Vector3d(math.cos(alpha),math.sin(alpha),0.0)
        self.History = [self.Position]
    
    def Seperation(self, thrdis):
        for other in self.System.Agents:
            if self == other: continue
            getAway = self.Position - other.Position
            distance = getAway.Length
            if (distance > thrdis): continue
            getAway.Unitize()
            getAway *= thrdis / (distance + 1 )
            self.Velocity += getAway

    def Align(self, perception):
        #align

        sumX = 0
        sumY = 0
        sumZ = 0
        counter = 0
        for other in self.System.Agents:
            if self == other: continue

            line = self.Position - other.Position
            distance = line.Length

            if (distance > perception): continue
            #add all vector value for agents within perception distance
            sumX += other.Velocity.X
            sumY += other.Velocity.Y
            sumZ += other.Velocity.Z
            counter += 1

        #print sumX, sumY, sumZ
        
        if counter != 0:
            AverageVector = rg.Vector3d(sumX/counter, sumY/counter, sumZ/counter)
            SteeringForce = AverageVector - self.Velocity
            SteeringForce.Unitize()
        else:
            SteeringForce = rg.Vector3d(0,0,0)

        self.Velocity += SteeringForce

    def Cohesion(self, perception):
        #align

        sumX = 0
        sumY = 0
        sumZ = 0
        counter = 0
        for other in self.System.Agents:
            if self == other: continue

            line = self.Position - other.Position
            distance = line.Length

            if (distance > perception): continue
            #add all vector value for agents within perception distance
            sumX += other.Position.X
            sumY += other.Position.Y
            sumZ += other.Position.Z
            counter += 1

        if counter != 0:
            AverageLocation = rg.Point3d(sumX/counter, sumY/counter, sumZ/counter)

            SteeringForce = AverageLocation - self.Position

            SteeringForce.Unitize()
        else:
            SteeringForce = rg.Vector3d(0,0,0)

        self.Velocity += SteeringForce

    def Boundary(self, size, dis):
        scale = 10
        if min((size - self.Position.X), self.Position.X) < dis:
            if self.Position.X > 5:
                self.Velocity += rg.Vector3d(0,scale,0)
            else:
                self.Velocity += rg.Vector3d(0,-scale,0)
            

        if min((size - self.Position.Y), self.Position.Y) < dis:
            if self.Position.Y > 5:
                self.Velocity += rg.Vector3d(-scale,0,0)
            else:
                self.Velocity += rg.Vector3d(scale,0,0)

        # if self.Position.X < 0.0:
        #     self.Position.X = 0
        #     self.Velocity.X = - self.Velocity.X
        # elif self.Position.X > size:
        #     self.Position.X = size
        #     self.Velocity.X = - self.Velocity.X

        # elif self.Position.Y < 0.0:
        #     self.Position.Y = 0
        #     self.Velocity.Y = - self.Velocity.Y
        # elif self.Position.Y > size:
        #     self.Position.Y = size
        #     self.Velocity.Y = - self.Velocity.Y
        # elif self.Position.Z < 0.0:
        #     self.Position.Z = 0
        #     self.Velocity.Z = - self.Velocity.Z
        # elif self.Position.Z > size:
        #     self.Position.Z = size
        #     self.Velocity.Z = - self.Velocity.Z

    def Attractor(self, attractor, attractorStrength):
        #Attractor
        toAttractor = attractor - self.Position
        toAttractor.Unitize()
        toAttractor *= attractorStrength
        self.Velocity += toAttractor


    def Update(self):
        if (self.Velocity.Length > 0.1):
            self.Velocity.Unitize()
            self.Velocity *= 0.1
        self.Position += self.Velocity
        self.Align(0.5)
        self.Cohesion(1)
        self.Boundary(10,0.5)
        self.Seperation(0.5)
        #print(self.Velocity *0.1)

        self.History.append(self.Position)
        if len(self.History) > 5:
            del self.History[0]
        
        #self.Attractor(rg.Point3d(5,5,0), 0.5)


if Reset or not("AgentSystem" in globals()):
    myAgentSystem = AgentSystem(50)

else:
    myAgentSystem.Update()


paths = []
for p in myAgentSystem.Agents:
    paths.append(rg.Point3d(p.Position))
    #paths.append(rg.PolylineCurve(p.History))
    #paths.append(rg.Circle(p.Position,0.5))

a = paths