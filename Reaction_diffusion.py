import Rhino.Geometry as rg
import System.Drawing as drawing
import random



size = 100
resolution = 1
num = int(size/resolution)

dA = 2
dB = 1
f = 0.055
k = 0.062

def initialize():
    states = []
    for x in range(num):
        states.append([])

        for y in range(num):
            states[x].append({"a": 1, "b": 0})
            if x > 40 and x < 45 and y > 40 and y < 45:

                states[x][y] = ({"a": 0, "b": 1})
            elif x > 50 and x < 55 and y > 50 and y < 55:
                states[x][y] = ({"a": 0, "b": 1})
 
    return states 

def update(states):
    for x in range(num-1):
        for y in range(num-1):
            if x > 0 and y > 0:
                acopy = states[x][y]["a"]
                
                bcopy = states[x][y]["b"]
                
                states[x][y]["a"] = acopy + (dA * laplaceA(states,x,y)) - (acopy * bcopy* bcopy) + (f * (1 - acopy))
                states[x][y]["b"] = bcopy + (dB * laplaceB(states,x,y)) + (acopy * bcopy* bcopy) - ((k + f)* bcopy)
    return states
    
def laplaceA(states,x,y):
    sumA = 0
    sumA += states[x][y]["a"] * -1   
    sumA += states[x-1][y]["a"] * 0.2
    sumA += states[x+1][y]["a"] * 0.2
    sumA += states[x][y+1]["a"] * 0.2
    sumA += states[x][y-1]["a"] * 0.2

    sumA += states[x-1][y-1]["a"] * 0.05
    sumA += states[x+1][y-1]["a"] * 0.05
    sumA += states[x+1][y+1]["a"] * 0.05
    sumA += states[x-1][y+1]["a"] * 0.05
    
    return sumA



def laplaceB(states,x,y):
    sumB = 0
    sumB += states[x][y]["b"] * -1
    
    sumB += states[x-1][y]["b"] * 0.2
    sumB += states[x+1][y]["b"] * 0.2
    sumB += states[x][y+1]["b"] * 0.2
    sumB += states[x][y-1]["b"] * 0.2

    sumB += states[x-1][y-1]["b"] * 0.05
    sumB += states[x+1][y-1]["b"] * 0.05
    sumB += states[x+1][y+1]["b"] * 0.05
    sumB += states[x-1][y+1]["b"] * 0.05
    
    return sumB




#Main Script


def drawmesh():
    
    displayMesh = rg.Mesh()

    colors = []

    for x in range(num):
        for y in range(num):
            displayMesh.Vertices.Add(rg.Point3d(x,y,0))
            displayMesh.Vertices.Add(rg.Point3d(x+1,y,0))
            displayMesh.Vertices.Add(rg.Point3d(x+1,y+1,0))
            displayMesh.Vertices.Add(rg.Point3d(x,y+1,0))
            v = displayMesh.Vertices.Count
            displayMesh.Faces.AddFace(v-4,v-3,v-2,v-1)
            
            value = abs(states[x][y]["a"]-states[x][y]["b"])* 255
            if value > 255:
                value = 255
            elif value < 0:
                value = 0
            #print(value)
            colors.append(drawing.Color.FromArgb(0, value,value,value))
            
    for i in range(len(displayMesh.Faces)):

        face = displayMesh.Faces[i]
        color = colors[i]
        displayMesh.VertexColors.SetColor(face,color)

    return displayMesh




if Reset:
    states = initialize()

else:
    update(states)
    ddd = drawmesh()
    




