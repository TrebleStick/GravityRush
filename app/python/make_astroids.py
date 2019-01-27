import math, random
from PIL import Image, ImageDraw
#define basic colours
black = (0,0,0)
white=(255,255,255)
trans = (0,0,0,0)
#define dimesions
img_sizeX = 500
img_sizeY = 500
img_dimesions = (img_sizeX,img_sizeY)

# Code from https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon
def generatePolygon( ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts ) :
# #Start with the centre of the polygon at ctrX, ctrY,
#     then creates the polygon by sampling points on a circle around the centre.
#     Randon noise is added by varying the angular spacing between sequential points,
#     and by varying the radial distance of each point from the centre.
#
#     Params:
#     ctrX, ctrY - coordinates of the "centre" of the polygon
#     aveRadius - in px, the average radius of this polygon, this roughly controls how large the polygon is, really only useful for order of magnitude.
#     irregularity - [0,1] indicating how much variance there is in the angular spacing of vertices. [0,1] will map to [0, 2pi/numberOfVerts]
#     spikeyness - [0,1] indicating how much variance there is in each vertex from the circle of radius aveRadius. [0,1] will map to [0, aveRadius]
#     numVerts - self-explanatory
#
#     Returns a list of vertices, in CCW order.

    irregularity = clip( irregularity, 0,1 ) * 2*math.pi / numVerts
    spikeyness = clip( spikeyness, 0,1 ) * aveRadius

    # generate n angle steps
    angleSteps = []
    lower = (2*math.pi / numVerts) - irregularity
    upper = (2*math.pi / numVerts) + irregularity
    sum = 0
    for i in range(numVerts) :
        tmp = random.uniform(lower, upper)
        angleSteps.append( tmp )
        sum = sum + tmp

    # normalize the steps so that point 0 and point n+1 are the same
    k = sum / (2*math.pi)
    for i in range(numVerts) :
        angleSteps[i] = angleSteps[i] / k

    # now generate the points
    points = []
    angle = random.uniform(0, 2*math.pi)
    for i in range(numVerts) :
        r_i = clip( random.gauss(aveRadius, spikeyness), 0, 2*aveRadius )
        x = ctrX + r_i*math.cos(angle)
        y = ctrY + r_i*math.sin(angle)
        points.append( (int(x),int(y)) )

        angle = angle + angleSteps[i]

    return points

def clip(x, min, max) :
    if( min > max ) :  return x
    elif( x < min ) :  return min
    elif( x > max ) :  return max
    else :             return x



for i in range(0,10):

    verts = generatePolygon( ctrX=img_sizeX/2, ctrY=img_sizeY/2, aveRadius=100, irregularity=0.5, spikeyness=0.2, numVerts=12 )

    tupVerts = list(map(tuple,verts))
    im = Image.new('RGBA', img_dimesions, trans)
    imPxAccess = im.load()
    draw = ImageDraw.Draw(im)

    draw.line( tupVerts+[tupVerts[0]], width=2, fill=white )

    # im.show() #uncomment to show during generation
    filename = "astroids/astroid_" + str(i) + ".png"
    im.save(filename, format='PNG')

# now you can save the image (im), or do whatever else you want with it.
