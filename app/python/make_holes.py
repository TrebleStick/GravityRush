from math import cos, sin, pi
from PIL import Image, ImageDraw

#define basic colours
black = (0,0,0)
white=(255,255,255)
trans = (0,0,0,0)
#define dimesions
img_sizeX = 500
img_sizeY = 500
img_dimesions = (img_sizeX,img_sizeY)

# code from https://codereview.stackexchange.com/questions/144073/drawing-an-archimedean-spiral-using-pillow
def translate(point, screen_size):
    return point[0] + screen_size / 2, point[1] + screen_size / 2


def draw_spiral(a, b, img, step=0.1, loops=5):
    """
    Draw the Archimdean spiral defined by:
    r = a + b*theta
    Args:
        a (real): First parameter
        b (real): Second parameter
        img (Image): Image to write spiral to.
        step (real): How much theta should increment by. (default: 0.5)
        loops (int): How many times theta should loop around. (default: 5)
    """
    draw = ImageDraw.Draw(img)
    theta = 0.0
    r = a
    prev_x = int(r*cos(theta))
    prev_y = int(r*sin(theta))
    while theta < 2 * loops * pi:
        theta += step
        r = a + b*theta
        # Draw pixels, but remember to convert to Cartesian:
        x = int(r*cos(theta))
        y = int(r*sin(theta))
        draw.line(translate((prev_x, prev_y), img.size[0]) +
                  translate((x, y), img.size[0]), width=2, fill=white)
        prev_x = x
        prev_y = y


for i in range(0,10):
    img = Image.new('RGBA', img_dimesions, trans)
    draw_spiral(1, i+1, img, loops=11-i)
    filename = "holes/hole_" + str(i) + ".png"
    img.save(filename, format='PNG')
