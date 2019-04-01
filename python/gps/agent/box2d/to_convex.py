import Box2D as b2 
import numpy as np

MAX_VALUE = 1.79E+308
THRESH = 5

"""
* Separates a non-convex polygon into convex polygons and adds them as fixtures to the <code>body</code> parameter.<br/>
* There are some rules you should follow (otherwise you might get unexpected results) :
* <ul>
* <li>This class is specifically for non-convex polygons. If you want to create a convex polygon, you don't need to use this class - Box2D's <code>b2PolygonShape</code> class allows you to create convex shapes with the <code>setAsArray()</code>/<code>setAsVector()</code> method.</li>
* <li>The vertices must be in clockwise order.</li>
* <li>No three neighbouring points should lie on the same line segment.</li>
* <li>There must be no overlapping segments and no "holes".</li>
* </ul> <p/>
* @param body The b2Body, in which the new fixtures will be stored.
* @param fixtureDef A b2FixtureDef, containing all the properties (friction, density, etc.) which the new fixtures will inherit.
* @param verticesVec The vertices of the non-convex polygon, in clockwise order.
* @param scale <code>[optional]</code> The scale which you use to draw shapes in Box2D. The bigger the scale, the better the precision. The default value is 30. 
"""
def separate(verticesVec):
    # create an img
    vec = np.asarray(verticesVec)
    img = np.zeros((500, 500, 3))
    convex_polygon_list = []
    vec = vec[::-1]  # reverse the order, converting CCW to
    figsVec = calcShapes(vec)
    for i in range(len(vec)):
        verticesVec = []  # list of type b2Vec2
        vec = figsVec[i]
        for j in range(len(vec)):
            verticesVec.append(b2.b2Vec2(vec[j][0]/scale, vec[j][1]/scale))
        


def Validate(verticesVec):
    n = len(verticesVec)
    ret = 0 
    fl2 = False
    for i in range(n):
        i2 = (i+1) if i<(n-1) else 0
        i3 = (i-1) if i>0 else (n-1)
        fl = False
        for j in range(n):
            if (j!=i) and (j!=i2):
                if not fl:
                    d = det(verticesVec[i][0], verticesVec[i][1],verticesVec[i2][0], verticesVec[i2][1], verticesVec[j][0], verticesVec[j][1])
                    print("d", i, i2, j, d)
                    if d>0:
                        # not convex
                        fl = True
                if j!=i3:
                    j2 = (j+1) if j<(n-1) else 0
                    if hitSegment(verticesVec[i][0], verticesVec[i][1],verticesVec[i2][0],verticesVec[i2][1],verticesVec[j][0], verticesVec[j][1],verticesVec[j2][0],verticesVec[j2][1]):
                        ret = 1
        print("fl", fl, "ret", ret)
        if not fl:
            fl2 = True
    
    if fl2:
        if ret == 1:
            ret = 3
        else:
            ret = 2
    
    return ret

def calcShapes(verticesVec):
    queue = []
    queue.append(verticesVec)
    while (len(queue)):
        vec = queue[0]
        n = len(vec)
        isConvex = True

        for i in range(n):
            i1 = i
            i2 = (i+1) if i<(n-1) else i+1-n
            i3 = (i+2) if i<(n-2) else i+2-n 

            p1 = vec[i1]
            p2 = vec[i2]
            p3 = vec[i3]
            d = det(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])
            print("d",p1, p2, p3, d)
            if d<0:
                # get a reflex point
                isConvex = False
                minLen = MAX_VALUE

                for j in range(n):
                    # find the nearest opposite edge with endpoints h, k
                    if (j!=i1) and (j!=i2):
                        j1 = j
                        j2 = j+1 if j<(n-1) else 0

                        v1 = vec[j1]
                        v2 = vec[j2]
                        v = hitRay(p1[0], p1[1], p2[0], p2[1], v1[0], v1[1], v2[0], v2[1])
                        print("hitRay", p1, p2, v1, v2, v)
                        if v:
                            dx = p2[0] - v[0]
                            dy = p2[1] - v[1]
                            t = dx**2 + dy**2
                            
                            if t<minLen:
                                h = j1
                                k = j2
                                hitV = v
                                minLen = t
                if minLen==MAX_VALUE:
                    print ("Error: minLen==MAX_VALUE, run Validate now")
                    valid_code = Validate(verticesVec)
                    print("error", valid_code)
                vec1 = []
                vec2 = []
                # i, j are indexes, v, vec[] are the points
                j1 = h
                j2 = k
                v1 = vec[j1]
                v2 = vec[j2]
                # if the intersection point (hitV) is not one of the endpoints of the edge
                if not pointsMatch(hitV[0], hitV[1], v2[0], v2[1]):
                    vec1.append(hitV)
                if not pointsMatch(hitV[0], hitV[1], v1[0], v1[1]):
                    vec2.append(hitV)
                
                h = -1
                k = i1
                while(True):
                    if k!=j2:
                        vec1.append(vec[k])
                    else:
                        if h<0 or h>=n:
                            raise ValueError("check the conditions in while(True)")
                        # TODO: check the original code
                        if not isOnSegment(v2[0], v2[1], vec[h][0], vec[h][1], p1[x], p1[y]):
                            vec1.append(vec[k])
                        break

                    h = k
                    print("update h for vec1", h)
                    if (k-1)< 0:
                        k = n-1
                    else:
                        k=k-1
                vec1 = vec1.reverse()
                print("vec1", vec1)
                h = -1
                k = i2
                while(True):
                    if k!=j1:
                        vec2.append(vec[k])
                    else:
                        if h<0 or h>=n:
                            raise ValueError
                        # TODO: check the original code
                        if k==j1 and not isOnSegment(v1[0], v1[1], vec[h][0], vec[h][1], p2[x], p2[y]):
                            vec2.append(vec[k])
                        break

                    h = k
                    print("update h for vec2", h)
                    if (k+1)>n-1:
                        k = 0
                    else:
                        k=k+1
                print("vec2", vec2)
                queue.append(vec1,vec2)
                queue.pop(0)  # remove the first element

                break
        if isConvex:
            figsVec.append(queue[0])
            queue.pop(0)
    return figsVec

def hitRay(x1, y1, x2, y2, x3, y3, x4, y4):
    """
    understanding:
    check whether pt2 will be hit by the rays that are radiated from pt1
        and that go to all the other edges of the polygon (connecting between pt3 & pt4)
    If exist, pt2 is a reflex point
    """
    # calculate the px, py as the coordinates of the potential intersection
    # of line 1-2, 3-4
    t1 = x3-x1
    t2 = y3-y1
    t3 = x2-x1
    t4 = y2-y1 
    t5 = x4-x3
    t6 = y4-y3
    t7 = t4*t5-t3*t6
    
    # # original method
    # a = (t5*t2-t6*t1)/t7
    # px = x1+a*t3
    # py = y1+a*t4

    # my method
    t8 = x1*y2-x2*y1
    t9 = x3*y4-x4*y3
    px = (t5*t8-t3*t9)/t7
    py = (t6*t8-t4*t9)/t7
    print("px, py", px, py)
    
    # use isOnSegment to take ray and line segment into consideration
    b1 = isOnSegment(x2, y2, x1, y1, px, py)
    b2 = isOnSegment(px, py, x3, y3, x4, y4)

    print("b1, b2", b1, b2)
    if (b1 and b2):
        # return b2.b2Vec2(px, py)
        return (px, py)
    return None

def hitSegment(x1, y1, x2, y2, x3, y3, x4, y4):
    """
    check if pt1, pt2, pt3, pt4 form overlapping lines
    b1: if pt is collinear with pt1, pt2
    b2: if pt is collinear with pt3, pt4
    if pt is collinear with (pt1, pt2) (pt3, pt4) at the same time, 
        pt1, 2, 3, 4 are overlapping lines
    """ 
    t1 = x3-x1
    t2 = y3-y1
    t3 = x2-x1
    t4 = y2-y1 
    t5 = x4-x3
    t6 = y4-y3
    t7 = t4*t5-t3*t6

    a = (t5*t2-t6*t1)/t7
    px = x1+a*t3
    py = y1+a*t4


    b1 = isOnSegment(px, py, x1, y1, x2, y2)
    b2 = isOnSegment(px, py, x3, y3, x4, y4)

    if (b1 and b2):
        return b2.b2Vec2(px, py)
    
    return None

def isOnSegment(px, py, x1, y1, x2, y2):
    b1 = (((x1+THRESH)>=px) and (px>=(x2-THRESH))) or (((x1-THRESH)<=px) and (px<=(x2+THRESH)))
    b2 = (((y1+THRESH)>=py) and (py>=(y2-THRESH))) or (((y1-THRESH)<=py) and (py<=(y2+THRESH)))
    b = isOnLine(px, py, x1, y1, x2, y2)
    print("isOnSegment", b1, b2, b)
    return ((b1 and b2) and b)

def pointsMatch(x1, y1, x2, y2):
    """
    Test if two points are coincident
    """
    dx = abs(x2-x1)
    dy = abs(y2-y1)
    return ((dx<THRESH) and (dy<THRESH))

def isOnLine(px, py, x1, y1, x2, y2):
    if abs(x2-x1)>THRESH:
        a = (y2-y1)/(x2-x1)
        possibleY = a*(px-x1)+y1
        diff = abs(possibleY-py)
        # print(diff)
        return (diff<THRESH)

    return abs(x1-px)<THRESH

def det(x1, y1, x2, y2, x3, y3):
    # det == 0, the three points are collinear
    # ref: https://www.quora.com/If-three-points-x1-y1-x2-y2-x3-y3-satisfy-y1-x3-x2-y2-x1-x3-y3-x2-x1-0-then-they-are-collinear-Why-is-this-true
    return x1*y2+x2*y3+x3*y1-y1*x2-y2*x3-y3*x1

def convert_form(verticesVec, map_height, scale):
    new_verticesVec = []
    for item in verticesVec:
        new_item = (item[0]*scale, (map_height- item[1])*scale)
        new_verticesVec.append(new_item)
    return new_verticesVec    


if __name__=='__main__':
    poly = np.asarray([
        [123,   1],
        [195, 138],
        [286, 123],
        [319, 129],
        [448, 258],
        [460, 285],
        [460,   1]
        ])
    separate(poly)
    # print(convex_polygon_list)
    cv2.imshow('img', img)
    cv2.waitKey(0)