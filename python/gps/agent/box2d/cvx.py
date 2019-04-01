import cv2
import numpy as np 

def to_convex(poly):
    # create an img
    poly = np.asarray(poly)
    img = np.zeros((500, 500, 3))
    convex_polygon_list = []
    while True:
        # print("poly", poly)
        if_convex_pt, img = is_convex(poly, img)
        # print(if_convex_pt)
        if all(item == True for item in if_convex_pt):
            # return poly
            convex_polygon_list.append(poly)
            # print("finish decomposition")
            return convex_polygon_list, img
        else:
            start_idx = find_start_idx(if_convex_pt)
            # print("start_idx", start_idx)
            if start_idx == -2:
                print("non-convex but cannot find reflex point, must be sth wrong")
            if start_idx == -1:
                print("find reflex point but no appropriate starting point")
            else:
                poly, convex_polygon_list, img = cut_convex_part(poly, if_convex_pt, start_idx, convex_polygon_list, img)

    # return poly

# while(any(item == False for item in if_convex_pt)):

def is_convex(poly, img):
    cv2.drawContours(img, [poly], -1, (255, 0, 0), 3)
    # print(poly)
    hull = cv2.convexHull(poly).squeeze()
    # print("hull", hull)
    # cv2.drawContours(img, [hull], -1, (0, 255, 0), 3)
    if_convex_pt = []
    # make a list to record the concave/convex point
    n = len(poly)
    for i in range(n):
        if np.sum(np.isin(poly[i], hull)) == 2:
            if_convex_pt.append(True) 
        else:
            if_convex_pt.append(False)
    return if_convex_pt, img
    
def find_start_idx(if_convex_pt):
    start_idx = -2
    n = len(if_convex_pt)
    for i in range(n):
        if not if_convex_pt[i]:
            # print(if_convex_pt)
            start_idx = -1
            i2 = (i+1) if i<(n-1) else i+1-n
            i3 = (i+2) if i<(n-2) else i+2-n
            # have two consecutive convex_point neighbors
            if if_convex_pt[i2] and if_convex_pt[i3]:
                start_idx = i  # find the starting index
                break
    return start_idx

def cut_convex_part(poly, if_convex_pt, start_idx, convex_polygon_list, img):
    i = start_idx
    n = len(poly)
    # the next two consecutive points should be convex
    i2 = (i+1) if i<(n-1) else i+1-n
    i3 = (i+2) if i<(n-2) else i+2-n
    contour = np.asarray([poly[i], poly[i2], poly[i3]])
    convex_polygon_list.append(contour)
    # print("contour", contour)
    cv2.drawContours(img, [contour], -1, (0, 0, 255), 1)
    if_convex_pt.pop(i2)
    poly = np.delete(poly, i2, 0)
    return poly, convex_polygon_list, img


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
    convex_polygon_list, img = to_convex(poly)
    # print(convex_polygon_list)
    cv2.imshow('img', img)
    cv2.waitKey(0)
