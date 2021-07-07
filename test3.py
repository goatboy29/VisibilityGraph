from VisGraph import *
import matplotlib.pyplot as plt
import random

G = Graph();

#Import Objects
f = open('points.txt','r');
for line in f:
    if "#Object" in line:
        pts = [];
        nodes = [];
        subline = f.readline();
        while "#END" not in subline:
            nums = subline.split()
            pts.append((float(nums[0]),float(nums[1])))
            subline = f.readline()

        if pts[0] == pts[-1]:
            shape = Polygon(pts);
        else:
            shape = LineString(pts);

        big_shape = shape.buffer(0.1,2);

        x,y = big_shape.exterior.xy;
        big_pts = [];
        for i in range(len(x)-1):
            big_pts.append((x[i],y[i]));

        for pt in big_pts:
            nodes.append(G.create_node(pt[0],pt[1]));

        G.create_obstacle(nodes);

f.close()

n1 = G.create_node(0.5,2.5)
n2 = G.create_node(2.3,1.0)
n3 = G.create_node(2.0,2.0)

G.build_visibility();

path = G.aStar(n1,n2)
print(path)

G.plot_graph(path);

'''
for i in range(5):
    a = G.nodes[random.randint(0,len(G.nodes)-1)]
    b = a;
    while b is a:
        b = G.nodes[random.randint(0,len(G.nodes)-1)]

    print(a.get_xy(),b.get_xy())
    path = G.aStar(a,b)

    G.plot_graph(path);
'''
