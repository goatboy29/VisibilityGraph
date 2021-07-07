from math import dist
import matplotlib.pyplot as plt
from shapely.geometry import LineString,Polygon,Point

class Node:
    def __init__(self,x,y):
        self.x = x;
        self.y = y;
        self.parent = None;
        self.neighbors = [];
        self.open = True;
        self.pathDist2Start = None;
        self.straightDist2End = None;

    def resetVisibility(self):
        self.neighbors = [];

    def resetAncestory(self):
        self.parent = None;
        self.pathDist2Start = None;
        self.straightDist2End = None;

    def add_neighbor(self,neighbor_node):
        self.neighbors.append(neighbor_node);

    def get_xy(self):
        return (self.x,self.y)

    def dist_to_node(self,n2):
        x2,y2 = n2.get_xy();
        return dist([self.x,self.y],[x2,y2]);

    def dist_to_coords(self,x2,y2):
        return dist([self.x,self.y],[x2,y2]);

    def evaluate_neighbors(self):
        #print('Me:', self.get_xy(),self.pathDist2Start,self.parent.get_xy())
        for n in self.neighbors:
            if n is self.parent:
                continue
            dn = self.dist_to_node(n);
            d = self.pathDist2Start + dn;
            if n.pathDist2Start is not None:
                if (self.pathDist2Start + dn)<n.pathDist2Start:
                    n.parent = self;
                    n.pathDist2Start = d;
                elif(n.pathDist2Start+dn)<self.pathDist2Start:
                    self.parent = n;
                    self.pathDist2Start = n.pathDist2Start+dn;

            else:
                n.parent = self;
                n.pathDist2Start = d;

            #print(n.get_xy(),n.pathDist2Start,n.parent.get_xy())
        return;

    
class Obstacle:
    def __init__ (self,nodes):
        self.nodes = nodes;
        self.poly = None;
        self.line = None;

        self.shape = self.create_shape();

    def create_shape(self):
        pts = [];
        for n in self.nodes:
            pts.append(n.get_xy());

        pts.append(self.nodes[0].get_xy())#close the loop
        return Polygon(pts);

    def contains(self,node):
        if self.shape.contains(Point(*node.get_xy())):
            return True;
        else:
            return False;


    

class Graph:
    def __init__(self):
        self.nodes = [];
        self.obstacles = [];
    
    def create_node(self,x,y):
        n = Node(x,y);
        self.append_node(n);
        return n;

    def create_obstacle(self,nodes):
        o = Obstacle(nodes);
        self.append_obstacle(o);
        return o;

    def append_node(self,n):
        self.nodes.append(n);
    
    def append_obstacle(self,o):
        self.obstacles.append(o);

    def closest_node_to(self,x,y):

        closest_node = None;
        closest_dist = float('inf');

        for n in self.nodes:
            d = n.dist_to_coords(x,y);
            if (d < closest_dist):
                closest_node = n;
                closest_dist = d;

        return closest_node;

    def build_visibility(self):

        for n in self.nodes:
            if not n.open:
                continue
            for m in self.nodes:
                if not m.open:
                    continue
                if n is m:
                    continue

                if self.is_visible(n,m):
                    n.add_neighbor(m);
                    m.add_neighbor(n);

            n.open = False;

    def is_visible(self,n1,n2):

        x1,y1 = n1.get_xy();
        x2,y2 = n2.get_xy();
        ls = LineString([(x1,y1), (x2,y2)]);
        
        for o in self.obstacles:
            if o.contains(n1):
                n1.open = False; #node is contained within obstacle and is therefore not visible by anyone
                return False;
            elif o.contains(n2):
                n2.open = False;
                return False;
            if ls.crosses(o.shape):
                return False;
            if ls.within(o.shape):
                return False;

        return True;


    def plot_graph(self,path = None):        
        #Obstacles
        for o in self.obstacles:
            plt.plot(*o.shape.exterior.xy,c='m',ls='-');

        #Nodes
        for n in self.nodes:
            x,y = n.get_xy();
            plt.scatter(x,y,c='red');

            #visiblility lines
            for m in n.neighbors:
                x2,y2 = m.get_xy();
                plt.plot([x,x2],[y,y2],c='y',ls=':');

        #Ancestory
        for n in self.nodes:
            if n.parent is not None:
                x1,y1 = n.get_xy();
                x2,y2 = n.parent.get_xy();
                plt.plot([x1,x2],[y1,y2],c='b',ls='-');
                if n.parent is n:
                    plt.scatter(x1,y1,100,c='m')

        #path
        if path is not None:
            for i in range(len(path)-1):
                x1,y1 = path[i].get_xy();
                x2,y2 = path[i+1].get_xy();
                plt.plot([x1,x2],[y1,y2],c='y',ls='--');

        axes = plt.gca();
        axes.set_aspect(1);

        plt.show();

    def reset_parents(self):
        for n in self.nodes:
            n.resetAncestory();

    def aStar(self,n1,n2):
        self.reset_parents();

        #See if nodes already exist in graph or if they need to be added
        i1 = None;
        i2 = None;
        for i in range(len(self.nodes)):
            if n1 is self.nodes[i]:
                i1 = i;
            if n2 is self.nodes[i]:
                i2 = i;

        rebuild = False;

        if i1 is None:
            self.append_node(n1);
            i1 = len(self.nodes)-1;
            rebuild = True;
        
        if i2 is None:
            self.append_node(n2);
            i2 = len(self.nodes)-1;
            rebuild = True;

        if rebuild:
            self.build_visibility();

        #if n1 or n2 have no neighbors, then no path can connect the two, so need to try a path between different nodes
        if not n1.neighbors or not n2.neighbors:
            return [];


        #A* search algorithm
        n1.parent = n1;
        n1.pathDist2Start = 0;
        n1.straightDist2End = n1.dist_to_node(n2);

        openSet = [n1];

        while openSet: #while there are still nodes in the openSet to evaluate
            curr = self.getLowestCostNode(openSet);
            
            if curr is n2: #reached the end
                path = [n2];
                while(path[-1] is not n1):
                    path.append(path[-1].parent)
                return path;

            for n in curr.neighbors:
                tent_score = curr.pathDist2Start + curr.dist_to_node(n);
                if n.pathDist2Start is None or tent_score < n.pathDist2Start:
                    n.parent = curr;
                    n.pathDist2Start = tent_score;
                    if n.straightDist2End is None:
                        n.straightDist2End = n.dist_to_node(n2);
                    if n not in openSet:
                        openSet.append(n);

        return [];
                

    def getLowestCostNode(self,nodes):
        id = None;
        low_cost = float('inf');

        for i in range(len(nodes)):
            cost = nodes[i].pathDist2Start + nodes[i].straightDist2End;
            if cost<low_cost:
                id = i;
                low_cost = cost;

        return nodes.pop(id);


'''
    def shortest_path(self,n1,n2):
        self.reset_parents();
        self.evaluated = [False for i in range(len(self.nodes))];

        #See if nodes already exist in graph or if they need to be added
        i1 = None;
        i2 = None;
        for i in range(len(self.nodes)):
            if n1 is self.nodes[i]:
                i1 = i;
            if n2 is self.nodes[i]:
                i2 = i;

        rebuild = False;

        if i1 is None:
            self.append_node(n1);
            i1 = len(self.nodes)-1;
            rebuild = True;
        
        if i2 is None:
            self.append_node(n2);
            i2 = len(self.nodes)-1;
            rebuild = True;

        if rebuild:
            self.build_visibility();

        if not n1.neighbors or not n2.neighbors:
            return [];

        n1.parent = n1;
        n1.pathDist2Start = 0;
        while not all(self.evaluated): #REVISIT: Nodes that are within obstacles aren't visible by anyone. Don't evaluate nodes that have no neighbors. Return error code if there is no path to node
            for i in range(len(self.nodes)):
                if not self.nodes[i].neighbors: #has no neighbors. Effectively cannot be seen by anyone
                    self.evaluated[i] = True;
                if self.nodes[i].parent is not None and not self.evaluated[i]:
                    self.nodes[i].evaluate_neighbors();
                    self.evaluated[i] = True;

        path = [n2];

        while(path[-1] is not n1):
            path.append(path[-1].parent)

        return path;
'''