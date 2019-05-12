# Proposed Algorithm Overview
Map Layout is represented using a connected graph. Each vertex in the graph represents a key location that must be patrolled and each edge represents a possible path from one vertex to another without any obstacles in the way. 
'''
(0,0),(0,1),(1,0),(1,1)
(0,0):(0,1), (1,0)
(0,1):(0,0), (1,1)
(1,0):(0,0), (1,1)
(1,1):(0,1), (1,0)

'''
'''
class Vertex:
  def __init__(x, y, neighborlist):
    self.x = x
    self.y - y

neighborlist = [(0,1), (1,0)]
x = 0
y = 0


CAPTURE
  valid_vertices = []
  For each connected vertex next to current vertex
    Add connected vertex to valid vertices
  For all connected edges
    If robot is currently traveling along edge
      remove connected vertex associated with edge from valid vertices
  While valid_vertices:
    
      

EDGE_TRAVEL



    
