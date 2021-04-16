import math
import sys
import time
import os
from OCC.Display.SimpleGui import init_display
from OCC.Extend.DataExchange import read_stl_file
from OCC.Extend.DataExchange import read_iges_file
from OCC.Core.BRepExtrema import *
from OCC.Display.SimpleGui import init_display
from OCC.Core.StlAPI import *
from OCC.Core.TopoDS import *
from OCC.Display.SimpleGui import *
from OCC.Extend.DataExchange import *
from OCC.Core.TopoDS import *
from OCC.Core.TopExp import *
from OCC.Core.TopAbs import *
from OCC.Core.TopTools import *
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import *
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCC.Core.BRepTools import breptools_Write

from OCC.Display.SimpleGui import *
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRep import *
from OCC.Core.BRepAlgoAPI import *
from OCC.Core.BRepBuilderAPI import *
from OCC.Core.BRepPrimAPI import *
from OCC.Display.SimpleGui import init_display
from OCC.Core.gp import *
from OCC.Core.BRepOffsetAPI import *
from OCC.Core.BRepOffset import *
from OCC.Core.GeomAbs import *
from OCC.Core.BRepAlgo import *
display, start_display, add_menu, add_function_to_menu = init_display()

def z_max_finder(stl_shp,tol=1e-6,use_mesh=True):
    """first change the model to mesh form in order to get an
    accurate MAX and Min bounfing box from topology"""
    bbox = Bnd_Box()
    bbox.SetGap(tol)
    if use_mesh:
        mesh = BRepMesh_IncrementalMesh()
        mesh.SetParallelDefault(True)
        mesh.SetShape(stl_shp)
        mesh.Perform()
        if not mesh.IsDone():
            raise AssertionError("Mesh not done.")
    brepbndlib_Add(stl_shp, bbox, use_mesh)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return zmax

def get_vertex_from_edge(aEdge):
    myvertexlist=[]
    VertexExplorer = TopExp_Explorer(aEdge, TopAbs_VERTEX)
    aVertex1 = topods.Vertex(VertexExplorer.Current())
    VertexExplorer.Next()
    aVertex2 = topods.Vertex(VertexExplorer.Current())
    myvertexlist.append(aVertex1)
    myvertexlist.append(aVertex2)
    return myvertexlist
    
def Get_my_X(aVertex):
     #translating the vertex to a point to get point location
    point=BRep_Tool.Pnt(aVertex)
    x=point.X()
    return x

def Get_my_Y(aVertex):
     #translating the vertex to a point to get point location
    point=BRep_Tool.Pnt(aVertex)
    return point.Y()

def Get_my_Z(aVertex):
     #translating the vertex to a point to get point location
    point=BRep_Tool.Pnt(aVertex)
    z=point.Z()
    return z

def raster_from_wire(aWire):
    aPrismVec = gp_Vec(0, 0,200)
    aVertex=aWire.Vertex()
    z=Get_my_Z(aVertex)
    f1=BRepBuilderAPI_MakeFace (aWire.Wire())
    myBody = BRepPrimAPI_MakePrism(f1.Face(), aPrismVec)
    myedge=BRepBuilderAPI_MakeEdge(gp_Pnt(0,0,z),gp_Pnt(50,0,z))
    boolean_result =BRepAlgoAPI_Section(myBody.Shape(),myedge.Shape())
    display.DisplayShape(boolean_result.Shape())
    
def make_edges_list(section):
    sections2=[]
    EdgeExplorer = TopExp_Explorer(section.Shape(), TopAbs_EDGE)
    while EdgeExplorer.More():
        aEdge = topods.Edge(EdgeExplorer.Current())
        sections2.append(aEdge)
        EdgeExplorer.Next()
    return sections2
    
def find_outer_boundary(edges_list):
    boundary_edge_list=[]
    #getting first edge from the list
    first_edge_vertex_list=get_vertex_from_edge(edges_list[0])
    #getting a reference point-where the boundary starts and finishes
    reference_point=[Get_my_X(first_edge_vertex_list[0]),Get_my_Y(first_edge_vertex_list[0])]
    #getting runner point which is a common point between current edge and next connected edge
    runner_point=[Get_my_X(first_edge_vertex_list[1]),Get_my_Y(first_edge_vertex_list[1])]
    boundary_edge_list.append(edges_list[0])
    nb=0
    for m in range(len(edges_list)-2):
        for i in range(len(edges_list)-1):
            aEdge1=edges_list[i+1]
            vertex_list1=get_vertex_from_edge(aEdge1)
            vertex1=[Get_my_X(vertex_list1[0]),Get_my_Y(vertex_list1[0]),Get_my_Z(vertex_list1[0])]
            vertex2=[Get_my_X(vertex_list1[1]),Get_my_Y(vertex_list1[1]),Get_my_Z(vertex_list1[1])]
        
            if vertex1[0]==runner_point[0] and vertex1[1]==runner_point[1] and nb!=i+1:
                boundary_edge_list.append(aEdge1)
                runner_point= vertex2
                nb=i+1
            elif vertex2[0]==runner_point[0] and vertex2[1]==runner_point[1] and nb!=i+1:
                boundary_edge_list.append(aEdge1)
                runner_point= vertex1
                nb=i+1
    return boundary_edge_list
            
def Section_finder(Z_level_depth,stl_shp):
   sections = []
   sections2 = []
   vertexlist=[]

   point1=gp_Pnt(0,0,Z_level_depth)
   dir1=gp_Dir(0,0,1)
   plan1=gp_Pln(point1,dir1)
   section_shp = BRepAlgoAPI_Section()
   section_shp = BRepAlgoAPI_Section(stl_shp, plan1, False)
   section_shp.Build()
    #getting list of edges from section
   edges_list=make_edges_list(section_shp)
   outer_boundary=find_outer_boundary(edges_list)
   #for k in range(len(outer_boundary)):
      # display.DisplayShape(outer_boundary[k])
    #making wire from edges
   mkWire=BRepBuilderAPI_MakeWire()   
   for k in range(len(outer_boundary)):
       aWire = BRepBuilderAPI_MakeWire(outer_boundary[k])
       mkWire.Add(aWire.Wire())
   return mkWire
        
def top_level_raster(x1,y1,x2,y2,z,step_over):
    #x1,y1,are the coordinate of first vertex(down left) of a rectangule 
    #x2,y2,are the coordinate of second(right top) vertex of rectangule
    #step_over is distance between each pass
    #z should be higher than model
    nb_pass=abs(y1-y2)//step_over
    step_over=abs(y1-y2)/nb_pass
    mkWire=BRepBuilderAPI_MakeWire()
    xp=x1
    yp=y1
    point=[xp,yp,z]
    point_list=[]
    point_list.append(point)
   
    for i in range(int(nb_pass)):
       if i%2==0:
           edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x2,yp,z))
           xp=x2
           point=[xp,yp,z]
           point_list.append(point)
           edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
           yp=yp+step_over
           point=[xp,yp,z]
           point_list.append(point)
           
       else:
           edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x1,yp,z))
           xp=x1
           point=[xp,yp,z]
           point_list.append(point)
           edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
           yp=yp+step_over
           point=[xp,yp,z]
           point_list.append(point)     
   
       aWire1=BRepBuilderAPI_MakeWire(edge1.Edge())
       aWire2=BRepBuilderAPI_MakeWire(edge2.Edge())
       mkWire.Add(aWire1.Wire())
       mkWire.Add(aWire2.Wire())
   #display.DisplayShape(mkWire.Shape())
    
    VertexExplorer = TopExp_Explorer(mkWire.Shape(), TopAbs_VERTEX)
    while VertexExplorer.More():
          aVertex = topods.Vertex(VertexExplorer.Current())
          VertexExplorer.Next()
    return point_list
               
               
def translate_topods_from_vector(brep_or_iterable, vec, copy=False):
    '''
    translate a brep over a vector
    @param brep:    the Topo_DS to translate
    @param vec:     the vector defining the translation
    @param copy:    copies to brep if True
    '''
    trns = gp_Trsf()
    trns.SetTranslation(vec)
    brep_trns = BRepBuilderAPI_Transform(brep_or_iterable, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def Load_CAD_Model_STL():
    
    stl_shp = read_stl_file('mymodel1.stl')
    # for reading IGS files
    #stl_shp=read_iges_file('mymodel4.igs')
    #stl_shp=TopoDS_Shape(stl_shp)
        
#height of chamber box
    hieght=z_max_finder(stl_shp)
    
    #making chamber box
    box_shape = BRepPrimAPI_MakeBox(gp_Pnt(-18, -18, 0),65, 65, hieght).Shape()
   
    display.DisplayShape(stl_shp)
    display.DisplayShape(box_shape,color = "BLACK", transparency = 0.7)
    
    display.FitAll()
 
def make_vertex_list(section):
    #getting a vertex from a intersection between an offset Wire and an Edge
    sections2=[]
    VertexExplorer = TopExp_Explorer(section.Shape(), TopAbs_VERTEX)
    while VertexExplorer.More():
          aVertex = topods.Vertex(VertexExplorer.Current())
          sections2.append(aVertex)
          VertexExplorer.Next()
          
    if sections2!=[]:
       return sections2
    else:
        a=int(0)
        return a
    
def find_closest_vertex(vertex_list,x):
    distance=10000
    
    for i in range(len(vertex_list)) :
        my_X=Get_my_X(vertex_list[i])
        if abs(my_X-x)<distance:
            distance=abs(my_X-x)
            my_vertex=vertex_list[i]
    
    return my_vertex
        
def section_point_finder_left(x1,x2,y,z,offset):
#finding the intersection point between a raster line and the offset wire
    myedge=BRepBuilderAPI_MakeEdge(gp_Pnt(x1,y,z),gp_Pnt(x2,y,z))
    boolean_result =BRepAlgoAPI_Section(offset.Shape(),myedge.Shape())
    mynewvertex=make_vertex_list(boolean_result)
    
    if type(mynewvertex)!=int:
        best_vertex=find_closest_vertex(mynewvertex,x2)
        x=Get_my_X(best_vertex)
        y=Get_my_Y(best_vertex)
        z=Get_my_Z(best_vertex)
        point13=gp_Pnt(x,y,z)
        #display.DisplayShape(point13)
        return point13
    else:
        a=int(0)
        return a
    
def section_point_finder_right(x1,x2,y,z,offset):
#finding the intersection point between a raster line and the offset wire
    myedge=BRepBuilderAPI_MakeEdge(gp_Pnt(x1,y,z),gp_Pnt(x2,y,z))
    boolean_result =BRepAlgoAPI_Section(offset.Shape(),myedge.Shape())
    mynewvertex=make_vertex_list(boolean_result)
    
    if type(mynewvertex)!=int:
        best_vertex=find_closest_vertex(mynewvertex,x1)
        x=Get_my_X(best_vertex)
        y=Get_my_Y(best_vertex)
        z=Get_my_Z(best_vertex)
        point13=gp_Pnt(x,y,z)
        #display.DisplayShape(point13)
        return point13
    else:
        a=int(0)
        return a    
    
def raster_from_offset(x1,y1,x2,y2,z,step_over,offset_wire) :
    #collecting all edges in a list to make a wire at the end
    edges_list_1=[]
    #list of points for simulation
    point_list_right=[]
    point_list_left=[]
    current_point=[]
    #number of steps
    nb_steps=int(abs(y1-y2)//step_over)
    #proper lebgth of step over to cover whole area
    step_over=abs(y1-y2)/nb_steps
    
    #area is been devided in to two in order to be accessable for section finding
    x_mid=(abs(x1-x2)/2)+x1+2
    x_section=x1+1

    #current position is described and changes during loop
    xp=x2
    yp=y1
    current_point=[xp,yp,z]
    point_list_right.append(current_point)
    for i in range(nb_steps):
        section_point1=section_point_finder_right(x2,x_section,yp,z,offset_wire)
        section_point2=section_point_finder_right(x2,x_section,yp+step_over,z,offset_wire)

        if i%2==0:
           if type(section_point1)==int:
               edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x_mid,yp,z))
               xp=x_mid
               current_point=[xp,yp,z]
               point_list_right.append(current_point)
               if type(section_point2)==int:
                  edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
                  yp=yp+step_over
                  current_point=[xp,yp,z]
                  point_list_right.append(current_point)
               else:
                   edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point2)
                   xp=section_point2.X()
                   yp=section_point2.Y()
                   current_point=[xp,yp,z]
                   point_list_right.append(current_point)
                   
           else:
               edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point1)
               xp=section_point1.X()
               yp=section_point1.Y()
               current_point=[xp,yp,z]
               point_list_right.append(current_point)
               if type(section_point2)==int:
                  edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
                  yp=yp+step_over
                  current_point=[xp,yp,z]
                  point_list_right.append(current_point)
               else:
                   edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point2)
                   xp=section_point2.X()
                   yp=section_point2.Y()
                   current_point=[xp,yp,z]
                   point_list_right.append(current_point)
           
        else:
           edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x2,yp,z))
           xp=x2
           current_point=[xp,yp,z]
           point_list_right.append(current_point)
           edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
           yp=yp+step_over
           current_point=[xp,yp,z]
           point_list_right.append(current_point)
        #display.DisplayShape(edge1.Shape())
        #display.DisplayShape(edge2.Shape())
    x_mid=(abs(x1-x2)/2)+x1-2
    x_section=x2-1
    
    #connecting two path
    safe_z=60
    current_point=[xp,yp,safe_z]
    point_list_right.append(current_point)
    #current position is described and changes during loop
    xp=x1
    yp=y1
    current_point=[xp,yp,safe_z]
    point_list_right.append(current_point)
    current_point=[xp,yp,z]
    for i in range(nb_steps):
        section_point1=section_point_finder_left(x_section,x1,yp,z,offset_wire)
        section_point2=section_point_finder_left(x_section,x1,yp+step_over,z,offset_wire)
        if i%2==0:
           if type(section_point1)==int:
               edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x_mid,yp,z))
               xp=x_mid
               current_point=[xp,yp,z]
               point_list_right.append(current_point)
               if type(section_point2)==int:
                  edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
                  yp=yp+step_over
                  current_point=[xp,yp,z]
                  point_list_right.append(current_point)
               else:
                   edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point2)
                   xp=section_point2.X()
                   yp=section_point2.Y()
                   current_point=[xp,yp,z]
                   point_list_right.append(current_point)
           else:
               edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point1)
               xp=section_point1.X()
               yp=section_point1.Y()
               current_point=[xp,yp,z]
               point_list_right.append(current_point)
               if type(section_point2)==int:
                  edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
                  yp=yp+step_over
                  current_point=[xp,yp,z]
                  point_list_right.append(current_point)
               else:
                   edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),section_point2)
                   xp=section_point2.X()
                   yp=section_point2.Y()
                   current_point=[xp,yp,z]
                   point_list_right.append(current_point)
        else:
           edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(x1,yp,z))
           xp=x1
           current_point=[xp,yp,z]
           point_list_right.append(current_point)
           edge2=BRepBuilderAPI_MakeEdge(gp_Pnt(xp,yp,z),gp_Pnt(xp,yp+step_over,z))
           yp=yp+step_over
           current_point=[xp,yp,z]
           point_list_right.append(current_point)
           
    current_point=[xp,yp,safe_z]
    point_list_right.append(current_point) 
    current_point=[x1,y1,safe_z]
    point_list_right.append(current_point)
        #display.DisplayShape(edge1.Shape())
        #display.DisplayShape(edge2.Shape())
    return point_list_right
def Generating_Path():
    suction_Dia=12
    cover_percentage=0.4
    step_over=suction_Dia*cover_percentage
    step_down=suction_Dia*0.5
    Generating_Path_function(step_over,step_down)
    
def Generating_Path_function(step_over,step_down):
    
    
    raster_point_list=[]
    list1=[]
    path_points=[]
  
    stl_shp = read_stl_file('mymodel22.stl')
    
    #stl_shp2=read_iges_file('mymodel4.igs')
    z_max=z_max_finder(stl_shp)
    #number of possible levels
    nb_z_level= z_max//step_down
    nb_z_level=int(nb_z_level)
    #proper z level distance for the current loaded model with respect to standard
    my_z_level=z_max/nb_z_level
    for i in range(nb_z_level):
        # to avoid having section on z=0 then +1 is added to first level.
        myWire=Section_finder(i*my_z_level+1,stl_shp)
        list1.append(myWire)
    offset_oriented_list=[]
    z_level_list=[]
    for j in range(len(list1)):
        z_level=Get_my_Z(list1[j].Vertex())
        offset=BRepOffsetAPI_MakeOffset(list1[j].Wire(),GeomAbs_Intersection,False)
        offset.Perform(0.5,0.0)
        
        display.DisplayShape(offset.Shape())
        
        offset2=BRepOffsetAPI_MakeOffset(list1[j].Wire(),GeomAbs_Intersection,False)
        offset2.Perform(4,0.0)
        
        z_level_list.append(z_level)
        #to generate raster from offset path
        
        raster1=raster_from_offset(-20,-19,45,45,z_level,step_over,offset2)
        raster_point_list.append(raster1)
        
        #to generate offset oriented path
        offset_edge_list=make_edges_list(offset)
        offset_points=find_boundary_points_form_offset(offset_edge_list)
        offset_oriented_list.append(offset_points)
    
    safe_point=[1,1,z_max+2]
    top_level_point_list=top_level_raster(-20,-20,45,45,z_max,step_over)
    for k in range(len(top_level_point_list)):
          path_points.append(top_level_point_list[k])
    path_points.append(safe_point)
    
    j=0
    i=0
    for i in range(len(raster_point_list)):
        entering_point=[raster_point_list[-i-1][0][0],raster_point_list[-i-1][0][1],z_max+5]
        path_points.append(entering_point) 
        for j in range(len(raster_point_list[-i-1])):
            path_points.append(raster_point_list[-i-1][j])
            rising_point=[raster_point_list[-i-1][j][0],raster_point_list[-i-1][j][1],z_max+5]
            
        path_points.append(rising_point)
        path_points.append(safe_point)
  
    #a different path_point list for offset oriented(with pitch angle)
    i=0
    j=0
    path_points2=[]
    for i in range(len(offset_oriented_list)):
        entering_point=[offset_oriented_list[-i-1][0][0],offset_oriented_list[-i-1][0][1],z_max+5,0]
        path_points2.append(entering_point)
        for j in range(len(offset_oriented_list[-i-1])):
            path_points2.append(offset_oriented_list[-i-1][j])
            rising_point=[offset_oriented_list[-i-1][j][0],offset_oriented_list[-i-1][j][1],z_max+5,0]
            
        path_points2.append(rising_point)
      
    #point_path is a list of points that each point is a list with x,y,z   in order to show
    # the path we should translate each point to a gp_Pnt calss
    i=0
    path_points_gp=[]
    for i in range(len(path_points)):
       xp=path_points[i][0] 
       yp=path_points[i][1] 
       zp=path_points[i][2]
       apoint=gp_Pnt(xp,yp,zp)
       path_points_gp.append(apoint)
    i=0
    path_length=0
    for i in range(len(path_points_gp)-1):
        path_edge=BRepBuilderAPI_MakeEdge(path_points_gp[i],path_points_gp[i+1])
        display.DisplayShape(path_edge.Shape())
        
        g1 = GProp_GProps()
        brepgprop_LinearProperties(path_edge.Shape(), g1)
        length = g1.Mass()
        path_length+= length
    
    i=0
    path_points_gp=[]
    for i in range(len(path_points2)):
       xp=path_points2[i][0] 
       yp=path_points2[i][1] 
       zp=path_points2[i][2]
       apoint=gp_Pnt(xp,yp,zp)
       path_points_gp.append(apoint)
    i=0
    for i in range(len(path_points_gp)-1):
        path_edge=BRepBuilderAPI_MakeEdge(path_points_gp[i],path_points_gp[i+1])
        display.DisplayShape(path_edge.Shape())
        g1 = GProp_GProps()
        brepgprop_LinearProperties(path_edge.Shape(), g1)
        length = g1.Mass()
        path_length+= length
    print('Path Length=', path_length)
     
    return path_points,path_points2
    
def vertex_distance(vertex1,vertex2):
    x1=Get_my_X(vertex1)   
    y1=Get_my_Y(vertex1)
    z1=Get_my_Z(vertex1)
    x2=Get_my_X(vertex2)
    y2=Get_my_Y(vertex2)
    z2=Get_my_Z(vertex2)
    distance=math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
    return distance
    
def find_dir(point1,point2,angle):
    vector=[point2[0]-point1[0]+0.00001,point2[1]-point1[1]+0.00001]
    #dot product of 2 vectors is zero if they are perpandicular
    v1=1
    if vector[1]>0:
        v2=-vector[0]/vector[1]
    else:
        v1=-1
        v2=vector[0]/vector[1]
    v_length=math.sqrt(v1**2+v2**2)
    v3=math.atan((angle*math.pi)/180)*v_length
    dir1=[v1,v2,v3]
    return dir1

def make_move(point1,point2,angle):
    #moving from point1 to point2
    p1=gp_Pnt(point1[0],point1[1],point1[2])    
    p2=gp_Pnt(point2[0],point2[1],point2[2])  
    velocity_factor=0.1
    speed=int((abs(point2[0]-point1[0])+abs(point2[1]-point1[1])+abs(point2[2]-point1[2]))*velocity_factor)+1
    delta_x=(point2[0]-point1[0])/speed
    delta_y=(point2[1]-point1[1])/speed
    delta_z=(point2[2]-point1[2])/speed
    
    if 0<angle<90:
        dir1=find_dir(point2,point1,angle)
    else:
        dir1=0 
    my_cylinder=build_shape(point1[0],point1[1],point1[2],dir1)
    my_cylinder_Trsf= gp_Trsf()
    #speed+1 due to counting i forn zero
    for i in range( speed+1):
        my_cylinder_Trsf.SetTranslation(gp_Pnt(point1[0],point1[1],point1[2]),
                                        gp_Pnt(point1[0]+delta_x*i,point1[1]+delta_y*i,
                                               point1[2]+delta_z*i))
        my_cylinder_location = TopLoc_Location(my_cylinder_Trsf)
        display.Context.SetLocation(my_cylinder, my_cylinder_location)
        display.Context.UpdateCurrentViewer()
    display.Context.Remove(my_cylinder,True)
    
    
def build_shape(x,y,z,dir1):
    point1=gp_Pnt(x,y,z)
    if type(dir1)!=int:
        dir1=gp_Dir(dir1[0],dir1[1],dir1[2])
    else:
        dir1=gp_Dir(0,0,10)
        
    axis1=gp_Ax2(point1,dir1)
    boxshp = BRepPrimAPI_MakeCylinder(axis1,2,20).Shape()
    ais_boxshp = display.DisplayShape(boxshp,color = "RED", transparency = 0.1, update=True)[0]
    return ais_boxshp 

def find_boundary_points_form_offset(edges_list):
    boundary_point_list=[]
    #getting first edge from the list
    first_edge_vertex_list=get_vertex_from_edge(edges_list[0])
    #getting a reference point-where the boundary starts and finishes
    reference_point=[Get_my_X(first_edge_vertex_list[0]),Get_my_Y(first_edge_vertex_list[0]),Get_my_Z(first_edge_vertex_list[0])]
    #getting runner point which is a common point between current edge and next connected edge
    runner_point=[Get_my_X(first_edge_vertex_list[1]),Get_my_Y(first_edge_vertex_list[1]),Get_my_Z(first_edge_vertex_list[1])]
    boundary_point_list.append(reference_point)
    boundary_point_list.append(runner_point)
    edge_nb=0
    for m in range(len(edges_list)-2):
        for i in range(len(edges_list)-1):
            aEdge1=edges_list[i+1]
            vertex_list1=get_vertex_from_edge(aEdge1)
            vertex1=[Get_my_X(vertex_list1[0]),Get_my_Y(vertex_list1[0]),Get_my_Z(vertex_list1[0])]
            vertex2=[Get_my_X(vertex_list1[1]),Get_my_Y(vertex_list1[1]),Get_my_Z(vertex_list1[1])]
        
            if vertex1[0]==runner_point[0] and vertex1[1]==runner_point[1]and edge_nb!=i+1 :
                boundary_point_list.append(vertex2)
                runner_point= vertex2
                edge_nb=i+1
                break
            elif vertex2[0]==runner_point[0] and vertex2[1]==runner_point[1]and edge_nb!=i+1:
                runner_point= vertex1
                boundary_point_list.append(vertex1)
                edge_nb=i+1
                break
    boundary_point_list.append(reference_point)
    #boundary_point_list.append([boundary_point_list[-1][0],boundary_point_list[-1][1],40])
    point_list_2=[]
    i=0
    pitch_angle=45
    for i in range(len(boundary_point_list)-1):
        point1=boundary_point_list[i]
        point2=boundary_point_list[i+1]
        length=math.sqrt((point2[0]-point1[0])**2+(point2[1]-point1[1])**2+(point2[2]-point1[2])**2)
        
        if length>0.5:
            new_point1=[point1[0],point1[1],point1[2],pitch_angle]
            point_list_2.append(new_point1)
    #point_list_2.append(boundary_point_list[-1])
    return point_list_2

def simulate(suction_Dia,point_list,angle,material_box):
    boolean_result=material_box
    n=0

    for j in range(len(point_list)-1):
        point1=point_list[j]
        point2=point_list[j+1]
        #making a cylinder to boolean_cut from material box to simulate powder removing process
        dir2=gp_Dir(point2[0]-point1[0],point2[1]-point1[1],point2[2]-point1[2])
        length=math.sqrt((point2[0]-point1[0])**2+(point2[1]-point1[1])**2+(point2[2]-point1[2])**2)+2
        axis1=gp_Ax2(gp_Pnt(point1[0],point1[1],point1[2]),dir2)
        
        plan1=gp_Pln(gp_Pnt(point1[0],point1[1],point1[2]),dir2)
        vec1=gp_Vec(gp_Pnt(point1[0],point1[1],point1[2]),gp_Pnt(point2[0],point2[1],point2[2]))
        face1=BRepBuilderAPI_MakeFace(plan1,-suction_Dia/2,suction_Dia/2,-suction_Dia/2,suction_Dia/2)
        '''
        my_cylinder_simulation=BRepPrimAPI_MakePrism(face1.Face(),vec1).Shape()
        
        edge1=BRepBuilderAPI_MakeEdge(gp_Pnt(point1[0],point1[1],point1[2]),gp_Pnt(point2[0],poin2[1],point2[2]))
        aWire1=BRepBuilderAPI_MakeWire(edge1.Edge())'''
        
        my_cylinder_simulation = BRepPrimAPI_MakeCylinder(axis1,suction_Dia/2,length).Shape()
        
        #boolean_result1=BRepAlgoAPI_Cut(boolean_result.Shape(), my_cylinder_simulation)
        #if boolean_result1.IsDone():
        #boolean_result=BRepAlgoAPI_Cut(boolean_result.Shape(), my_cylinder_simulation)
        tol=1e-3
        parallel=True
        shape_A=boolean_result.Shape()
        shape_B=my_cylinder_simulation
        cut = BRepAlgoAPI_Cut()
        L1 = TopTools_ListOfShape()
        L1.Append(shape_A)
        L2 = TopTools_ListOfShape()
        L2.Append(shape_B)
        cut.SetArguments(L1)
        cut.SetTools(L2)
        cut.SetFuzzyValue(tol)
        cut.SetRunParallel(parallel)
        cut.Build()
        boolean_result=cut
        
        if j==0:
            ais_boolean_shp1 = display.DisplayShape(boolean_result.Shape(),color = "BLACK", transparency = 0, update=True)[0]
            ais_boolean_shp2 = display.DisplayShape(boolean_result.Shape(),color = "BLACK", transparency = 0, update=True)[0]
            mymove=make_move(point1,point2,angle)
            display.Context.Remove(ais_boolean_shp1,True)
        elif j==(len(point_list)-2):
            display.Context.Remove(ais_boolean_shp1,True)
            display.Context.Remove(ais_boolean_shp2,True)
    
        elif j%2==0 :
            ais_boolean_shp2 = display.DisplayShape(boolean_result.Shape(),color = "BLACK", transparency = 0, update=True)[0]
            mymove=make_move(point1,point2,angle)
            display.Context.Remove(ais_boolean_shp1,True)
        else:
            ais_boolean_shp1 = display.DisplayShape(boolean_result.Shape(),color = "BLACK", transparency = 0, update=True)[0]
            mymove=make_move(point1,point2,angle)
            display.Context.Remove(ais_boolean_shp2,True)
            
    return boolean_result

        
    #ais_boolean_shp = display.DisplayShape(boolean_result.Shape(),color = "BLACK", transparency = 0, update=True)[0]
    
def show_path_from_points(point_list):
    for i in range(len(point_list)-1):
        point1=point_list[i]
        point2=point_list[i+1]
        edge=BRepBuilderAPI_MakeEdge(gp_Pnt(point1[0],point1[1],point1[2]),gp_Pnt(point2[0],point2[1],point2[2]))
        display.DisplayShape(edge.Shape())

def Simulate_Path():
    suction_Dia=12
    cover_percentage=0.4
    step_over=suction_Dia*cover_percentage
    step_down=suction_Dia*0.5
    point_list=Generating_Path_function(step_over,step_down)
    display.EraseAll()
    stl_shp1 = read_stl_file('mymodel1.stl')
    props3 = GProp_GProps()
    brepgprop_VolumeProperties(stl_shp1, props3)
    model_mass = props3.Mass()
    
    
    view_box=BRepPrimAPI_MakeBox(gp_Pnt(-30, -30, 0),100, 100, 60)
    display.DisplayShape(view_box.Shape(),color = "BLACK",transparency = 0.999)
    display.DisplayShape(stl_shp1)
    #powder box=material
    material_box=BRepPrimAPI_MakeBox(gp_Pnt(-18, -18, 0),58, 58, 22)

    props2 = GProp_GProps()
    brepgprop_VolumeProperties(material_box.Shape(), props2)
    mass2 = props2.Mass()
    
    
    first_simulate= simulate(suction_Dia,point_list[0],90,material_box)
    
    second_simulate=simulate(suction_Dia,point_list[1],45,first_simulate)

    props = GProp_GProps()
    brepgprop_VolumeProperties(second_simulate.Shape(), props)
    mass = props.Mass()
    print("remained volume = %",(mass*100/(mass2-model_mass)))
    display.DisplayShape(second_simulate.Shape(),color = "BLACK",transparency = 0.3)


def reverse_point_list(point_list):
    point_list2=[]
    for i in range(len(point_list)-1):
        point_list2.append(point_list[-i-1])
    return point_list2
        
        
def Simulate_raster_from_offset():
    path=Generating_Path()
    path_offset=path[0]
    path_z_level=path[1]
    display.EraseAll()
    stl_shp = read_stl_file('mymodel3.stl')
    #stl_shp2 = read_stl_file('mymodel2.stl') 
    # for reading IGS files
    stl_shp2=read_iges_file('mymodel4.igs')
    all_points=[]
    hieght=z_max_finder(stl_shp)
    #making chamber box
    material_box = BRepPrimAPI_MakeBox(gp_Pnt(-70, -70, 0),130, 130, hieght)
    display.DisplayShape(stl_shp)
    #display.DisplayShape(box_shape,color = "BLACK", transparency = 0.7)
    for i in range(len(path_z_level)-1):
        my_path=raster_from_offset(-70,-70,65,65,path_z_level[-i-1],10,path_offset[-i-1])
        show_path_from_points(my_path)
        all_points=my_connector(my_path,all_points)
    simulate(all_points,90,material_box)

def Simulate_offset():
    path=Generating_Path()
    display.EraseAll()
    stl_shp = read_stl_file('mymodel3.stl')  
    stl_shp2 = read_stl_file('mymodel2.stl')
# for reading IGS files
    #stl_shp=read_iges_file()
    hieght=z_max_finder(stl_shp)
    #making chamber box
    #box_shape = BRepPrimAPI_MakeBox(gp_Pnt(-25, -25, 0),60, 60, hieght).Shape()
    display.DisplayShape(stl_shp2)
    #display.DisplayShape(box_shape,color = "BLACK", transparency = 0.7)
    path_offset=path[0]
    path_z_level=path[1]
    all_points=[]
    for j in range(len(path_offset)-1):
        edge_list=make_edges_list(path_offset[-j-1])
        for i in range(len(edge_list)):
            display.DisplayShape(edge_list[i])
            outer_points=find_boundary_points_form_offset(edge_list)
        #all_points=my_connector(outer_points,all_points)
        simulate(outer_points,45)

def my_connector(point_list1,point_list2):
    for i in range(len(point_list1)):
        point_list2.append(point_list1[i])
    
    return point_list2
        
        
  
def Erase_Display():
    display.EraseAll()
  
if __name__ == '__main__':
    add_menu('Creat Trajectory')
    add_function_to_menu('Creat Trajectory', Load_CAD_Model_STL)
    add_function_to_menu('Creat Trajectory', Generating_Path)
    add_function_to_menu('Creat Trajectory', Erase_Display)
    add_function_to_menu('Creat Trajectory', Simulate_Path)

    
    

 
    start_display()