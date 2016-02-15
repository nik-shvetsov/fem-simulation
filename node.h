#ifndef NODE_H
#define NODE_H

#include <gmTrianglesystemModule>

class Node //class for representing an element: with center node, adjacent triangles and edges
{
public:

   Node();
   ~Node();
   Node(GMlib::TSVertex<float> &centrVert);
   GMlib::Array<GMlib::TSTriangle<float>*> getTriangles(); //method for getting an array of adjacent triangles of a vertex
   GMlib::Array<GMlib::TSEdge<float>*> getEdges(); //method for getting an array of adjacent edges of a vertex
   GMlib::TSVertex<float>* getMainVertex(); //method for getting center vertex of a node
   void setZ(float z); //method for setting z-coordinate for a vertex or set of vertices

private:

  GMlib::TSVertex<float>* _centrVert;

}; // END class Node

#endif // NODE_H
