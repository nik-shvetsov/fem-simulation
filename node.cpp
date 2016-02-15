#include "node.h"

   Node::Node() {}
   Node::~Node() {}

   Node::Node(GMlib::TSVertex<float> &centrVert)
   {
       _centrVert = &centrVert;
   }

   GMlib::Array<GMlib::TSTriangle<float>*> Node::getTriangles() //method for getting an array of adjacent triangles of a vertex
   {
       return _centrVert->getTriangles();
   }

   GMlib::Array<GMlib::TSEdge<float>*> Node::getEdges() //method for getting an array of adjacent edges of a vertex
   {
       return _centrVert->getEdges();
   }

   GMlib::TSVertex<float>* Node::getMainVertex() //method for getting center vertex of a node
   {
       return _centrVert;
   }

   void Node::setZ(float z) //method for setting z-coordinate for a vertex or set of vertices
   {
       if (_centrVert!=nullptr)
       {
            _centrVert->setZ(z);
       }
   }
