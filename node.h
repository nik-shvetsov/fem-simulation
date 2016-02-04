#ifndef NODE_H
#define NODE_H


#include <gmTrianglesystemModule>

//using namespace GMlib;

class Node
{
public:

   Node(){}

   Node(GMlib::TSVertex<float> &centrVert)
   {
       _centrVert = &centrVert;
   }

   GMlib::Array<GMlib::TSTriangle<float>*> getTriangles()
   {
       return _centrVert->getTriangles();
   }

   GMlib::Array<GMlib::TSEdge<float>*> getEdges()
   {
       return _centrVert->getEdges();
   }

   GMlib::TSVertex<float>* getMainVertex()
   {
       return _centrVert;
   }

   void setZ(float z)
   {
       if (_centrVert!=nullptr)
       {
            _centrVert->setZ(z);
       }

   }

  ~Node() {}

protected:

private:

  GMlib::TSVertex<float>* _centrVert;

}; // END class Node



#endif // NODE_H
