#ifndef NODE_H
#define NODE_H


#include <gmTrianglesystemModule>

//using namespace GMlib;

class Node
{
public:
   Node() {}

   GMlib::Array<GMlib::TSTriangle<float>*> getTriangles()
   {
       return _conVert->getTriangles();
   }

  ~Node() {}

protected:

private:

  GMlib::TSVertex<float>* _conVert;

}; // END class Node



#endif // NODE_H
