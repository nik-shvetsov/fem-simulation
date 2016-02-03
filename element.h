#ifndef ELEMENT_H
#define ELEMENT_H


#include "node.h"

class Element : public GMlib::TriangleFacets<float>
{
public:

  Element() {}
  ~Element() {}

private:

  Node _centreNode;
  GMlib::ArrayLX<GMlib::TSTriangle<float>> _triangles;
  GMlib::ArrayLX<GMlib::TSEdge<float>> _edges;

}; // END class element



#endif // ELEMENT_H
