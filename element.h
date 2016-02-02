#ifndef ELEMENT_H
#define ELEMENT_H


#include "node.h"

class Element : public GMlib::TriangleFacets<float>
{
public:

  Element() {}
  ~Element() {}

protected:
  void localSimulate(double dt) override
  {

  }

private:

  GMlib::ArrayLX<Node> _nodes;
  GMlib::DMatrix<float> _A;
  GMlib::DVector<float> _b;


}; // END class element



#endif // ELEMENT_H
