#ifndef FEMOBJECT_H
#define FEMOBJECT_H

#include "node.h"

class Femobject : public GMlib::TriangleFacets<float>
{
public:

  Femobject();
  ~Femobject();
  void randomTriangulation(double radius, int triangles); //building random points for triangulation
  float RandomFloat(float a, float b); //method for generating random float number from one to another
  void regularTriangulation(float small_radius, int rings, int innerNodes); //building regular points for triangulation
  GMlib::Vector<GMlib::Vector<float,2>,3> findVectors(GMlib::TSEdge<float>* edge); //method for finding vectors for calculation using common edge(non-diagonal A's)
  GMlib::Vector<GMlib::Vector<float,2>,3> findVectors(GMlib::TSTriangle<float>* tri, Node* node); //method for finding vectors for calculation using triangle and center node(diagonal A's)
  void performComputations(); //calculating stiffness matrix _A and matrix _b
  void updateFem(double force); //setting Z(height) value for every node using force value

protected:

  void localSimulate(double dt) override; //simulating of the calculated model

private:

  GMlib::ArrayLX<Node> _nodes;
  GMlib::DMatrix<float> _A;
  GMlib::DVector<float> _b;

  bool _goUp;
  float _force;
  float _maxForce;

}; // END class Femobject



#endif // FEMOBJECT_H
