#ifndef FEMOBJECT_H
#define FEMOBJECT_H


#include "node.h"

class Femobject : public GMlib::TriangleFacets<float>
{
public:

  Femobject() {}
  ~Femobject() {}

  void randomTriangulation(double radius, int triangles)
  {
      double s = std::sqrt((4*M_PI*radius*radius)/(std::sqrt(3)*triangles));//length of side of a triangle
      int bordernodes = ((2*M_PI*radius)/s);
      int nodes = (triangles + 2 + bordernodes)/2;

      std::cout<<s<<std::endl<<bordernodes<<std::endl<<nodes<<std::endl;

      auto p0 = GMlib::Point<float,2>(0.0f, 0.0f);
      this->insertAlways(GMlib::TSVertex<float> (p0));

      //adding border points
      GMlib::Vector<float,2> rotVect(radius,0);
      GMlib::Angle a = M_2PI/(bordernodes);
      GMlib::SqMatrix<float,2> matrRot (a, GMlib::Vector<float,2> (1,0), GMlib::Vector<float,2> (0,1));

      for (int j=1; j<=(bordernodes); j++)
      {
          rotVect =  matrRot * rotVect;
          this->insertAlways(GMlib::TSVertex<float> (static_cast<GMlib::Point<float,2>>(rotVect)));
          //std::cout<<rotVect<<std::endl;
      }

      //radius correction
      GMlib::Vector<float,2> pointVector(radius,0);
      auto point1 = GMlib::Point<float,2>(static_cast<GMlib::Point<float,2>>(pointVector));
      pointVector = matrRot*pointVector;
      auto point2 = GMlib::Point<float,2>(static_cast<GMlib::Point<float,2>>(pointVector));
      auto distanceVect = GMlib::Vector<float,2>(point1-point2);
      double distance = distanceVect.getLength();

      double b = 2*(radius)*std::sin(a.getRad()/2);

      double radCorr = std::sqrt((b*b)-(distance/2)*(distance/2));

      double newRadius = radius-(radCorr*2);

      //std::cout<<radius<<std::endl<<distance<<std::endl<<b<<std::endl<<radCorr<<std::endl;


      for (int i=1; i<=nodes; i++)

      {
          auto pt = GMlib::TSVertex<float> (RandomFloat(-newRadius,newRadius), RandomFloat(-newRadius,newRadius));
          if (pt.getParameter().getLength() < newRadius)
          {
            this->insertAlways(pt);
            std::cout<<pt<<std::endl;
          }
          else
          {
              i--;
          }
      }

      this->triangulateDelaunay();
  }

  float RandomFloat(float a, float b) {
      float random = ((float) rand()) / (float) RAND_MAX;
      float diff = b - a;
      float r = random * diff;
      return a + r;
  }

  void regularTriangulation(float small_radius, int rings, int innerNodes)
  {
      auto p0 = GMlib::Point<float,2>(0.0f, 0.0f);
      this->insertAlways(GMlib::TSVertex<float> (p0));

      for (int i=1; i<=rings; i++)
      {
          GMlib::Vector<float,2> rotVect(float(i)*small_radius,0);

          std::cout<<rotVect<<std::endl;

          GMlib::Angle a = M_2PI/(innerNodes);
          GMlib::SqMatrix<float,2> matrRot (a, GMlib::Vector<float,2> (1,0), GMlib::Vector<float,2> (0,1));
          for (int j=1; j<=(innerNodes); j++)
          {
              rotVect =  matrRot * rotVect;
              this->insertAlways(GMlib::TSVertex<float> (static_cast<GMlib::Point<float,2>>(rotVect)));

              std::cout<<rotVect<<std::endl;
          }
          innerNodes += innerNodes;
      }

      this->triangulateDelaunay();

  }

protected:
  void localSimulate(double dt) override
  {

  }

private:

  GMlib::ArrayLX<Node> _nodes;
  GMlib::DMatrix<float> _A;
  GMlib::DVector<float> _b;


}; // END class Femobject



#endif // FEMOBJECT_H
