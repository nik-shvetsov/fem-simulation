#ifndef FEMOBJECT_H
#define FEMOBJECT_H


#include "node.h"

class Femobject : public GMlib::TriangleFacets<float>
{
public:

  Femobject()
  {
      _force=1;
      _goUp=true;
      _maxForce=2;
  }
  ~Femobject() {}

  void randomTriangulation(double radius, int triangles)
  {
      double s = std::sqrt((4*M_PI*radius*radius)/(std::sqrt(3)*triangles));//length of side of a triangle
      int bordernodes = ((2*M_PI*radius)/s);
      int nodes = (triangles + 2 + bordernodes)/2;

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
      }

      //radius correction
      GMlib::Vector<float,2> pointVector(radius,0);
      auto point1 = GMlib::Point<float,2>(static_cast<GMlib::Point<float,2>>(pointVector));
      pointVector = matrRot*pointVector;
      auto point2 = GMlib::Point<float,2>(static_cast<GMlib::Point<float,2>>(pointVector));
      auto distanceVect = GMlib::Vector<float,2>(point1-point2);
      double distance = distanceVect.getLength();

      /*
      double b = 2*(radius)*std::sin(a.getRad()/2);
      double radCorr = std::sqrt((b*b)-(distance/2)*(distance/2));
      double newRadius = radius-(radCorr*0.3); //radCorr*2
      */

      double kat = cos(a.getRad()/2)*radius;
      double radCorr = radius-kat;
      double newRadius = radius-(radCorr*2);


      for (int i=1; i<=nodes; i++)

      {
          auto pt = GMlib::TSVertex<float> (RandomFloat(-newRadius,newRadius), RandomFloat(-newRadius,newRadius));
          if (pt.getParameter().getLength() < newRadius)
          {
              bool flag = true;
              for (int k=0; k<this->size();k++)
              {
                  if ((this->getElement(k).getParameter()-pt.getParameter()).getLength() < s/2)
                  flag=false;
              }
              if (flag==true)
              {
                  this->insertAlways(pt);
                  //std::cout<<pt<<std::endl;
              }

          }
          else
          {
              i--;
          }
      }

      this->triangulateDelaunay();
  }

  float RandomFloat(float a, float b)
  {
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

          //std::cout<<rotVect<<std::endl;

          GMlib::Angle a = M_2PI/(innerNodes*i);
          GMlib::SqMatrix<float,2> matrRot (a, GMlib::Vector<float,2> (1,0), GMlib::Vector<float,2> (0,1));
          for (int j=1; j<=(i*innerNodes); j++)
          {
              rotVect =  matrRot * rotVect;
              this->insertAlways(GMlib::TSVertex<float> (static_cast<GMlib::Point<float,2>>(rotVect)));

              //std::cout<<rotVect<<std::endl;
          }

      }

      this->triangulateDelaunay();

  }

  GMlib::Vector<GMlib::Vector<float,2>,3> findVectors(GMlib::TSEdge<float>* edge)
  {
      GMlib::Vector<GMlib::Vector<float,2>,3> colVector;

      auto triangles = edge->getTriangle(); //get 2 triangles from a common edge
      auto points1 = triangles[0]->getVertices(); //get 3 vertices from first triangle
      auto points2 = triangles[1]->getVertices(); //get 3 vertices from second

      GMlib::Point<float,2> p0,p1,p2,p3;
      p0 = edge->getFirstVertex()->getParameter();
      p1 = edge->getLastVertex()->getParameter();


      for (int i=0; i < 3; i++) //extract point 3 and 4
      {
          if (points1[i] != edge->getFirstVertex() && points1[i] != edge->getLastVertex())
          {
                p2 = points1[i]->getParameter();
          }

          if (points2[i] != edge->getFirstVertex() && points2[i] != edge->getLastVertex())
          {
                p3 = points2[i]->getParameter();
          }
      }

      colVector[0] = p1-p0;
      colVector[1] = p2-p0;
      colVector[2] = p3-p0;

      return colVector;
  }

  GMlib::Vector<GMlib::Vector<float,2>,3> findVectors(GMlib::TSTriangle<float>* tri, Node* node)
  {
      GMlib::Vector<GMlib::Vector<float,2>,3> colVector;
      auto points = tri->getVertices();
      GMlib::Point<float,2> p0,p1,p2;

          if (node->getMainVertex() == points[1])
          {
              std::swap(points[0],points[1]);
              std::swap(points[1],points[2]);
          }

          if (node->getMainVertex() == points[2])
          {
              std::swap(points[0],points[2]);
              std::swap(points[1],points[2]);
          }

      p0 = points[0]->getParameter();
      p1 = points[1]->getParameter();
      p2 = points[2]->getParameter();

      colVector[0] = p2-p0;
      colVector[1] = p1-p0;
      colVector[2] = p2-p1;

      return colVector;
  }



  void performComputations()
  {
      //this->triangulateDelaunay();
      for (int i=0; i<this->size(); i++)
      {
          //add vertices except boundary
          if (!((*this)[i].boundary()))
          {
            _nodes+=Node((*this)[i]);
            std::cout<<this->getVertex(i)<<std::endl;
          }
      }

      std::cout<<"Size of stiffness matrix A: "<<_nodes.size()<<std::endl;

      //preparing matrix A
      _A.setDim(_nodes.size(),_nodes.size());
      for (int i=0; i<_nodes.size(); i++)
      {
          for (int j=0; j<_nodes.size();j++)
          {
              _A[i][j]=0;
              std::cout<<_A[i][j]<<"  ";
          }
          std::cout<<std::endl;
      }

      //computing non-diagonal elements----------------------------------------


      //taking 2 nodes, check if they have common triangles
      for (int i=0; i<_nodes.size();i++)
      {
          for (int j=i+1; j<_nodes.size();j++)
          {
              //GMlib::Array<GMlib::TSTriangle<float>*> tr_1 = _nodes[i].getTriangles();
              //auto tr_1 = _nodes[i].getTriangles();
              //auto tr_2 = _nodes[j].getTriangles();

              //check if nodes have common edge
              //GMlib::Array<GMlib::TSEdge<float>*> edg_1 = _nodes[i].getEdges();
              auto edg_1 = _nodes[i].getEdges();
              auto edg_2 = _nodes[j].getEdges();
              GMlib::TSEdge<float>* commEdg;

              bool check = false;

              for (int k = 0; k < edg_1.size(); k++)
              {
                  for (int p = 0; p < edg_2.size(); p++)
                  {
                      if (edg_1[k]==edg_2[p])
                          check = true;
                          commEdg = edg_1[k];
                  }
              }

              if (check) //if 2 common triangles between vertices exist
              {
                  //compute
                  //_A[i][j] = _A[j][i] = 2;

                  auto colVector = findVectors(commEdg);

                  double dd = 1/(colVector[0]*colVector[0]);
                  double dh1 = dd * (colVector[1]*colVector[0]);
                  double area1 = std::abs(colVector[0]^colVector[1]);
                  double h1 = dd * area1 * area1;

                  double dh2 = dd * (colVector[2]*colVector[0]);
                  double area2 = std::abs(colVector[0]^colVector[2]);
                  double h2 = dd * area2 * area2;

                  //std::cout<<colVector[0]<<std::endl;
                  //std::cout<<colVector[1]<<std::endl;
                  //std::cout<<colVector[2]<<std::endl;


                  _A[i][j] = _A[j][i] = ((((dh1 * (1 - dh1))/h1) - dd) * (area1)/2) +
                                        ((((dh2 * (1 - dh2))/h2) - dd) * (area2)/2);
              }
          }
      }


      //computing diagonal elements
      for (int i=0; i<_nodes.size();i++)
      {
          GMlib::Array<GMlib::TSTriangle<float>*> triangles = _nodes[i].getTriangles();
          for (int k=0; k<triangles.size();k++)
          {
              auto colVector = findVectors(triangles[k], &_nodes[i]); //find 3 vectors with triangle and centre node
              //compute
              GMlib::Vector<float,2> d1,d2,d3;
              d1 = colVector[0];
              d2 = colVector[1];
              d3 = colVector[2];
              _A[i][i] += (d3*d3)/(2*(std::abs(d1^d2)));
          }
      }


    //get matrix a on screen
      std::cout<<std::endl;
      for (int i=0; i<_nodes.size(); i++)
      {
          for (int j=0; j<_nodes.size();j++)
          {
              std::cout<<_A[i][j]<<"  ";
          }
          std::cout<<std::endl;
      }

    //inverting matrix
      _A.invert();

      //computing _b
      _b.setDim(_nodes.size());

      for (int i = 0; i<_nodes.size(); i++)
      {
          _b[i]=0;
          auto triangles = _nodes[i].getTriangles();
          for (int j=0; j<triangles.size();j++)
          {
              _b[i]+=triangles[j]->getArea2D()/3;
              //std::cout<<triangles[j]->getArea2D()<<std::endl;
          }
          //std::cout<<std::endl;

      }


      //get b a on screen
        std::cout<<std::endl;
        for (int i=0; i<_nodes.size(); i++)
        {
                std::cout<<_b[i]<<std::endl;
        }
  }

  void updateFem(double force)
  {
      //std::cout<<_nodes.size()<<std::endl;

      GMlib::DVector<float> x = _A*(_b*force);
      //std::cout<<x.getDim()<<std::endl;
      for (int i=0; i<_nodes.size();i++)
      {
         _nodes[i].setZ(x[i]);
      }
  }

protected:
  void localSimulate(double dt) override
  {
      if(_goUp) _force += 2*dt;
      if (!_goUp) _force -= 2*dt;

      if (_force > _maxForce) _goUp = false;
      if (_force < -1*(_maxForce)) _goUp = true;

      updateFem(_force);

      this->replot();
  }

private:

  GMlib::ArrayLX<Node> _nodes;
  GMlib::DMatrix<float> _A;
  GMlib::DVector<float> _b;

  bool _goUp;
  float _force;
  float _maxForce;


}; // END class Femobject



#endif // FEMOBJECT_H
