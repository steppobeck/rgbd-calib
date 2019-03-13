#ifndef ATTRIBUTEFINDER_H
#define ATTRIBUTEFINDER_H

#include <vector>
#include <iostream>



#if 0
INPUT: camera world position spos and nniSamples
OUTPUT: nniSamples with normals, where calculation was possible

std::vector<knobi::pointStruct> knobi_points;

for each P in 3D points
knobi::pointStruct p;
p._px = P[0];
p._py = P[1];
p._pz = P[2];
p._nx = <- calculate direction from point to spos as a guess
knobi_points.push_back(p);

if(knobi_points.size() > 500){
  Attributefinder a;
  a.makePointAttributes(knobi_points, 30 /*max. node cluster size*/, 40 /*max. tree depth*/, 20 /*k-nearest2search4 normal*/, false /*bool_flipnormals*/ );
  for each p in knobi_points, flip normal to spos
}

for each P in 3D points
  apply normal (from knobi or guess, depending on condition above)

END
#endif

namespace knobi{

struct pointStruct{
  //constructor
  pointStruct(float px=0.0, float py=0.0, float pz=0.0,
              float nx=0.0, float ny=0.0, float nz=0.0,
              float bx=0.0, float by=0.0, float bz=0.0,
              float tx=0.0, float ty=0.0, float tz=0.0):

              _px(px), _py(py), _pz(pz),
              _nx(nx), _ny(ny), _nz(nz),
              _bx(bx), _by(by), _bz(bz),
              _tx(tx), _ty(ty), _tz(tz), _valid(true){
  }



  //member
  float _px, _py, _pz;
  float _nx, _ny, _nz;
  float _bx, _by, _bz;
  float _tx, _ty, _tz;
  unsigned _valid;
};

struct nodeBounds{
    float minx_, maxx_; //bounds of the tree node
    float miny_, maxy_;
    float minz_, maxz_;
};

class Node{
  public:
    //constructor
    Node(void);

    Node(nodeBounds &bounds, unsigned int nodeid, unsigned int treelevel):
      bounds_(bounds), id_(nodeid), right_(NULL), left_(NULL), pointscount_(NULL),
      isleaf_(false), leafpoints_(NULL){ };

    ~Node();

    //member
    int splitaxis_;
    nodeBounds bounds_;
    int id_;
    Node* right_;
    Node* left_; //pointers to left and right tree nodes
    int pointscount_; //pointer to the set of 'pointscount' points contained in the leaf.
    bool isleaf_;
    pointStruct* leafpoints_;
};

class matrix{
    public:
        matrix(float a00=1, float a01=0, float a02=0,
               float a10=0, float a11=1, float a12=0,
               float a20=0, float a21=0, float a22=1){

               matrix_[0][0]= a00; matrix_[0][1]= a01; matrix_[0][2]= a02;
               matrix_[1][0]= a10; matrix_[1][1]= a11; matrix_[1][2]= a12;
               matrix_[2][0]= a20; matrix_[2][1]= a21; matrix_[2][2]= a22;
        }

        friend matrix operator+(const matrix& a, const matrix& b){
              return matrix(a.matrix_[0][0]+b.matrix_[0][0], a.matrix_[0][1]+b.matrix_[0][1], a.matrix_[0][2]+b.matrix_[0][2],
                            a.matrix_[1][0]+b.matrix_[1][0], a.matrix_[1][1]+b.matrix_[1][1], a.matrix_[1][2]+b.matrix_[1][2],
                            a.matrix_[2][0]+b.matrix_[2][0], a.matrix_[2][1]+b.matrix_[2][1], a.matrix_[2][2]+b.matrix_[2][2]);
        }

        friend matrix operator*(float skalar, const matrix& b){
              return matrix(skalar * b.matrix_[0][0], skalar * b.matrix_[0][1], skalar * b.matrix_[0][2],
                            skalar * b.matrix_[1][0], skalar * b.matrix_[1][1], skalar * b.matrix_[1][2],
                            skalar * b.matrix_[2][0], skalar * b.matrix_[2][1], skalar * b.matrix_[2][2]);
        }

        void printMatrix(void){
            std::cout<<"matrix: \n" <<matrix_[0][0] <<" "<<matrix_[0][1] <<" "<<matrix_[0][2] <<"\n"
                                    <<matrix_[1][0] <<" "<<matrix_[1][1] <<" "<<matrix_[1][2] <<"\n"
                                    <<matrix_[2][0] <<" "<<matrix_[2][1] <<" "<<matrix_[2][2] <<std::endl;
        }

        float getElement(int i, int j){
          return matrix_[i][j];
        }

        void setElement(int i, int j, float value){
          matrix_[i][j] = value;
        }


   private:
      float matrix_[3][3];


};

class double3{
  public:

    double3(float x=0, float y=0, float z=0):x_(x),y_(y),z_(z){};

    friend double3 operator + (const double3 &a, const double3 &b){  //Addition von Vektoren
      return double3(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_);
    }

    friend double3 operator * (const float &a, const double3 &b){  //Skalar * Vektor
      return double3(a * b.x_, a * b.y_, a * b.z_);
    }

    friend float operator * (const double3 &a, const double3 &b){  //Vektor * Vektor
      return float(a.x_ * b.x_ + a.y_ * b.y_ + a.z_ * b.z_);
    }


    friend double3 operator - (const double3 &a, const double3 &b){  //Subtraktion von Vektoren
      return double3(a.x_ - b.x_, a.y_ - b.y_,  a.z_ - b.z_);
    }

    friend double3 operator - (const double3 &a){                            //Vektor spiegeln
      return double3(-a.x_, -a.y_, -a.z_);
    }

    matrix tensorprodukt3x3(const double3 &a, const double3 &b){
      return matrix(a.x_ * b.x_, a.x_ * b.y_, a.x_ * b.z_,
                    a.y_ * b.x_, a.y_ * b.y_, a.y_ * b.z_,
                    a.z_ * b.x_, a.z_ * b.y_, a.z_ * b.z_);
    }

    int getLen(void){
      return 3;
    }

    float getData(int axis){
      float axis_value=0;

      switch(axis){
        case 0: axis_value = x_;
                break;
        case 1: axis_value = y_;
                break;
        case 2: axis_value = z_;
                break;
      }
      return axis_value;
    }

    void setData(int axis, float value){
      switch(axis){
        case 0: x_ = value;
                break;
        case 1: y_ = value;
                break;
        case 2: z_ = value;
                break;
      }
    }

    void printVektor(void){
      std::cout<<"x: " <<x_ <<"\ty: " <<y_ <<"\tz: " <<z_ <<std::endl;
    }

    float x_,y_,z_;

};


class Attributefinder{


  public:
    Attributefinder();
    ~Attributefinder();

    Node* makePointAttributes(std::vector<pointStruct>& pointvector, int clustersize /*max. node cluster size*/, int treedepth /*max. tree depth*/, int knearest2search4 /*k-nearest2search4*/, bool flipnormals);
    void printPoints2Terminal(std::vector<pointStruct>& pointvector);



  private:

    //build tree: functions
    nodeBounds makeBBox(std::vector<pointStruct>& pointvector);
    void buildTree(pointStruct* points, const int &pointscount, const int &maxpointspercell, Node* const treenode, const int &maxdepth, int treeheight = 1, int nodeindex = 0);
    //build tree: functions - inplace quicksort
    int rand_int(const int &l, const int &r);
    int partition_(pointStruct* points,const int &left, const int &right, const int &axis, const int &pivot);
    void quicksort(pointStruct* points,const int &left, const int &right, const int &axis);

    //build tree: member
    Node* rootnode;
    int leafesdone_;
    int maxtreedepth_;
    int clustersize_;

    //k-nearest functions
    unsigned search_k_neighbors(Node* const tree, const pointStruct &pos, pointStruct** const k_nearest_array, const int &max_k_nearest2search);
    double search_k_neighbors_(Node* const treenode, const pointStruct &pos, pointStruct** const k_nearest_array, const int &max_k_nearest2search, int &num_collected_k_nearest, double* const dist_in_leaf_arr, double max_dist2search);
    double attempt(pointStruct** const k_nearest_array, double* const dist_in_leaf_array, const int &max_k_nearest2search, pointStruct* const point2check, const double &dist_in_leaf, const double &max_dist2search, int &num_collected_k_nearest);
    double squaredDist(const Node* const treenode, const pointStruct &pos);

    //k-nearest member
    //int max_k_nearest2search_;

    //make noormal, binormal, tangent: functions
    void eigsrt_Jacobi(double3& d, matrix& V);
    void JacobiRotation(matrix m, int iterations,  long double maxfehler);
    double3 getEigenValues(void);
    double3 getEigenVector(unsigned int column);

    //make noormal, binormal, tangent: member
    double3 Eigenwerte_;
    matrix Eigenvektoren_;
    pointStruct** near_k_pointerarray_pointer_;
    pointStruct* pointsarraypointer_;


};

}
#endif // ATTRIBUTEFINDER_H
