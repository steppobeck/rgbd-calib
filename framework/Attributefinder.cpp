#include "Attributefinder.hpp"

#include <cmath>
#include <cstdlib>

namespace knobi{

Attributefinder::Attributefinder(){
  //ctor
  leafesdone_=0;
  maxtreedepth_=0;
  clustersize_=0;
}

Attributefinder::~Attributefinder(){
  //dtor
  delete pointsarraypointer_;
  pointsarraypointer_ = NULL;
  delete near_k_pointerarray_pointer_;
  near_k_pointerarray_pointer_ = NULL;
//  delete rootnode;
//  rootnode = NULL;

}

namespace{

  float distance(pointStruct& a, pointStruct&b){
    return sqrt((a._px - b._px)*(a._px - b._px) +
		(a._py - b._py)*(a._py - b._py) +
		(a._pz - b._pz)*(a._pz - b._pz));
  }

}

Node* //einzige aufzurufende methode außerhalb der klasse
Attributefinder::makePointAttributes(std::vector<pointStruct>& pointvector, int clustersize /*max. node cluster size*/, int treedepth /*max. tree depth*/, int knearest2search4 /*k-nearest2search4*/, bool flipnormals){

  //create bbox
  nodeBounds boundBox = makeBBox(pointvector);

  //create rootnode of the tree
  rootnode = new Node(boundBox, 0 /*node id*/, 0/*node level*/);

  //workaround: der baum selbst wird mit Hilfe eines pointStruct array aufgebaut
  //habs mit vektoren versucht, musste aber nach 3 Stunden aufgeben damit einen baum zu bauen. hab's einfach nicht hinbekommen
  int pointcount = pointvector.size();
  pointsarraypointer_ = new pointStruct[pointcount];
  for(int i=0; i<pointcount; i++){
    pointsarraypointer_[i] = pointvector[i];
  }

  //create kd-tree
  buildTree(pointsarraypointer_, pointcount, clustersize /*max.clustersize*/, rootnode, treedepth /*max.treedepth*/, 1, rootnode->id_);
  Node* tree = rootnode; //nach der buildTree funktion enthält rootnode nun den gesamten kd-baum. tree dient nur der einfacheren verständlichkeit

  //  near_k ist ein pointer auf ein nächste-nachbarn-array das wiederum pointer auf die gefundenen k-nächsten nachbarn hält
  near_k_pointerarray_pointer_ = new pointStruct*[knearest2search4];

   //  finde die k-nächsten nachbarn für jeden punkt und berechnen für jeden punkt im pointvektor die attribute normal, binormale und tangente
  for(int i=0; i<pointcount; i++){

      pointStruct point2makeAttributes = pointvector[i];
      search_k_neighbors(tree,point2makeAttributes,near_k_pointerarray_pointer_,knearest2search4);

      pointStruct schwerpunkt;

      float summed_distance = 0.0;
      float max_distance = 0.0;
      for(int j = 0; j<knearest2search4; j++){

          schwerpunkt._px = schwerpunkt._px + near_k_pointerarray_pointer_[j]->_px;
          schwerpunkt._py = schwerpunkt._py + near_k_pointerarray_pointer_[j]->_py;
          schwerpunkt._pz = schwerpunkt._pz + near_k_pointerarray_pointer_[j]->_pz;

	  // steppo
	  float d = distance(point2makeAttributes, *(near_k_pointerarray_pointer_[j]));
	  summed_distance += d;
	  max_distance = std::max(max_distance, d);

      }

      //if(summed_distance/knearest2search4 > 0.05)
      if(max_distance > 0.01/*0.01*/){
	pointvector[i]._valid = false;
	//std::cerr << max_distance << std::endl;
	continue;
      }


      schwerpunkt._px = schwerpunkt._px / knearest2search4;
      schwerpunkt._py = schwerpunkt._py / knearest2search4;
      schwerpunkt._pz = schwerpunkt._pz / knearest2search4;

      //berechne die Summe der kovarianzmatrizen für alle k-nächsten-nachbar vektoren
      matrix covarianzmatrix, summed_covarianz;
      for(int k = 0; k<knearest2search4; k++){

          double3 punkt_i(near_k_pointerarray_pointer_[k]->_px,near_k_pointerarray_pointer_[k]->_py,near_k_pointerarray_pointer_[k]->_pz);
          double3 p_schwer(schwerpunkt._px,schwerpunkt._py,schwerpunkt._pz) ;

          double3 vektor_pi_pschwer = punkt_i - p_schwer;
          double3 vektor_pi_pschwer_transp = punkt_i - p_schwer; //nicht wirklich transponiert aber der besseren lesbarkeit halber ... dem rechner ist die darstellung eines vektor in transponierter schreibweise wurscht

          //berechnen die covraianzmatrix
          covarianzmatrix = vektor_pi_pschwer.tensorprodukt3x3(vektor_pi_pschwer, vektor_pi_pschwer_transp);

          //wichte die covarinazmatrix durch skalarmultiplikation mit der euklidischen distanz
          //experimentell und irgendwie nicht so gut im ergebnis - nochmal durchdenken
          //Normalen ohne wichtung "sauberer"
//           covarianzmatrix = pow(2.718281828459045235, -1*(
//                                                          (punkt_i.x_ * point2makeAttributes._px - punkt_i.y_ * point2makeAttributes._py - punkt_i.z_ * point2makeAttributes._pz ) /
//                                                          (0.33 * (punkt_i.x_ * point2makeAttributes._px + punkt_i.y_ * point2makeAttributes._py + punkt_i.z_ * point2makeAttributes._pz ))
//                                                          )
//                                 ) * covarianzmatrix;

          if(k==0){
              summed_covarianz = covarianzmatrix;
          }
          else{
              summed_covarianz = summed_covarianz + covarianzmatrix;
          }
      }

      // löse das gleichungssystem
      JacobiRotation(summed_covarianz, 10/*max. iterationsschritte*/, 0.00000001/*maximal erlaubter fehler*/);

      // weise die eigenvektoren (normale, binormale und tangente) den punkten im vektor zu
      pointvector[i]._nx = Eigenvektoren_.getElement(0,0);  pointvector[i]._bx = Eigenvektoren_.getElement(0,1);  pointvector[i]._tx = Eigenvektoren_.getElement(0,2);
      pointvector[i]._ny = Eigenvektoren_.getElement(1,0);  pointvector[i]._by = Eigenvektoren_.getElement(1,1);  pointvector[i]._ty = Eigenvektoren_.getElement(1,2);
      pointvector[i]._nz = Eigenvektoren_.getElement(2,0);  pointvector[i]._bz = Eigenvektoren_.getElement(2,1);  pointvector[i]._tz = Eigenvektoren_.getElement(2,2);



      double3 normalVec, dir2originVec;
        // flippe die normale , wenn sie vom ursprung wegzeigt
      dir2originVec.x_ = 0.0 - pointvector[i]._px;
      dir2originVec.y_ = 0.0 - pointvector[i]._py;
      dir2originVec.z_ = 0.0 - pointvector[i]._pz;

      // flippe die normale , wenn sie vom schwerpunkt aller nächsten nachbarn wegzeigt
//      dir2originVec.x_ = pointvector[i]._px - schwerpunkt._px;
//      dir2originVec.y_ = pointvector[i]._py - schwerpunkt._py;
//      dir2originVec.z_ = pointvector[i]._pz - schwerpunkt._pz;

      normalVec.x_ = pointvector[i]._nx;
      normalVec.y_ = pointvector[i]._ny;
      normalVec.z_ = pointvector[i]._nz;

      if( normalVec * dir2originVec < 0.0  && flipnormals){
//            std::cout<<"Normal flipped"<<std::endl;
           pointvector[i]._nx =  -pointvector[i]._nx; pointvector[i]._bx =  -pointvector[i]._bx; pointvector[i]._tx =  -pointvector[i]._tx;
           pointvector[i]._ny =  -pointvector[i]._ny; pointvector[i]._by =  -pointvector[i]._by; pointvector[i]._ty =  -pointvector[i]._ty;
           pointvector[i]._nz =  -pointvector[i]._nz; pointvector[i]._bz =  -pointvector[i]._bz; pointvector[i]._tz =  -pointvector[i]._tz;
      }

  }

  return tree;
}



void
Attributefinder::printPoints2Terminal(std::vector<pointStruct>& pointvector){

  for(unsigned int i=0; i<pointvector.size(); ++i){
     std::cout <<"point["<<i <<"]._px = " <<pointvector[i]._px   <<"\tpoint["<<i <<"]._py = " <<pointvector[i]._py   <<"\tpoint["<<i <<"]._pz = " <<pointvector[i]._pz  <<std::endl;
  }

}





//// -- Quicksort entlang einer vorgegebenen Achse ----////
int // finde den median entlang der übergebenen achse
Attributefinder::partition_(pointStruct* points,const int &left, const int &right, const int &axis, const int &pivot){

  float pivotvalue;
  if (axis==0){
    pivotvalue = points[pivot]._px;
  }
  else if (axis==1){
    pivotvalue = points[pivot]._py;
  }
  else{
    pivotvalue = points[pivot]._pz;
  }

  pointStruct temp = points[right];
  points[right] = points[pivot];
  points[pivot] = temp;

  int splitindex = left;
  float temphelper;

  for(int i=left; i<right; i++){
    if(axis==0){
      temphelper = points[i]._px;
    }
    else if(axis==1){
      temphelper = points[i]._py;
    }
    else{
      temphelper = points[i]._pz;
    }

    if(temphelper < pivotvalue){
      temp = points[i];
      points[i] = points[splitindex];
      points[splitindex] = temp;
      splitindex++;
    }
  }

  temp  = points[splitindex];
  points[splitindex] = points[right];
  points[right] = temp;
  return splitindex;
}

int
Attributefinder::rand_int(const int &l, const int &r){
  return rand() % (r-l)+l;
}

void // sortiert entlang der entsprechend übergebenen achse entweder nach x, y oder z
Attributefinder::quicksort(pointStruct* points ,const int &left, const int &right, const int &axis){
  if(right > left){
    int pivot = rand_int(left, right);
    int newpivot = partition_(points, left, right, axis, pivot);
    quicksort(points, left, newpivot-1, axis); //recursive call for left part of the array
    quicksort(points, newpivot+1, right, axis); //recursive call for the right part of the array
  }
}
//// --ENDE -- Quicksort entlang einer vorgegebenen Achse ----////





//// -- erstelle den kd-Baum -----------------------------------------////
void
Attributefinder::buildTree(pointStruct* points, const int &pointscount, const int &maxpointspercell, Node* const treenode, const int &maxdepth, int treeheight, int nodeindex){

  //breche die recursion ab, wenn die maximale baumtiefe erreicht ist oder, wenn die geforderte clustersize erreicht bzw. unterschritten ist
  if(treeheight >= maxdepth || pointscount <= maxpointspercell){
    treenode->leafpoints_ = points;
    treenode->pointscount_ = pointscount;
    treenode->isleaf_ = true;
    leafesdone_++;
    if(clustersize_<pointscount) clustersize_= pointscount;
		return;
  }

  if(maxtreedepth_<=treeheight){
    maxtreedepth_=treeheight+1;
  }

  // schreibe build-status auf die konsole
  //int total_leafes2create = powf(2.0, maxtreedepth_-1);
  //int percent=100*((float)(leafesdone_)/total_leafes2create);
  //std::cout <<"\033[A\033[K\n\033[K\r\033[A" <<std::flush; //clear last 2 lines of the console
  //std::cout <<"Building tree, please wait... " <<leafesdone_ <<" leaves are created. \n"<< percent <<"% of the structure are done. Tree depth: "
  //          <<maxtreedepth_  <<" Deepest tree level: " <<maxtreedepth_-1 <<". " <<(powf(2, maxtreedepth_))-1 <<" nodes in total."
  //          <<"\t clustersize is " <<clustersize_
  //          <<std::flush;


  // 1. Step: split along the axis of who the tree's bound is largest
  if( (treenode->bounds_.maxx_ - treenode->bounds_.minx_) > (treenode->bounds_.maxy_ - treenode->bounds_.miny_) ){
    treenode->splitaxis_ = 0; //for x-axis
  }
  else if( (treenode->bounds_.maxy_ - treenode->bounds_.miny_) > (treenode->bounds_.maxz_ - treenode->bounds_.minz_) ){
    treenode->splitaxis_ = 1; //for y-axis
  }
  else{
    treenode->splitaxis_ = 2; //for z-axis
  }



  // 2. Step: sort points (for balancing the tree) along the largest bounds_ axis found in step 1
  quicksort(points, 0, pointscount-1, treenode->splitaxis_);

  int numleftpoints = pointscount>>1; //bitshift right halfs pointscount
  int numrightpoins = pointscount - numleftpoints;


  // 3. Step: create child nodes for the found split axis
  if(treenode->splitaxis_ == 0){
      float partition_x;

      if(numleftpoints != numrightpoins){ //if differs, choose most right points axis value to partition the axis
          partition_x = points[numleftpoints]._px;
      }
      else{ //if it not differs, calculate a new value for the axis between the last 2 most right points and take it as partition value
          partition_x = (points[numleftpoints-1]._px + points[numleftpoints]._px)*0.5;
      }

      //create left child node and set most right partition value along x-axis
      treenode->left_ = new Node(treenode->bounds_, ((nodeindex*2)+1), treeheight);  treenode->left_->bounds_.maxx_ = partition_x; treenode->isleaf_ = false;
       //create right child node and set most left partition value along x-axis
      treenode->right_ = new Node(treenode->bounds_, ((nodeindex*2)+2), treeheight ); treenode->right_->bounds_.minx_ = partition_x; treenode->isleaf_ = false;

  }
  else if(treenode->splitaxis_ == 1){
      float partition_y;

      if(numleftpoints != numrightpoins){
          partition_y = points[numleftpoints]._py;
      }
      else{
          partition_y = (points[numleftpoints-1]._py + points[numleftpoints]._py)*0.5;
      }

      treenode->left_ = new Node(treenode->bounds_, ((nodeindex*2)+1), treeheight);   treenode->left_->bounds_.maxy_ = partition_y; treenode->isleaf_ = false;
      treenode->right_ = new Node(treenode->bounds_, ((nodeindex*2)+2), treeheight);  treenode->right_->bounds_.miny_ = partition_y; treenode->isleaf_ = false;
  }
  else{
      float partition_z;

      if(numleftpoints != numrightpoins){
          partition_z = points[numleftpoints]._pz;
      }
      else{
          partition_z = (points[numleftpoints-1]._pz + points[numleftpoints]._pz)*0.5;
      }

      treenode->left_ = new Node(treenode->bounds_, ((nodeindex*2)+1), treeheight);  treenode->left_->bounds_.maxz_ = partition_z; treenode->isleaf_ = false;
      treenode->right_ = new Node(treenode->bounds_, ((nodeindex*2)+2), treeheight); treenode->right_->bounds_.minz_ = partition_z; treenode->isleaf_ = false;
  }

  // 4.Step: create recursive sub-trees
  buildTree(points, numleftpoints, maxpointspercell, treenode->left_, maxdepth, treeheight+1, ((nodeindex*2)+1) );
  buildTree( (points+numleftpoints), numrightpoins, maxpointspercell, treenode->right_, maxdepth, treeheight+1, ((nodeindex*2)+2) );
}
//// --ENDE-- erstelle den kd-Baum -----------------------------------------////





//// -- berechnen die Boundbox die alle punkte umfasst----------------------////
nodeBounds
Attributefinder::makeBBox(std::vector<pointStruct>& pointvector){

  nodeBounds boundBox;

  for(unsigned int i=0; i<pointvector.size(); ++i){
      if(i==0){
          boundBox.minx_ = pointvector[i]._px;   boundBox.miny_ = pointvector[i]._py;   boundBox.minz_ = pointvector[i]._pz;
          boundBox.maxx_ = pointvector[i]._px;   boundBox.maxy_ = pointvector[i]._py;   boundBox.maxz_ = pointvector[i]._pz;
      }
      //get lowest and highest axis values to create the bounding box about the point cloud
      if(boundBox.minx_ > pointvector[i]._px) boundBox.minx_ = pointvector[i]._px;
      if(boundBox.miny_ > pointvector[i]._py) boundBox.miny_ = pointvector[i]._py;
      if(boundBox.minz_ > pointvector[i]._pz) boundBox.minz_ = pointvector[i]._pz;
      if(boundBox.maxx_ < pointvector[i]._px) boundBox.maxx_ = pointvector[i]._px;
      if(boundBox.maxy_ < pointvector[i]._py) boundBox.maxy_ = pointvector[i]._py;
      if(boundBox.maxz_ < pointvector[i]._pz) boundBox.maxz_ = pointvector[i]._pz;
  }

  //std::cout <<"\nboundingBox.minx_ = " <<boundBox.minx_ <<"\tboundingBox.miny_ = " <<boundBox.miny_ <<"\tboundingBox.minz_ = " <<boundBox.minz_
  //          <<"\nboundingBox.maxx_ = " <<boundBox.maxx_ <<"\tboundingBox.maxy_ = " <<boundBox.maxy_ <<"\tboundingBox.maxz_ = " <<boundBox.maxz_
  //          <<"\n\n"
  //          <<std::endl;

  return boundBox;
}
//// -- ENDE -- berechnen die Boundbox des rootknoten ------------------------------////




//// -- berechnen die k-nächsten Nachbarn ----------- ------------------------------////
unsigned
Attributefinder::search_k_neighbors(Node* const tree, const pointStruct &pos, pointStruct** const k_nearest_array, const int &max_k_nearest2search){
  //method to be used
  double* dist_in_leaf_array = new double[max_k_nearest2search];
  int num_collected_k_nearest = 0;
  double max_dist2search = 1e+250;
  double md = search_k_neighbors_(tree, pos, k_nearest_array, max_k_nearest2search, num_collected_k_nearest, dist_in_leaf_array, max_dist2search);
  //std::cerr << md << std::endl;
  delete[] dist_in_leaf_array;
}

double
Attributefinder::search_k_neighbors_(Node* const treenode, const pointStruct &pos, pointStruct** const k_nearest_array, const int &max_k_nearest2search, int &num_collected_k_nearest, double* const dist_in_leaf_arr, double max_dist2search){

	if(treenode->isleaf_==true){
	  //leaf node
		const float &x = pos._px;
		const float &y = pos._py;
		const float &z = pos._pz;

		for(int i = 0; i<treenode->pointscount_; i++){
		  //attempt to add all the points contained in the leaf node
		  //calculate squared distance to each point
			pointStruct* point2check = treenode->leafpoints_+i;


      if(point2check->_px == x && point2check->_py == y && point2check->_pz == z){
          //std::cout<<"\nfound myself, continue with next point" <<std::endl;
          continue;
			}

			float dist_x = point2check->_px-x;
			float dist_y = point2check->_py-y;
			float dist_z = point2check->_pz-z;
			float dist_in_leaf = dist_x*dist_x+dist_y*dist_y+dist_z*dist_z;

			max_dist2search = attempt(k_nearest_array, dist_in_leaf_arr, max_k_nearest2search, point2check, dist_in_leaf, max_dist2search, num_collected_k_nearest);

		}
	}
	else{
		float dist_a = squaredDist(treenode->left_,pos);
		float dist_b = squaredDist(treenode->right_,pos);
		if(dist_a<dist_b){
		  //search left first
			if(dist_a<=max_dist2search || num_collected_k_nearest<max_k_nearest2search)
				max_dist2search = search_k_neighbors_(treenode->left_,pos,k_nearest_array,max_k_nearest2search,num_collected_k_nearest,dist_in_leaf_arr,max_dist2search);
			if(dist_b<=max_dist2search || num_collected_k_nearest<max_k_nearest2search)
				max_dist2search = search_k_neighbors_(treenode->right_,pos,k_nearest_array,max_k_nearest2search,num_collected_k_nearest,dist_in_leaf_arr,max_dist2search);
		}
		else{
		  //search right first
			if(dist_b<=max_dist2search || num_collected_k_nearest<max_k_nearest2search)
				max_dist2search = search_k_neighbors_(treenode->right_,pos,k_nearest_array,max_k_nearest2search,num_collected_k_nearest,dist_in_leaf_arr,max_dist2search);
			if(dist_a<=max_dist2search || num_collected_k_nearest<max_k_nearest2search)
				max_dist2search = search_k_neighbors_(treenode->left_,pos,k_nearest_array,max_k_nearest2search,num_collected_k_nearest,dist_in_leaf_arr,max_dist2search);
		}
	}

	return max_dist2search;
}


double
Attributefinder::attempt(pointStruct** const k_nearest_array, double* const dist_in_leaf_array, const int &max_k_nearest2search, pointStruct* const point2check, const double &dist_in_leaf, const double &max_dist2search, int &num_collected_k_nearest){


  //attempt to add data point point2check into the current k_nearest_array
  if(num_collected_k_nearest < max_k_nearest2search){

      //max number has not been reached, add next point to check
      k_nearest_array[num_collected_k_nearest] = point2check;
      dist_in_leaf_array[num_collected_k_nearest] = dist_in_leaf;
      num_collected_k_nearest++;

      if(dist_in_leaf > max_dist2search || num_collected_k_nearest==1){
      //  std::cout<<"dist_in_leaf: " <<dist_in_leaf <<"\n";
          return dist_in_leaf;
      }

      //	std::cout<<"max_dist2search: " <<max_dist2search <<"\n";
      return max_dist2search;

	}

  //if desired number of max_k_nearest is once reached check if point is further away than allowed by user
  //if it is more than max_dist2search, can break here
  if(dist_in_leaf > max_dist2search){
      return max_dist2search;
  }

  pointStruct** current_point = NULL;
	double* current_point_distance = NULL;
	double currentvalue = 0;
	double currentvalue2 = 0;

  // find point furthest from tagret and distance to second furthest
	for(long i = 0; i<max_k_nearest2search; i++){

      pointStruct** target_point = k_nearest_array + i;
      double* dist_in_leaf2 = dist_in_leaf_array + i;
      double value_dist_leaf2 = *dist_in_leaf2;

      if(value_dist_leaf2>currentvalue){
        currentvalue = value_dist_leaf2;
        current_point_distance = dist_in_leaf2;
        current_point = target_point;
      }
      if(value_dist_leaf2>currentvalue2 && value_dist_leaf2<currentvalue)
        currentvalue2 = value_dist_leaf2;
	}

	//replace data point in the k_nearest_array, and adjust current max distance int the dist_in_leaf_array
	*current_point = point2check;
	*current_point_distance = dist_in_leaf;

	if(dist_in_leaf>currentvalue2){
	    return dist_in_leaf;
	}

  return currentvalue2;
}


double //helper method - returns squared distance to the node's bounds_ (0 if inside)
Attributefinder::squaredDist(const Node* const treenode, const pointStruct &pos){


  if(  pos._px > treenode->bounds_.minx_
    && pos._px < treenode->bounds_.maxx_
    && pos._py > treenode->bounds_.miny_
    && pos._py < treenode->bounds_.maxy_
    && pos._pz > treenode->bounds_.minz_
    && pos._pz < treenode->bounds_.maxz_){
    return 0.0;
  }

  double dist_x = 0.0;
  double dist_y = 0.0;
  double dist_z = 0.0;

  if(pos._px > treenode->bounds_.maxx_){
    dist_x = pos._px - treenode->bounds_.maxx_;

    if(pos._py < treenode->bounds_.miny_){
      dist_y = pos._py - treenode->bounds_.miny_;
    }
    else if (pos._py > treenode->bounds_.maxy_){
      dist_y = pos._py- treenode->bounds_.maxy_;
    }

    if(pos._pz < treenode->bounds_.minz_){
      dist_z = pos._pz - treenode->bounds_.minz_;
    }
    else if (pos._pz > treenode->bounds_.maxz_){
      dist_z = pos._pz - treenode->bounds_.maxz_;
    }
  }

  else if(pos._px < treenode->bounds_.minx_){
    dist_x = pos._px - treenode->bounds_.minx_;

    if(pos._py < treenode->bounds_.miny_){
      dist_y = pos._py - treenode->bounds_.miny_;
    }
    else if(pos._py > treenode->bounds_.maxy_){
      dist_y = pos._py - treenode->bounds_.maxy_;
    }

    if(pos._pz < treenode->bounds_.minz_){
      dist_z = pos._pz - treenode->bounds_.minz_;
    }
    else if(pos._pz > treenode->bounds_.maxz_){
      dist_z = pos._pz - treenode->bounds_.maxz_;
    }
  }

   else if(pos._pz < treenode->bounds_.minz_){
    dist_z = pos._pz - treenode->bounds_.minz_;

    if(pos._py < treenode->bounds_.minz_){
      dist_y = pos._py - treenode->bounds_.minz_;
    }
    else if(pos._py > treenode->bounds_.maxy_){
      dist_y = pos._py - treenode->bounds_.maxy_;
    }
  }

  else if(pos._pz > treenode->bounds_.maxz_){
    dist_z = pos._pz - treenode->bounds_.maxz_;

    if(pos._py < treenode->bounds_.miny_){
      dist_y = pos._py - treenode->bounds_.miny_;
    }
    else if(pos._py > treenode->bounds_.maxy_){
      dist_y = pos._py - treenode->bounds_.maxy_;
    }
  }

  else {
    if(pos._py < treenode->bounds_.miny_){
      dist_y = pos._py - treenode->bounds_.miny_;
    }
    else if(pos._py > treenode->bounds_.maxy_){
      dist_y = pos._py - treenode->bounds_.maxy_;
    }
  }

  return dist_x*dist_x + dist_y*dist_y + dist_z*dist_z;
}
//// -- ENDE -- berechnen die k-nächsten Nachbarn ----------------------------------------////









//// ------------ Jacobi Gleichungssystem Lösung ----------------------------------------////

void
Attributefinder::eigsrt_Jacobi(double3& d, matrix& V){

  int k, j, i;
  double p;
  int n = 3;

  for (i=0; i<n; i++)  {
    k = i;
    p = d.getData(k);               // p = d(k)
    for (j=i+1; j<n; j++) {
      if (d.getData(j) < p) {    // d(j) <= p
        k = j;
        p = d.getData(k);       // p = d(k)
      }
    }   // end for j = i+1 ...
    if (k != i)  {
      d.setData(k,d.getData(i));   // d(k) = d(i)
      d.setData(i,p);                // d(i) = p
      for (j=0; j<n; j++)  {
        p = V.getElement(j,i);                  // p = V(j,i)
        V.setElement(j,i,V.getElement(j,k));    // V(j,i) = V(j,k)
        V.setElement(j,k,p);                    // V(j,k) = p
      }
    }  // end for k != i
  }   // end for i = 0 ...

}

void
Attributefinder::JacobiRotation(matrix m,int iterations, long double maxfehler){


        int iNum = 3; //anzahl der zeilen & spalten der matrix
        int nMaxIt = iterations; //maximum iterations schritte
        long double eps = maxfehler; //max. zulässiger fehler

        int p,q,l;
        long double cn,sn,omega,x,y,diff;

        double iR1[3][3] = {  {m.getElement(0,0),m.getElement(0,1),m.getElement(0,2)},
                              {m.getElement(1,0),m.getElement(1,1),m.getElement(1,2)},
                              {m.getElement(2,0),m.getElement(2,1),m.getElement(2,2)}
                            };


        for (int i=0; i <= iNum-1; i++){

            Eigenvektoren_.setElement(i,i,1.0);

            for (int j = 0; j <= iNum-1; j++){

                if (i != j) Eigenvektoren_.setElement(i,j,0.0);
            }
        }

        l = 1;
        while (l < nMaxIt){

              double maxdiff=0.0;
              for (int i = 0; i < iNum-1 ; i++){

                  for (int j = i+1; j < iNum ; j++){

                      diff=fabs(iR1[i][j]);
                      if ((i != j)&&(diff >= maxdiff)){

                          maxdiff = diff;
                          p = i;
                          q = j;

                      }
                  }
              }

              if (maxdiff < eps){

                  for (int i = 0; i < iNum; ++i){
                    Eigenwerte_.setData(i, iR1[i][i]);
                  }

                  break;
              }

              x=-iR1[p][q];
              y=(iR1[q][q]-iR1[p][p]) / 2.0;
              omega=x / sqrt(x * x + y * y);

              if (y<0.0) omega = -omega;

              sn=1.0+sqrt(1.0-omega*omega);
              sn=omega / sqrt(2.0 * sn);
              cn=sqrt(1.0-sn*sn);

              maxdiff=iR1[p][p];

              iR1[p][p]=maxdiff*cn*cn+iR1[q][q]*sn*sn+iR1[p][q]*omega;
              iR1[q][q]=maxdiff*sn*sn+iR1[q][q]*cn*cn-iR1[p][q]*omega;
              iR1[p][q]=0.0;
              iR1[q][p]=0.0;


              for (int j=0; j<=iNum-1; j++){
                  if ((j!=p)&&(j!=q)){
                      maxdiff=iR1[p][j];
                      iR1[p][j]=maxdiff*cn+iR1[q][j]*sn;
                      iR1[q][j]=-maxdiff*sn+iR1[q][j]*cn;
                  }

              }

              for (int i=0; i<=iNum-1; i++){
                  if ((i!=p)&&(i!=q)){
                      maxdiff=iR1[i][p];
                      iR1[i][p]=maxdiff*cn+iR1[i][q]*sn;
                      iR1[i][q]=-maxdiff*sn+iR1[i][q]*cn;
                  }
              }

              for (int i = 0; i <= iNum-1; i++){
                  maxdiff=Eigenvektoren_.getElement(i,p);
                  Eigenvektoren_.setElement(i,p, maxdiff*cn+Eigenvektoren_.getElement(i,q)*sn);
                  Eigenvektoren_.setElement(i,q, -maxdiff*sn+Eigenvektoren_.getElement(i,q)*cn);
              }

              l++;
        }
//        std::cout<<"\n"<<l <<" sweeps needed to reduce error lower than " <<eps <<std::endl;
        eigsrt_Jacobi(Eigenwerte_, Eigenvektoren_);
}




}

