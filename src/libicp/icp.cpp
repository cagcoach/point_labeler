/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "icp.h"

using namespace std;

Icp::Icp (const double *M,const int32_t M_num,const int32_t dim, const float min_delta) :
  dim(dim), max_iter(100), min_delta(min_delta) {
  
  // check for correct dimensionality
  if (dim!=2 && dim!=3) {
    cout << "ERROR: LIBICP works only for data of dimensionality 2 or 3" << endl;
    M_tree = 0;
    return;
  }
  
  // check for minimum number of points
  if (M_num<5) {
    cout << "ERROR: LIBICP works only with at least 5 model points" << endl;
    M_tree = 0;
    return;
  }

  // copy model points to M_data
  M_data.resize(boost::extents[M_num][dim]);
  for (int32_t m=0; m<M_num; m++)
    for (int32_t n=0; n<dim; n++){
      //cout << m <<" "<< n << endl;
      M_data[m][n] = (float)M[m*dim+n];
    }

  // build a kd tree from the model point cloud
  M_tree = new kdtree::KDTree(M_data);
}

Icp::~Icp () {
  if (M_tree)
    delete M_tree;
}

void Icp::fit (double *T,const int32_t T_num,Matrix &R,Matrix &t,const double indist) {
  
  // make sure we have a model tree
  if (!M_tree) {
    cout << "ERROR: No model available." << endl;
    return;
  }
  
  // check for minimum number of points
  if (T_num<5) {
    cout << "ERROR: Icp works only with at least 5 template points" << endl;
    return;
  }
  
  // set active points
  vector<int32_t> active;
  if (indist<=0) {
    active.clear();
    for (int32_t i=0; i<T_num; i++)
      active.push_back(i);
  } else {
    active = getInliers(T,T_num,R,t,indist);
  }
  // run icp
  fitIterate(T,T_num,R,t,active);
}

uint32_t Icp::getInlierSize (const double *T,const int32_t T_num,Matrix &R,Matrix &t,const double indist) {
  
  // make sure we have a model tree
  if (!M_tree) {
    cout << "ERROR: No model available." << endl;
    return 0;
  }
  
  // check for minimum number of points
  if (T_num<5) {
    cout << "ERROR: Icp works only with at least 5 template points" << endl;
    return 0;
  }
  
  // set active points
  vector<int32_t> a;
  if (indist<=0) {
    a.clear();
    for (int32_t i=0; i<T_num; i++)
      a.push_back(i);
  } else {
    a = getInliers(T,T_num,R,t,indist);
  }
  return a.size();
}


float Icp::getSqDistance (const double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist){
  return getInliersSqDistance(T,T_num,R,t,indist);
}

std::vector<bool> Icp::getInDistance (const double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist){
  return getWorldPtsInDistance(T,T_num,R,t,indist);
}

void Icp::fitIterate(double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active) {
  
  // check if we have at least 5 active points
  //std::cout<<"Active "<<active.size()<<std::endl;
  if (active.size()<5)
    return;
  
  // iterate until convergence
  double delta;
  int cnt = 0;
  for (int32_t iter=0; iter<max_iter; iter++){
    delta=fitStep(T,T_num,R,t,active);
      if (delta<min_delta)
        break;
  }
  //std::cout<<"Delta "<<delta<<std::endl;
}
