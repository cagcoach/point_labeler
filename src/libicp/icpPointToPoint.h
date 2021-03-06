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

#ifndef ICP_POINT_TO_POINT_H
#define ICP_POINT_TO_POINT_H

#define NUMTHREADS 8

#include "icp.h"
#include "CTPL/ctpl.h"



class IcpPointToPoint : public Icp {

public:

  IcpPointToPoint (const double *M,const int32_t M_num,const int32_t dim,const float min_delta) : Icp(M,M_num,dim,min_delta) {}
  IcpPointToPoint (const double *M,const int32_t M_num,const int32_t dim) : Icp(M,M_num,dim) {}

  virtual ~IcpPointToPoint () {}

private:
  //ctpl::thread_pool p{NUMTHREADS};
  double fitStep (double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active);
  std::vector<int32_t> getInliers (const double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist);
  float getInliersSqDistance (const double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist);
  std::vector<bool> getWorldPtsInDistance (const double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist); 

};



#endif // ICP_POINT_TO_POINT_H
