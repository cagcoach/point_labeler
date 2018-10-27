#include "AutoAuto.h"



#include <dirent.h>
#include <string>
#include <fstream>
#include "libicp/icpPointToPoint.h"
#include "libicp/matrix.h"
#include <QDebug>
#include <climits>
#include "base64.h"
#include <string>
#include <ctime>
#include <random>
#include <boost/algorithm/string.hpp>



using namespace glow;

AutoAuto::AutoAuto(const std::vector<std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>>> cars_):cars(cars_){
  connect(this, SIGNAL(carFinished()),this,SLOT(manageResults()));

}

AutoAuto::AutoAuto(const std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> cars_, std::string config){
	std::cout<<config.substr(0,100)<<std::endl;
  int type_;
  cars.push_back(cars_);
  std::istringstream config_stream(config);
  config_stream >> type_;
  switch(type_){
    case Car::STATIC_CAR:{
      Eigen::Matrix4f pose_;
      Eigen::Vector4f dir_;
      std::string ps;
      std::string model;
      config_stream >>
        model >>
        pose_(0,0) >>
        pose_(1,0) >>
        pose_(2,0) >>
        pose_(0,1) >>
        pose_(1,1) >>
        pose_(2,1) >>
        pose_(0,2) >>
        pose_(1,2) >>
        pose_(2,2) >>
        pose_(0,3) >>
        pose_(1,3) >>
        pose_(2,3) >>
        dir_(0) >>
        dir_(1) >>
        dir_(2) >>
        ps;
      pose_(3,0) = 0;
      pose_(3,1) = 0;
      pose_(3,2) = 0;
      pose_(3,3) = 1;
      *selectedpts = pointStringToGlowVector(ps, pose_);
      auto type=(Car::Type)type_;
      dir = dir_;
      std::shared_ptr<Car> c;
      try {
        c = std::make_shared<StaticCar>(*(cars[0]->at(model)));
      } catch (const std::out_of_range& oor) {
          //std::cerr << "Out of Range error: " << oor.what() << '\n';
          std::string message= "Car not in list: " + model;
          throw message.c_str();
      }
      c->setPosition(pose_);
      c->setOriginalPoints(selectedpts);
      results.push_back(c);
      wanted_result_size =1;
      //std::cout<<ps.substr(0,100)<<std::endl;
      connect(this, SIGNAL(carFinished()),this,SLOT(manageResults()));
    }
    break;
    case Car::MOVING_CAR:{
      Eigen::Vector4f dir_;
      int initPose;
      std::string positions_str;
      std::string points_str;
      config_stream >>
        initPose >>
        positions_str >>
        dir_(0) >>
        dir_(1) >>
        dir_(2) >>
        points_str;

      //Split Positions
      std::vector<std::string> single_pos_str;
      std::map<int,Eigen::Matrix4f> position;
      boost::split(single_pos_str, positions_str, [](char c){return c == ';';});
      for (const auto& s:single_pos_str){
        Eigen::Matrix4f pose_;
        int scanid;
        setlocale(LC_ALL|~LC_NUMERIC, "");
        std::vector<std::string> num_single_pos_str;
        std::vector<std::string> pose_str;
        boost::split(num_single_pos_str, s, [](char c){return c == ':';});
        scanid = std::stoi(num_single_pos_str[0]);
        //boost::split(pose_str,num_single_pos_str[1], [](char c){return c == ',';});
        
        std::replace( num_single_pos_str[1].begin(), num_single_pos_str[1].end(), ',', ' ');
        std::istringstream poses_txt(num_single_pos_str[1]);
        poses_txt >>
          pose_(0,0) >>
          pose_(1,0) >>
          pose_(2,0) >>
          pose_(0,1) >>
          pose_(1,1) >>
          pose_(2,1) >>
          pose_(0,2) >>
          pose_(1,2) >>
          pose_(2,2) >>
          pose_(0,3) >>
          pose_(1,3) >>
          pose_(2,3);
        pose_(3,0) = 0;
        pose_(3,1) = 0;
        pose_(3,2) = 0;
        pose_(3,3) = 1;
        //std::cout<<pose_<<std::endl;
        position[scanid] = pose_;
        
      }

      //Split Points
      std::vector<std::string> single_point_str;
      boost::split(single_point_str, points_str, [](char c){return c == ';';});
      std::map<int,std::shared_ptr<std::vector<glow::vec4>>> originalPoints;
      
      for (const auto& s:single_point_str){
        std::vector<std::string> num_single_point_str;
        int scanid;
        boost::split(num_single_point_str, s, [](char c){return c == ':';});
        scanid = std::stoi(num_single_point_str[0]);
        originalPoints[scanid]=std::make_shared<std::vector<glow::vec4>>();
        *(originalPoints[scanid])=pointStringToGlowVector(num_single_point_str[1], position[scanid]);
      }


      auto globalPoints=std::make_shared<std::vector<glow::vec4>>();

      for(const auto& i:originalPoints){
        auto pose_ = position[i.first];
        for(const auto& p:*(i.second)){
          Eigen::Vector4f v;
          v << p.x, p.y, p.z, 1;
          v = pose_.inverse() * v; 
          globalPoints->push_back(vec4(v.x(),v.y(),v.z(),1));
          //globalPoints->push_back(p);
        }
      }

      auto c = std::make_shared<MovingCar>("_generated",globalPoints);
      c->setPosition(position);
      c->setOriginalPoints(originalPoints);
      c->setInitPose(initPose);
      results.push_back(std::dynamic_pointer_cast<Car>(c));
      wanted_result_size =1;
      

      //throw "[TODO] Loading Moving_Car not Implemented";
    }
    break;
    default:
      throw "No such Type";
  }

	

}


AutoAuto::~AutoAuto(){

}

std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> AutoAuto::loadCarModels(const std::string& path){
  auto dirp = opendir(path.c_str()); //"../cars"
  if (dirp == NULL){
    std::cout << "Cars-Folder not found." <<std::endl;
    return nullptr;
  }
  
  struct dirent* dp;
  auto loadedCars = std::make_shared<std::map<std::string,  std::shared_ptr<Car>>>();
  while ((dp = readdir(dirp)) != NULL) {
    if (std::string(dp->d_name).length() < 4) continue; 
    if (0 != std::string(dp->d_name).compare (std::string(dp->d_name).length() - 4, 4, ".xyz")) continue; 

    std::basic_string<char> filepath = (std::string(path)+"/"+(dp->d_name)).c_str();

    std::ifstream infile(filepath);
    float x, y, z, a,b,c;

    std::shared_ptr<std::vector<glow::vec4>> v = std::make_shared<std::vector<glow::vec4>>();

    while (infile >> x >> y >> z >> a >> b >> c)
    {
        v->push_back(vec4(x,y,z,1));
    }
    if (v->size() == 0){
    	std::cout<<filepath<<": File corrupted or empty. SKIP"<<std::endl;
    	continue;
    }
    std::shared_ptr<Car> sptr_c=  std::make_shared<StaticCar>(std::string(dp->d_name), v);
    loadedCars->insert(std::pair<std::string, std::shared_ptr<Car>>(std::string(dp->d_name), sptr_c));
    //std::cout<<filepath<<": "<<v->size()<<" Points"<<std::endl;

  }
  (void)closedir(dirp);
  return loadedCars;
}

std::shared_ptr<Car> AutoAuto::icpMatch(const Car& inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t){
 
  std::shared_ptr<std::vector<glow::vec4>> carpts = inpc.getPoints();
  //std::cout<<inpc.getModel()<<": "<<carpts->size()<<" Car-Points"<<std::endl;

  std::vector<double> car_;
  for(auto const& value: *carpts) {
    car_.push_back(value.x);
    car_.push_back(value.y);
    car_.push_back(value.z);
  }

  //std::cout<<car_.size()<<std::endl;

  //add street for fitting
  Eigen::Vector3f mincar;
  mincar << (*carpts)[0].x, (*carpts)[0].y, (*carpts)[0].z;
  Eigen::Vector3f maxcar;
  maxcar << (*carpts)[0].x, (*carpts)[0].y, (*carpts)[0].z;
  
  for(auto const& value: *carpts) {
    if (mincar.x() > value.x) (mincar.x() = value.x);
    if (mincar.y() > value.y) (mincar.y() = value.y);
    if (mincar.z() > value.z) (mincar.z() = value.z);
    if (maxcar.x() < value.x) (maxcar.x() = value.x);
    if (maxcar.y() < value.y) (maxcar.y() = value.y);
    if (maxcar.z() < value.z) (maxcar.z() = value.z);
  }


  for(float i=mincar.x()-0.1; i<=maxcar.x()+0.1; i+=0.05){
    for(float j=mincar.y()-0.1; j<=maxcar.y()+0.1; j+=0.05){
      car_.push_back(i);
      car_.push_back(j);
      car_.push_back(mincar.z());
    }
  }
  //std::random_shuffle ( car_.begin(), car_.end() );

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator

  auto n = (car_.end()-car_.begin());
  for (auto i=(n/3)-1; i>0; --i) {
    std::uniform_int_distribution<> distr(0, i+1); // define the range
    int g = distr(eng);
    iter_swap (car_.begin()+i*3,car_.begin()+g*3);
    iter_swap (car_.begin()+i*3+1,car_.begin()+g*3+1);
    iter_swap (car_.begin()+i*3+2,car_.begin()+g*3+2);
  }
  //std::cout<<car_.size()<<std::endl;


  Matrix r_(3,3,r);
  Matrix t_(3,1,t);

  std::clock_t start;
  

  start = std::clock();

  //std::cout<<"ICP"<<std::endl;
  IcpPointToPoint icp(worldpts_->data(),worldpts_->size()/3,3,(float)1e-3);
  //icp.max_iter=200;
  //icp.fit(car_.data(),car_.size()/24,r_,t_,-1);

  /*icp.max_iter=50;
  icp.fit(car_.data(),car_.size()/12,r_,t_,-1);

  icp.max_iter=50;
  icp.min_delta=1e-4;
  icp.fit(car_.data(),car_.size()/6,r_,t_,-1);
  */
  //icp.max_iter=300;
  //icp.min_delta=1e-5;
  //icp.fit(car_.data(),car_.size()/24,r_,t_,0.1);
  icp.max_iter=300;
  icp.min_delta=1e-5;
  icp.fit(car_.data(),car_.size()/3,r_,t_,0.1);

  double duration_ = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

  std::cout<<"[duration: "<< duration_ <<"]"<<'\n';
  duration+=duration_;

  FLOAT r2[9];
  FLOAT t2[3];
  r_.getData(r2,0,0,2,2);
  t_.getData(t2,0,0,2,0);

  Matrix r_2(r_);
  Matrix t_2(t_);

  r_2.inv();
  t_2=r_2*t_2*-1;

  //std::cout<<"reverse matching"<<std::endl;

  IcpPointToPoint icp2(car_.data(),car_.size()/3,3,(float)1e10); //set min delta to "very high"
  float inlier=icp2.getSqDistance(worldpts_->data(),worldpts_->size()/3,r_2,t_2,0.3);
  inlier+=icp.getSqDistance(car_.data(),car_.size()/3,r_,t_,-1);
  Eigen::Matrix4f newrotmat;
  newrotmat <<
    r2[0], r2[1],r2[2], t2[0],
    r2[3], r2[4],r2[5], t2[1],
    r2[6], r2[7],r2[8], t2[2],
    0.,0.,0.,1.;

  /*for(const auto& value: *carpts) {
    Eigen::Vector4f v;
    v << value.x,value.y,value.z,1;
    v = newrotmat * v;
    //cars_out[inpc.getModel()].push_back(vec4(v.x(),v.y(),v.z(),1));
  }*/
  //std::cout<<inpc.getModel()<<": "<<carpts->size()<<" Car-Points"<<std::endl;

  //std::cout<<inpc.getModel()<<": "<<cars_out[model].size()<<" Car-Points"<<std::endl;
  //progress->setValue(carprogress++);

  std::shared_ptr<Car> c = std::make_shared<StaticCar>(inpc);
  c->setPosition(newrotmat);
  c->setInlier(inlier);
  c->setOriginalPoints(selectedpts);
  //std::cout<<"matched"<<std::endl;
  return c;
}

void AutoAuto::matchPosition(const std::shared_ptr<std::vector<glow::vec4>> pts, const Eigen::Vector4f dir_, const Eigen::Vector4f center_, int step){
   //calculate box size
  //pool.stop(true);
  last_step = step;
  if ((int)cars.size()<=step){
    emit carFinished();
    emit carProgressFinished(this);
    return;
  }
  wanted_result_size += cars[step]->size();
  selectedpts = pts;
  dir = dir_;
  emit carProgressUpdate(0);
  
  Eigen::Vector4f carpos;
  if (center_==Eigen::Vector4f::Zero()){

    std::vector<float> xlist;
    std::vector<float> ylist;
    std::vector<float> zlist;

    for(auto const& value: *pts) {
      xlist.push_back(value.x);
      ylist.push_back(value.y);
      zlist.push_back(value.z);
    }
    std::sort (xlist.begin(), xlist.end());
    std::sort (ylist.begin(), ylist.end());
    std::sort (zlist.begin(), zlist.end());


    Eigen::Vector4f vec20; 
    vec20 << 
      xlist[floor(xlist.size()*0.05)],
      ylist[floor(ylist.size()*0.05)],
      zlist[floor(zlist.size()*0.05)],
      1;
    Eigen::Vector4f vec80;
    vec80 << 
      xlist[floor(xlist.size()*0.95)],
      ylist[floor(ylist.size()*0.95)],
      zlist[floor(zlist.size()*0.95)],
      1;
    
    carpos = vec20 + (0.5 * (vec80-vec20));
  }else{
    carpos = center_;
  }
  center = center_;
  //compute car rotation matrix
  Eigen::Vector3f f; f<<dir.x(),dir.y(),dir.z();
  f=f.normalized();
  Eigen::Vector3f u; u<<0,0,1;
  Eigen::Vector3f s = f.cross(u).normalized();
  u = s.cross(f).normalized();


  Eigen::Matrix4f rottransmat;
  rottransmat << s.x(), u.x(),-f.x(), carpos.x(),
                 s.y(), u.y(),-f.y(), carpos.y(),
                 s.z(), u.z(),-f.z(), carpos.z(),
                 0.,0.,0.,1.;

  Eigen::Matrix4f carOrientation;
  carOrientation << -1,0,0,0,
                    0,0,1,0,
                    0,1,0,0,
                    0,0,0,1;

  rottransmat*=carOrientation;

  FLOAT r[] = {
    rottransmat(0,0),rottransmat(0,1),rottransmat(0,2),
    rottransmat(1,0),rottransmat(1,1),rottransmat(1,2),
    rottransmat(2,0),rottransmat(2,1),rottransmat(2,2)
  };
  FLOAT t[] = {
    rottransmat(0,3),
    rottransmat(1,3),
    rottransmat(2,3)
  };

  //Convert Format 
  auto pts_ = std::make_shared<std::vector<double>>();
  std::random_shuffle ( pts_->begin(), pts_->end() );
  
  for(auto const& value: *pts) {
    pts_->push_back(value.x);
    pts_->push_back(value.y);
    pts_->push_back(value.z);
  }

  /*std::map<std::string,std::vector<glow::vec4>> cars_out;
  std::map<std::string,uint32_t> cars_inlier;

  for (auto const& x : *cars) {
    cars_out.insert({x.first,std::vector<glow::vec4>()});
    cars_inlier.insert({x.first,0});
    std::cout<<x.first<<": "<<" Added to vector"<<std::endl;
  }*/


  //std::vector<std::shared_ptr<Car>> results;
  //MAIN LOOP
  //uint32_t carprogress=0;
  //AutoAuto* this_ = this;
  pool.reinit();
  for (auto const& x : *(cars.at(step))) {
    pool.push([x, pts_, r, t, this](int){
    	auto rslt = icpMatch(*(x.second), pts_, r, t);
    	addResult(rslt);
    	carFinished();
    	qDebug()<<"Car finished";
    });
  }
}

void AutoAuto::manageResults(){
  int ressize = results.size();
	//results.push_back(car);
	manageResultsMutex.lock();
	//std::cout<<"RESULT "<<results.size()<<"/"<<cars->size()<<std::endl;
	emit carProgressUpdate((float)ressize/wanted_result_size);
	if (ressize==wanted_result_size){
    std::cout<<"Total: "<<duration<<" seconds"<<std::endl;
	  auto cmp = [](std::shared_ptr<Car> const & a, std::shared_ptr<Car> const & b) { 
	    return a->getInlier() < b->getInlier();
	  };

	  std::sort(results.begin(),results.end(),cmp);
	  emit carProgressFinished(this);
	  pool.stop(true);
	  //std::cout<<"GOT ALL RESULTS"<<std::endl;
	}
	manageResultsMutex.unlock();

}

std::vector<std::shared_ptr<Car>> AutoAuto::getResults(){
	return results;
} 

void AutoAuto::addResult(std::shared_ptr<Car> result){
	resultsMutex.lock();
	results.push_back(result);
	resultsMutex.unlock();
}

void AutoAuto::setSelectedCar(int i){
	selectedcar = i;
}

void AutoAuto::setSelectedpts(std::vector<glow::vec4> s){
  *selectedpts = s;
};

int AutoAuto::getSelectedCar(){
	return selectedcar;
}

std::string AutoAuto::pointGlowVectorToString(const std::vector<glow::vec4> v,Eigen::Matrix4f pose){
	std::string outvec = "";
	std::vector<Eigen::Vector4f> ev;
	for(const auto& p:v){
		Eigen::Vector4f e;
		e<<p.x,p.y,p.z,1;
		e=pose.inverse()*e*1000;
		int16_t coords[3];
		if(e.x()>32767 || e.x()<-32767 ||e.y()>32767 || e.y()<-32767 ||e.z()>32767 || e.z()<-32767){
			std::cout<<"[ ERROR ]  Coordinates out of Range"<<e.x()<<","<<e.y()<<","<<e.z()<<std::endl;
			continue;
		}

		coords[0]=e.x();
		coords[1]=e.y();
		coords[2]=e.z();

		outvec += base64_encode((BYTE*) coords, 6);
		//(char *)coords;
	}
	return outvec;

}

std::vector<glow::vec4> AutoAuto::pointStringToGlowVector(const std::string s,Eigen::Matrix4f pose__){
	std::vector<glow::vec4> outvec;
	for(unsigned int i=0;i<s.length();i+=8){
		std::vector<BYTE>coords = base64_decode(s.substr(i, 8));
		Eigen::Vector4f e;
		e << (float)((int16_t *)coords.data())[0],
		     (float)((int16_t *)coords.data())[1],
		     (float)((int16_t *)coords.data())[2],
		     1;

		e=pose__*e/1000.;
		outvec.push_back(glow::vec4(e.x(),e.y(),e.z(),1));
		std::cout<<e.x()<<","<<e.y()<<","<<e.z()<<std::endl;
	}
  std::cout<<pose__<<std::endl;
	return outvec;
}

std::string AutoAuto::getString(){
	return getString(selectedcar);
}

std::string AutoAuto::getString(int i){
	std::ostringstream s;
  switch(results[i]->getType()){
    case Car::STATIC_CAR:{
      Eigen::Matrix4f pose = results[i]->getPosition();
    	s << static_cast<int>(results[i]->getType());
      s << " " << results[i]->getModel();
    	s << " " << pose(0,0);
    	s << " " << pose(1,0);
    	s << " " << pose(2,0);
    	s << " " << pose(0,1);
    	s << " " << pose(1,1);
    	s << " " << pose(2,1);
    	s << " " << pose(0,2);
    	s << " " << pose(1,2);
    	s << " " << pose(2,2);
    	s << " " << pose(0,3);
    	s << " " << pose(1,3);
    	s << " " << pose(2,3);
    	s << " " << dir(0);
    	s << " " << dir(1);
    	s << " " << dir(2);
    	s << " " << results[i]->getPointString();
    	//std::cout << s.str().substr(0,(s.str().length()>800?800:s.str().length())) <<" [...]"<< std::endl;
    	//std::vector<glow::vec4> v = pointStringToGlowVector(s ,results[selectedcar]->getPosition());
    	return s.str();
    }
    case Car::MOVING_CAR:{
      auto mcar = std::dynamic_pointer_cast<MovingCar>(results[i]);
      s << static_cast<int>(results[i]->getType())<<" ";
      s << mcar->getInitPose()<<" ";
      bool first=true;
      for (auto const& x : mcar->getPositions()){
        if (!first){
          s << ";";
        }else{
          first=false;
        }
        s << x.first;  // string (key)
        Eigen::Matrix4f pose = x.second; // string's value 
        s << ":" << pose(0,0);
        s << "," << pose(1,0);
        s << "," << pose(2,0);
        s << "," << pose(0,1);
        s << "," << pose(1,1);
        s << "," << pose(2,1);
        s << "," << pose(0,2);
        s << "," << pose(1,2);
        s << "," << pose(2,2);
        s << "," << pose(0,3);
        s << "," << pose(1,3);
        s << "," << pose(2,3);
      }
      s << " " << dir(0);
      s << " " << dir(1);
      s << " " << dir(2);
      s << " " << results[i]->getPointString();

      return s.str();
    }
  }
}
