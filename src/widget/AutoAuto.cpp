#include "AutoAuto.h"



#include <dirent.h>
#include <string>
#include <fstream>
#include "libicp/icpPointToPoint.h"
#include "libicp/matrix.h"
#include <QDebug>





using namespace glow;


AutoAuto::~AutoAuto(){

}

std::shared_ptr<std::map<std::string, Car>> AutoAuto::loadCarModels(const std::string& path){
  auto dirp = opendir(path.c_str()); //"../cars"
  if (dirp == NULL){
    std::cout << "Cars-Folder not found." <<std::endl;
    return nullptr;
  }
  
  struct dirent* dp;
  auto loadedCars = std::make_shared<std::map<std::string, Car>>();
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
    
    loadedCars->insert(std::pair<std::string, Car>(dp->d_name,Car(std::string(dp->d_name), v)));
    //std::cout<<filepath<<": "<<v->size()<<" Points"<<std::endl;

  }
  (void)closedir(dirp);
  return loadedCars;
}

std::shared_ptr<Car> AutoAuto::icpMatch(Car inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t){
 
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

  //std::cout<<car_.size()<<std::endl;


  Matrix r_(3,3,r);
  Matrix t_(3,1,t);

  //std::cout<<"ICP"<<std::endl;

  IcpPointToPoint icp(worldpts_->data(),worldpts_->size()/3,3);
  icp.fit(car_.data(),car_.size()/3,r_,t_,-1);

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
  uint32_t inlier=icp2.getInlierSize(worldpts_->data(),worldpts_->size()/3,r_2,t_2,0.1);
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

  std::shared_ptr<Car> c = std::make_shared<Car>(inpc);
  c->setPosition(newrotmat);
  c->setInlier(inlier);
  //std::cout<<"matched"<<std::endl;
  return c;
}

void AutoAuto::matchPosition(const std::vector<glow::vec4>& pts, const Eigen::Vector4f dir){
   //calculate box size
  //pool.stop(true);
  emit carProgressUpdate(0);
  std::vector<float> xlist;
  std::vector<float> ylist;
  std::vector<float> zlist;

  for(auto const& value: pts) {
    xlist.push_back(value.x);
    ylist.push_back(value.y);
    zlist.push_back(value.z);
  }
  std::sort (xlist.begin(), xlist.end());
  std::sort (ylist.begin(), ylist.end());
  std::sort (zlist.begin(), zlist.end());


  Eigen::Vector4f vec20; 
  vec20 << 
    xlist[floor(xlist.size()*0.2)],
    ylist[floor(ylist.size()*0.2)],
    zlist[floor(zlist.size()*0.2)],
    1;
  Eigen::Vector4f vec80;
  vec80 << 
    xlist[floor(xlist.size()*0.8)],
    ylist[floor(ylist.size()*0.8)],
    zlist[floor(zlist.size()*0.8)],
    1;
  
  Eigen::Vector4f carpos = vec20 + (0.5 * (vec80-vec20));

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
  for(auto const& value: pts) {
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
  connect(this, SIGNAL(carFinished()),this,SLOT(manageResults()));
  for (auto const& x : *cars) {
    pool.push([x, pts_, r, t, this](int){
    	auto rslt = icpMatch(x.second, pts_, r, t);
    	addResult(rslt);
    	carFinished();
    	qDebug()<<"Car finished";
    });
  }

}

void AutoAuto::manageResults(){
	//results.push_back(car);
	manageResultsMutex.lock();
	//std::cout<<"RESULT "<<results.size()<<"/"<<cars->size()<<std::endl;
	emit carProgressUpdate((float)results.size()/cars->size());
	if (results.size()==cars->size()){
	  auto cmp = [](std::shared_ptr<Car> const & a, std::shared_ptr<Car> const & b) { 
	    return a->getInlier() > b->getInlier();
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