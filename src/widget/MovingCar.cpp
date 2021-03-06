#include "MovingCar.h"
#include "base64.h"


MovingCar::~MovingCar(){

}
void MovingCar::setPosition(Eigen::Matrix4f position_) {
	position[scan]=position_;
	changed=true;
};

void MovingCar::setPosition(std::map<int,Eigen::Matrix4f> position_){
	position=position_;
	changed=true;
}

Eigen::Matrix4f MovingCar::getPosition() const {
	return getPosition(scan);
}

Eigen::Matrix4f MovingCar::getPosition(const int scan_) const {
	return position.at(scan_);
}
std::string MovingCar::getModel() const {
	return model;
}
std::shared_ptr<std::vector<glow::vec4>> MovingCar::getPoints() const {
	return points;
}
std::shared_ptr<std::vector<glow::vec4>> MovingCar::getGlobalPoints() const {
	return getGlobalPoints(scan);
}
std::shared_ptr<std::vector<glow::vec4>> MovingCar::getGlobalPoints(const int scan_) const {
    auto gp = std::make_shared<std::vector<glow::vec4>>();
	if (position.find(scan_) == position.end()) return gp;
	for(const auto& value: *points) {
      Eigen::Vector4f v;
   	  v << value.x,value.y,value.z,1;
      v = position.at(scan_) * v;
      gp->push_back(glow::vec4(v.x(),v.y(),v.z(),1));
    }
    return gp;

}
void MovingCar::setInlier(float i) {
	inlier=i;
	changed=true;
}
float MovingCar::getInlier() const{
	return inlier;
}

std::map<int,Eigen::Matrix4f> MovingCar::getPositions(){
	return position;
}

void MovingCar::setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>> op){
	originalPoints[scan] = op;
	changed=true;
}

void MovingCar::setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>> op, int scan_){
	originalPoints[scan_] = op;
	changed=true;
}

void MovingCar::setOriginalPoints(std::map<int,std::shared_ptr<std::vector<glow::vec4>>> op){
	originalPoints = op;
	changed=true;
}

std::shared_ptr<std::vector<glow::vec4>> MovingCar::getOriginalPoints(){
	return getOriginalPoints(scan);
}

std::shared_ptr<std::vector<glow::vec4>> MovingCar::getOriginalPoints(int scan_){
	auto it = originalPoints.find(scan_);
	if(it!=originalPoints.end()){
		return it->second;
	}else{
		return nullptr;
	}
	
}

void MovingCar::setPointString(std::string ps){
	pointString=ps;
	changed=false;
}

std::string MovingCar::getPointString(){
	if (!changed) return pointString;

	std::stringstream out;
	bool first=true;
	for(const auto& v:originalPoints){
		if (!first){
			out << ";";
		}else{
			first=false;
		}
		out << v.first;
		out << ":";
		out << pointGlowVectorToString(v.second,position.at(v.first));
	}
	pointString = out.str();
	return pointString;
}

std::string MovingCar::pointGlowVectorToString(const std::shared_ptr<std::vector<glow::vec4>> v,Eigen::Matrix4f pose__){
	std::string outvec = "";
	std::vector<Eigen::Vector4f> ev;
	for(const auto p:(*v)){
		Eigen::Vector4f e;
		e<<p.x,p.y,p.z,1;
		std::cout<<p.x<<","<<p.y<<","<<p.z<<std::endl;
		e=pose__.inverse()*(e*1000);
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
	std::cout<<pose__<<std::endl;


	return outvec;

}