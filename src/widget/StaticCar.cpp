#include "StaticCar.h"
#include "base64.h"

StaticCar::~StaticCar(){

}

void StaticCar::setPosition(Eigen::Matrix4f position_) {
	position=position_;
};

void StaticCar::setPosition(std::map<int,Eigen::Matrix4f> position_) {
	position=position_.at(0);
	std::cout<<"Only saving first element as static"<<std::endl;
};

Eigen::Matrix4f StaticCar::getPosition() const {
	return position;
}

Eigen::Matrix4f StaticCar::getPosition(int scan) const {
	return position;
}
std::string StaticCar::getModel() const {
	return model;
}
std::shared_ptr<std::vector<glow::vec4>> StaticCar::getPoints() const {
	return points;
}

std::shared_ptr<std::vector<glow::vec4>> StaticCar::getGlobalPoints() const {
	auto gp = std::make_shared<std::vector<glow::vec4>>();
	for(const auto& value: *points) {
      Eigen::Vector4f v;
   	  v << value.x,value.y,value.z,1;
      v = position * v;
      gp->push_back(glow::vec4(v.x(),v.y(),v.z(),1));
    }
    return gp;
}

std::shared_ptr<std::vector<glow::vec4>> StaticCar::getGlobalPoints(int scan) const {
    return getGlobalPoints();
}

void StaticCar::setInlier(float i) {
	inlier=i;
}

float StaticCar::getInlier() const{
	return inlier;
}

void StaticCar::setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>> op){
	originalPoints = op;
}

void StaticCar::setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>> op, int scan){
	originalPoints = op;
}

void StaticCar::setOriginalPoints(std::map<int,std::shared_ptr<std::vector<glow::vec4>>> op){
	originalPoints = std::make_shared<std::vector<glow::vec4>>();
	for(const auto& v:op){
		originalPoints->insert(originalPoints->end(),v.second->begin(),v.second->end());
	}
}

std::shared_ptr<std::vector<glow::vec4>> StaticCar::getOriginalPoints(){
	return originalPoints;
}

std::shared_ptr<std::vector<glow::vec4>> StaticCar::getOriginalPoints(int scan){
	return originalPoints;
}

std::string StaticCar::getPointString(){
	return pointGlowVectorToString(getOriginalPoints(),getPosition());
	//std::cout << s.str().substr(0,(s.str().length()>800?800:s.str().length())) <<" [...]"<< std::endl;
	//std::vector<glow::vec4> v = pointStringToGlowVector(s ,results[selectedcar]->getPosition());
}

std::string StaticCar::pointGlowVectorToString(const std::shared_ptr<std::vector<glow::vec4>> v,Eigen::Matrix4f pose){
	std::string outvec = "";
	std::vector<Eigen::Vector4f> ev;
	for(const auto& p:(*v)){
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