#include "Car.h"

Car::~Car(){

}
void Car::setPosition(Eigen::Matrix4f position_) {
	position=position_;
};

Eigen::Matrix4f Car::getPosition() const {
	return position;
}
std::string Car::getModel() const {
	return model;
}
std::shared_ptr<std::vector<glow::vec4>> Car::getPoints() const {
	return points;
}
std::shared_ptr<std::vector<glow::vec4>> Car::getGlobalPoints() const {
	auto gp = std::make_shared<std::vector<glow::vec4>>();
	for(const auto& value: *points) {
      Eigen::Vector4f v;
   	  v << value.x,value.y,value.z,1;
      v = position * v;
      gp->push_back(glow::vec4(v.x(),v.y(),v.z(),1));
    }
    return gp;
}
void Car::setInlier(uint32_t i) {
	inlier=i;
}
uint32_t Car::getInlier() const{
	return inlier;
}
