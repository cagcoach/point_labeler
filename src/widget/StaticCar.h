#ifndef STATIC_CAR_H_
#define STATIC_CAR_H_

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <glow/glutil.h>
#include <map>
#include "Car.h"

class StaticCar : virtual public Car{
public:
  StaticCar(std::string model_, std::shared_ptr<std::vector<glow::vec4>> points_) : 
    model(model_),
    points(points_),
    position(Eigen::Matrix4f::Identity()) {};
  StaticCar(const Car& c):
    model(c.getModel()),
    points(c.getPoints()),
    position(c.getPosition()),
    inlier(c.getInlier()) {};
  ~StaticCar();
  void setPosition(Eigen::Matrix4f position_);
  void setPosition(std::map<int,Eigen::Matrix4f> position_);
  Eigen::Matrix4f getPosition() const;
  Eigen::Matrix4f getPosition(int scan) const;
  std::string getModel() const;
  std::shared_ptr<std::vector<glow::vec4>> getPoints() const;
  void setInlier(float i);
  float getInlier() const;
  std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints() const;
  std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints(int scan) const; 
  void setScan(int scan){};
  void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>);
  void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>, int scan);
  void setOriginalPoints(std::map<int,std::shared_ptr<std::vector<glow::vec4>>>);
  std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints();
  std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints(int scan);

  Car::Type getType(){return Car::STATIC_CAR;};
  std::string getPointString();

private:
  std::string pointGlowVectorToString(const std::shared_ptr<std::vector<glow::vec4>> v,Eigen::Matrix4f pose);


protected:
  std::string model;
  std::shared_ptr<std::vector<glow::vec4>> points {nullptr};
  std::shared_ptr<std::vector<glow::vec4>> originalPoints {nullptr};
  Eigen::Matrix4f position;
  float inlier {0};

};

#endif