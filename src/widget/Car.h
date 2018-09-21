#ifndef CAR_H_
#define CAR_H_

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <glow/glutil.h>

class Car{
public:
  Car(std::string model_, std::shared_ptr<std::vector<glow::vec4>> points_) : 
    model(model_),
    points(points_),
    position(Eigen::Matrix4f::Identity()) {};
  Car(const Car& c):
    model(c.getModel()),
    points(c.getPoints()),
    position(c.getPosition()),
    inlier(c.getInlier()) {};
  ~Car();
  void setPosition(Eigen::Matrix4f position_);
  Eigen::Matrix4f getPosition() const;
  std::string getModel() const;
  std::shared_ptr<std::vector<glow::vec4>> getPoints() const;
  void setInlier(float i);
  float getInlier() const;
  std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints() const; 


protected:
  std::string model;
  std::shared_ptr<std::vector<glow::vec4>> points {nullptr};
  Eigen::Matrix4f position;
  float inlier {0};

};

#endif