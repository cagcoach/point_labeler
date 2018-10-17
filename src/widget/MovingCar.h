#ifndef MOVING_CAR_H_
#define MOVING_CAR_H_

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <glow/glutil.h>
#include <map>
#include "Car.h"

class MovingCar : virtual public Car{
public:
  MovingCar(std::string model_, std::shared_ptr<std::vector<glow::vec4>> points_)
    { model = model_;
      points = points_;
      position[0]=(Eigen::Matrix4f::Identity());
    };
  MovingCar(const Car& c):
    model(c.getModel()),
    points(c.getPoints()),
    inlier(c.getInlier()) {
      position[0]=(c.getPosition());
    };
  MovingCar(const MovingCar& c):
    model(c.getModel()),
    points(c.getPoints()),
    inlier(c.getInlier()) {
      position[0]=(c.getPosition());
    };;
  ~MovingCar();
  void setPosition(Eigen::Matrix4f position_);
  void setPosition(std::map<int,Eigen::Matrix4f> position_);
  Eigen::Matrix4f getPosition() const;
  std::map<int,Eigen::Matrix4f> getPositions();
  Eigen::Matrix4f getPosition(int scan) const;
  std::string getModel() const;
  std::shared_ptr<std::vector<glow::vec4>> getPoints() const;
  void setInlier(float i);
  float getInlier() const;
  std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints() const;
  std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints(int scan) const; 
  void setScan(int scan_){scan = scan_;};
  Car::Type getType(){return Car::MOVING_CAR;};
  void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>);
  void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>, int scan);
  void setOriginalPoints(std::map<int,std::shared_ptr<std::vector<glow::vec4>>>);
  std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints();
  std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints(int scan);
  std::string getPointString();

private:
  std::string pointGlowVectorToString(const std::shared_ptr<std::vector<glow::vec4>> v,Eigen::Matrix4f pose);


protected:
  std::string model;
  std::shared_ptr<std::vector<glow::vec4>> points;
  std::map<int,std::shared_ptr<std::vector<glow::vec4>>> originalPoints;
  std::map<int,Eigen::Matrix4f> position;
  float inlier {0};
  int scan{0};

};

#endif