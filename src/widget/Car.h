#ifndef CAR_H_
#define CAR_H_

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <glow/glutil.h>
#include <map>


class Car{
public:
  //Car(){model="";};
  ~Car(){};
  virtual void setPosition(Eigen::Matrix4f position_) = 0;
  virtual void setPosition(std::map<int,Eigen::Matrix4f> position_) = 0;
  virtual Eigen::Matrix4f getPosition() const = 0;
  virtual Eigen::Matrix4f getPosition(int scan) const = 0;
  virtual std::string getModel() const = 0;
  virtual std::shared_ptr<std::vector<glow::vec4>> getPoints() const = 0;
  virtual void setInlier(float i) = 0;
  virtual float getInlier() const = 0;
  virtual std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints() const = 0;
  virtual std::shared_ptr<std::vector<glow::vec4>> getGlobalPoints(int scan) const = 0; 
  virtual void setScan(int scan){};
  virtual void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>) = 0;
  virtual void setOriginalPoints(std::shared_ptr<std::vector<glow::vec4>>, int scan) = 0;
  virtual void setOriginalPoints(std::map<int,std::shared_ptr<std::vector<glow::vec4>>>) = 0;
  virtual std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints() = 0;
  virtual std::shared_ptr<std::vector<glow::vec4>> getOriginalPoints(int scan) = 0;
  enum Type{STATIC_CAR = 0, MOVING_CAR = 1};
  virtual Type getType() = 0;
  virtual std::string getPointString()=0;
  virtual void setPointString(std::string ps)=0;
  /*
  static std::shared_ptr<Car> make_car(Type t){
    switch(t){
      case Car::MOVING_CAR:
        return std::make_shared<MovingCar>();
      case Car::STATIC_CAR:
        return std::make_shared<StaticCar>();
    }
  }
  */
protected:
  std::string model;
};


#endif