#include "Car.h"
#include "StaticCar.h"
#include "MovingCar.h"


static std::shared_ptr<Car> make_car(Car::Type t,std::string model_, std::shared_ptr<std::vector<glow::vec4>> points_){
  switch(t){
    case Car::MOVING_CAR:
      return std::make_shared<MovingCar>(model_,points_);
    case Car::STATIC_CAR:
      return std::make_shared<StaticCar>(model_,points_);
  }
};

static std::shared_ptr<Car> make_car(Car::Type t, Car& c){
  switch(t){
    case Car::MOVING_CAR:
      return std::make_shared<MovingCar>(c);
    case Car::STATIC_CAR:
      return std::make_shared<StaticCar>(c);
  }
};