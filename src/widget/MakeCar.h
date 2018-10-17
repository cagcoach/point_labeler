#include "Car.h"
#include "StaticCar.h"
#include "MovingCar.h"


static std::shared_ptr<Car> make_car(Car::Type t){
  switch(t){
    case Car::MOVING_CAR:
      return std::make_shared<MovingCar>();
    case Car::STATIC_CAR:
      return std::make_shared<StaticCar>();
  }
};