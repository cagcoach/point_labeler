#ifndef AUTOAUTO_H_
#define AUTOAUTO_H_

#include "Car.h"
#include "libicp/icpPointToPoint.h"
#include <map>
#include <QObject> 
#include <mutex>

#define NUM_THREADS 8

class AutoAuto : public QObject{
  Q_OBJECT
  public:
    AutoAuto(const std::shared_ptr<std::map<std::string, Car>> cars_):cars(cars_) {};

    ~AutoAuto();
    static std::shared_ptr<std::map<std::string, Car>> loadCarModels(const std::string& path);
    std::shared_ptr<Car> icpMatch(Car inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t);
    void matchPosition(const std::vector<glow::vec4>& pts, const Eigen::Vector4f dir);
  	std::vector<std::shared_ptr<Car>> getResults();
  signals:
  	void carFinished();
  	void carProgressStarted();
  	void carProgressUpdate(float done);
  	void carProgressFinished(AutoAuto*);

  protected slots:
  	void manageResults();

  protected:
  	void addResult(std::shared_ptr<Car> result);
  	std::mutex resultsMutex;
  	std::mutex manageResultsMutex;
    const std::shared_ptr<std::map<std::string, Car>> cars;
  	ctpl::thread_pool pool{NUM_THREADS};
  	std::vector<std::shared_ptr<Car>> results;

};

#endif