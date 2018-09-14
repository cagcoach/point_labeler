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
    AutoAuto(const std::shared_ptr<std::map<std::string, Car>> cars_, std::string config);

    ~AutoAuto();
    static std::shared_ptr<std::map<std::string, Car>> loadCarModels(const std::string& path);
    std::shared_ptr<Car> icpMatch(Car inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t);
    void matchPosition(const std::vector<glow::vec4>& pts, const Eigen::Vector4f dir);
  	std::vector<std::shared_ptr<Car>> getResults();
  	std::string getString();
  	std::string getString(int i);
  	void setSelectedCar(int i);
  	int getSelectedCar();
  signals:
  	void carFinished();
  	void carProgressStarted();
  	void carProgressUpdate(float done);
  	void carProgressFinished(AutoAuto*);

  protected slots:
  	void manageResults();

  protected:
  	void addResult(std::shared_ptr<Car> result);
  	static std::string pointGlowVectorToString(const std::vector<glow::vec4> v,Eigen::Matrix4f pose);
  	static std::vector<glow::vec4> pointStringToGlowVector(const std::string s,Eigen::Matrix4f pose);

  	std::mutex resultsMutex;
  	std::mutex manageResultsMutex;
    const std::shared_ptr<std::map<std::string, Car>> cars;

    //TODO Add const
    Eigen::Vector4f dir;
    //TODO Add const
    std::vector<glow::vec4> selectedpts;

  	ctpl::thread_pool pool{NUM_THREADS};
  	std::vector<std::shared_ptr<Car>> results;
  	int selectedcar=0;

};

#endif