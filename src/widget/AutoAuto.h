#ifndef AUTOAUTO_H_
#define AUTOAUTO_H_

#include "Car.h"
#include "libicp/icpPointToPoint.h"
#include <map>
#include <QObject> 
#include <mutex>

//#define NUM_THREADS 6

class AutoAuto : public QObject{
  Q_OBJECT
  public:
    AutoAuto(const std::vector<std::shared_ptr<std::map<std::string, Car>>> cars_);
    AutoAuto(const std::shared_ptr<std::map<std::string, Car>> cars_, std::string config);

    ~AutoAuto();
    static std::shared_ptr<std::map<std::string, Car>> loadCarModels(const std::string& path);
    std::shared_ptr<Car> icpMatch(Car inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t);
    void matchPosition(){matchPosition(selectedpts,dir,last_step+1);};
    void matchPosition(const std::vector<glow::vec4>& pts_, const Eigen::Vector4f dir_){matchPosition(pts_,dir_,last_step+1);};
    void matchPosition(const std::vector<glow::vec4>& pts, const Eigen::Vector4f dir, const int step);
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
    std::vector<std::shared_ptr<std::map<std::string, Car>>> cars;

    //TODO Add const
    Eigen::Vector4f dir;
    //TODO Add const
    std::vector<glow::vec4> selectedpts;
    unsigned int NUM_THREADS{std::thread::hardware_concurrency()};
    ctpl::thread_pool pool{NUM_THREADS};
    std::vector<std::shared_ptr<Car>> results;
    int last_step = -1;
    int wanted_result_size = 0;
    int selectedcar=0;
    double duration=0;
};

#endif
