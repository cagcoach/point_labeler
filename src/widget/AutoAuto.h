#ifndef AUTOAUTO_H_
#define AUTOAUTO_H_

#include "Car.h"
#include "libicp/icpPointToPoint.h"
#include <map>
#include <QObject> 
#include <mutex>
#include "MovingCar.h"
#include "StaticCar.h"
//#define NUM_THREADS 6

class AutoAuto : public QObject{
  Q_OBJECT
  public:
    AutoAuto(const std::vector<std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>>> cars_);

    AutoAuto(const std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> cars_, std::string config);

    ~AutoAuto();
    static std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> loadCarModels(const std::string& path);
    std::shared_ptr<Car> icpMatch(const Car& inpc, const std::shared_ptr<std::vector<double>> worldpts_, const FLOAT* r,  const FLOAT* t);
    void matchPosition()
      {matchPosition(selectedpts,dir,center,last_step+1);};
    void matchPosition(const std::shared_ptr<std::vector<glow::vec4>> pts_, const Eigen::Vector4f dir_)
      {matchPosition(pts_,dir_,center,last_step+1);};
    void matchPosition(const std::shared_ptr<std::vector<glow::vec4>> pts_, const Eigen::Vector4f dir_, const Eigen::Vector4f center_)
      {matchPosition(pts_,dir_,center_,last_step+1);};
    void matchPosition(const std::shared_ptr<std::vector<glow::vec4>> pts_, const Eigen::Vector4f dir_, const int step)
      {matchPosition(pts_,dir_,center,step);};
    void matchPosition(const std::shared_ptr<std::vector<glow::vec4>> pts, const Eigen::Vector4f dir, const Eigen::Vector4f center_, const int step);
  	void setSelectedpts(std::vector<glow::vec4> s);
    std::vector<std::shared_ptr<Car>> getResults();
  	std::string getString();
  	std::string getString(int i);
  	void setSelectedCar(int i);
    void setDir(Eigen::Vector4f dir_){dir=dir_;};
  	int getSelectedCar();
    void addResult(std::shared_ptr<Car> result);
  signals:
  	void carFinished();
  	void carProgressStarted();
  	void carProgressUpdate(float done);
  	void carProgressFinished(AutoAuto*);

  protected slots:
  	void manageResults();

  protected:
  	
  	static std::string pointGlowVectorToString(const std::vector<glow::vec4> v,Eigen::Matrix4f pose);
  	static std::vector<glow::vec4> pointStringToGlowVector(const std::string s,Eigen::Matrix4f pose);

  	std::mutex resultsMutex;
  	std::mutex manageResultsMutex;
    std::vector<std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>>> cars;

    //TODO Add const
    Eigen::Vector4f dir;
    Eigen::Vector4f center{Eigen::Vector4f::Zero()};
    //TODO Add const
    std::shared_ptr<std::vector<glow::vec4>> selectedpts{std::make_shared<std::vector<glow::vec4>>()};
    unsigned int NUM_THREADS{(unsigned int)std::thread::hardware_concurrency()};
    ctpl::thread_pool pool{(int)NUM_THREADS};
    std::vector<std::shared_ptr<Car>> results;
    int last_step = -1;
    int wanted_result_size = 0;
    int selectedcar=0;
    double duration=0;
    int triggeredforsize = -1;
};

#endif
