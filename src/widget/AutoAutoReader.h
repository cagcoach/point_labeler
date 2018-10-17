#ifndef AUTOAUTO_READER_H_
#define AUTOAUTO_READER_H_

#include "AutoAuto.h"

class AutoAutoReader{
public:
	AutoAutoReader();
	void initialize(const QString& directory, std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> autoautos_, std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> cars_);
	void save();
	void load();
private:
	std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> autoautos;
	std::string autoauto_dir;
	std::string autoauto_dir_tmp;
	std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> cars;
	std::vector<std::string> unknown;
};

#endif