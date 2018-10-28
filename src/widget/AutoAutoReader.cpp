#include "AutoAutoReader.h"
#include <iostream>
#include <fstream>
#include <QtCore/QDir>


AutoAutoReader::AutoAutoReader(){

}

void AutoAutoReader::initialize(const QString& directory, std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> autoautos_, std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> cars_){
	QDir base_dir(directory);
	autoauto_dir = base_dir.filePath("autoautos.txt").toStdString();
	autoauto_dir_tmp = base_dir.filePath("autoautos.txt.tmp").toStdString();
	autoautos = autoautos_;
	cars = cars_;
	unknown.clear();
}

void AutoAutoReader::save(){
	std::ofstream myfile;
	
	myfile.open (autoauto_dir_tmp.c_str());
	for(auto const& l:unknown){
		myfile << l << "\n";
	}
	for(auto const& a:(*autoautos)){
		myfile << a.second->getString() << "\n";
	}
	myfile.close();
	std::remove(autoauto_dir.c_str());
	std::rename(autoauto_dir_tmp.c_str(), autoauto_dir.c_str());
}
void AutoAutoReader::load(){
	std::ifstream infile(autoauto_dir.c_str());
	uint ctr=0;
	for( std::string line; getline( infile, line ); )
	{
	    try{
	    	auto autoautoptr = std::make_shared<AutoAuto>(cars, line);
	    	(*autoautos)[autoautoptr.get()] = (autoautoptr);
		} catch (const char* msg) {
			std::cout << msg << std::endl;
			unknown.push_back(line);
			continue;
		}
		ctr++;
	    // process pair (a,b)
	}
	std::cout<<ctr<<" cars loaded successfully"<<std::endl;
	infile.close();
}