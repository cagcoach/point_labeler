#include <stdint.h>
#include <QtGui/QImage>
#include <iostream>

// quick and dirty experiments.
int32_t main(int32_t argc, char** argv) {
  QImage img("/home/behley/data/kitti-odometry/dataset/sequences/00/image_2/000000.png");

  std::cout << qRed(img.pixel(0, 0)) << ", " << qGreen(img.pixel(0, 0)) << ", " << qBlue(img.pixel(0, 0)) << std::endl;

  std::cout << qRed(*(uint32_t*)(img.bits())) << std::endl;


  return 0;
}
