#include <QtCore/QDir>
#include <QtGui/QImage>
#include <fstream>
#include <iostream>
#include "data/kitti_utils.h"

void readPoints(const std::string& filename, std::vector<Eigen::Vector4f>& points, std::vector<float>& remissions) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) return;

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();

  points.clear();
  remissions.clear();

  points.resize(num_points);
  remissions.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i] = Eigen::Vector4f(values[4 * i], values[4 * i + 1], values[4 * i + 2], 1.0f);
    remissions[i] = values[4 * i + 3];
  }
}

int32_t main(int32_t argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: ./gen_labels <sequence folder> <imagelabel folder>" << std::endl;

    return 1;
  }

  std::string directory = argv[1];
  std::string input_label_dir = argv[2];

  QDir base_dir(QString::fromStdString(directory));
  QDir velodyne_dir(base_dir.filePath("velodyne"));
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);

  QDir label_dir(QString::fromStdString(input_label_dir));

  std::vector<std::string> velodyne_filenames;
  for (int32_t i = 0; i < entries.size(); ++i) {
    velodyne_filenames.push_back(velodyne_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir.exists("calib.txt"))
    throw std::runtime_error("Missing calibration file: " + base_dir.filePath("calib.txt").toStdString());

  KITTICalibration calib;
  calib.initialize(base_dir.filePath("calib.txt").toStdString());

  Eigen::Matrix4f Tr = calib["Tr"];
  Eigen::Matrix<float, 3, 4> P2 = calib["P2"].topLeftCorner(3, 4);

  // create label dir, etc.
  QDir colors_dir(base_dir.filePath("image_labels"));
  if (!colors_dir.exists()) base_dir.mkdir("image_labels");

  std::vector<Eigen::Vector4f> points;
  std::vector<float> remissions;

  std::cout << "Processing: " << std::flush;
  uint32_t progress = 10.0f;
  for (uint32_t i = 0; i < velodyne_filenames.size(); ++i) {
    if (100.0f * i / velodyne_filenames.size() > progress) {
      std::cout << progress << " " << std::flush;
      progress += 10;
    }
    readPoints(velodyne_filenames[i], points, remissions);

    std::vector<uint32_t> labels(points.size(), 0);

    QString img_filename = QFileInfo(QString::fromStdString(velodyne_filenames[i])).baseName() + ".png";
    if (label_dir.exists(img_filename)) {
      // project points and get color from image.
      QImage img(label_dir.filePath(img_filename));
      for (uint32_t j = 0; j < points.size(); ++j) {
        Eigen::Vector4f pos = Tr * points[j];
        if (pos.z() < 0) continue;

        Eigen::Vector3f coords = P2 * pos;
        coords.x() /= coords.z();
        coords.y() /= coords.z();

        if (coords.x() < 0 || coords.y() < 0 || coords.x() >= img.width() || coords.y() >= img.height()) continue;
        QRgb col = img.pixel(coords.x(), coords.y());
        uint32_t label = uint8_t(qRed(col));

        if (label == 0) labels[j] = 40;
        if (label == 1) labels[j] = 48;
        if (label == 2) labels[j] = 50;
        if (label == 4) labels[j] = 51;
        if (label == 13) labels[j] = 10;
        if (label == 8) labels[j] = 70;
        if (label == 11) labels[j] = 30;
        if (label == 18) labels[j] = 11;
        if (label == 17) labels[j] = 15;
      }
    }

    QString col_filename = QFileInfo(QString::fromStdString(velodyne_filenames[i])).baseName() + ".label";
    std::ofstream out(colors_dir.filePath(col_filename).toStdString().c_str());
    out.write(reinterpret_cast<const char*>(labels.data()), labels.size() * sizeof(uint32_t));
    out.close();
  }

  while (progress <= 100) {
    std::cout << progress << " " << std::flush;
    progress += 10;
  }

  std::cout << " finished." << std::endl;

  return 0;
}
