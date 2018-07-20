#include <QtCore/QDir>
#include <QtGui/QImage>
#include <QtGui/QPainter>
#include <QtGui/QPixmap>
#include <QtWidgets/QApplication>
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
  if (argc < 2) {
    std::cerr << "Usage: ./gen_colors <sequence folder>" << std::endl;

    return 1;
  }

  QApplication app(argc, argv);

  std::string directory = argv[1];

  QDir base_dir(QString::fromStdString(directory));
  QDir velodyne_dir(base_dir.filePath("velodyne"));
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);

  QDir image_dir(base_dir.filePath("image_2"));
  if (!image_dir.exists()) {
    std::cerr << "directory 'image_2' missing." << std::endl;
    return 1;
  }

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
  QDir colors_dir(base_dir.filePath("color"));
  if (!colors_dir.exists()) base_dir.mkdir("color");

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

    std::vector<uint8_t> colors(3 * points.size(), 0);
    // default colors are just the remissions as gray value.
//    for (uint32_t j = 0; j < remissions.size(); ++j) {
//      colors[3 * j] = colors[3 * j + 1] = colors[3 * j + 2] = 0;
//    }

    QString img_filename = QFileInfo(QString::fromStdString(velodyne_filenames[i])).baseName() + ".png";
    if (image_dir.exists(img_filename)) {
      // project points and get color from image.
      QImage img(image_dir.filePath(img_filename));
      //      QPixmap pix(image_dir.filePath(img_filename));
      //      QPainter painter(&pix);
      //      uint32_t count = 0;
      for (uint32_t j = 0; j < points.size(); ++j) {
        Eigen::Vector4f pos = Tr * points[j];
        if (pos.z() < 0) continue;

        Eigen::Vector3f coords = P2 * pos;
        coords.x() /= coords.z();
        coords.y() /= coords.z();
        if (coords.x() < 0 || coords.y() < 0 || coords.x() >= img.width() || coords.y() >= img.height()) continue;

        QRgb col = img.pixel(coords.x(), coords.y());
        colors[3 * j] = uint8_t(qRed(col));
        colors[3 * j + 1] = uint8_t(qGreen(col));
        colors[3 * j + 2] = uint8_t(qBlue(col));

        //        count++;
        //        painter.drawPoint(coords.x(), coords.y());
      }
      //      std::cout << count << " Points projected inside the image." << std::endl;
      //      painter.end();
      //      pix.save("projections.png");
    }

    QString col_filename = QFileInfo(QString::fromStdString(velodyne_filenames[i])).baseName() + ".rgb";
    std::ofstream out(colors_dir.filePath(col_filename).toStdString().c_str());
    out.write(reinterpret_cast<const char*>(colors.data()), colors.size());
    out.close();
  }

  while (progress <= 100) {
    std::cout << progress << " " << std::flush;
    progress += 10;
  }

  std::cout << " finished." << std::endl;

  return 0;
}
