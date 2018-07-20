#include <stdint.h>
#include <widget/KittiReader.h>
#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include "rv/string_utils.h"

void KittiReader::initialize(const QString& directory) {
  velodyne_filenames_.clear();
  label_filenames_.clear();
  image_filenames_.clear();
  color_filenames_.clear();
  imageLabel_filenames_.clear();
  poses_.clear();

  QDir base_dir(directory);
  QDir velodyne_dir(base_dir.filePath("velodyne"));
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);
  for (int32_t i = 0; i < entries.size(); ++i) {
    velodyne_filenames_.push_back(velodyne_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir.exists("calib.txt"))
    throw std::runtime_error("Missing calibration file: " + base_dir.filePath("calib.txt").toStdString());

  calib_.initialize(base_dir.filePath("calib.txt").toStdString());

  readPoses(base_dir.filePath("poses.txt").toStdString(), poses_);

  // create label dir, etc.
  QDir labels_dir(base_dir.filePath("labels"));

  // find corresponding label files.
  if (!labels_dir.exists()) base_dir.mkdir("labels");

  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label";
    if (!labels_dir.exists(filename)) {
      std::ifstream in(velodyne_filenames_[i].c_str());
      in.seekg(0, std::ios::end);
      uint32_t num_points = in.tellg() / (4 * sizeof(float));
      in.close();

      std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());

      std::vector<uint32_t> labels(num_points, 0);
      out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));

      out.close();
    }

    label_filenames_.push_back(labels_dir.filePath(filename).toStdString());
  }

  std::string missing_img = QDir::currentPath().toStdString() + "/../assets/missing.png";
  QDir image_dir(base_dir.filePath("image_2"));
  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".png";
    if (image_dir.exists(filename)) {
      image_filenames_.push_back(image_dir.filePath(filename).toStdString());
    } else {
      image_filenames_.push_back(missing_img);
    }
  }

  QDir colors_dir(base_dir.filePath("color"));
  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".rgb";
    if (!colors_dir.exists(filename))
      color_filenames_.push_back("missing");
    else {
      color_filenames_.push_back(colors_dir.filePath(filename).toStdString());
    }
  }

  QDir imageLabel_dir(base_dir.filePath("image_labels"));
  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label";
    if (!colors_dir.exists(filename))
      imageLabel_filenames_.push_back("missing");
    else {
      imageLabel_filenames_.push_back(imageLabel_dir.filePath(filename).toStdString());
    }
  }

  // assumes that (0,0,0) is always the start.
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();

  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector4f t = poses_[i].col(3);

    min.x() = std::min(t.x() - maxDistance_, min.x());
    min.y() = std::min(t.y() - maxDistance_, min.y());
    max.x() = std::max(t.x() + maxDistance_, max.x());
    max.y() = std::max(t.y() + maxDistance_, max.y());
  }

  //  std::cout << "tileSize = " << tileSize_ << std::endl;
  //  std::cout << "min = " << min << ", max = " << max << std::endl;

  offset_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;
  offset_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;

  //  std::cout << "offset = " << offset_ << std::endl;

  numTiles_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.x() - 0.5 * tileSize_) / tileSize_) + 1;
  numTiles_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.y() - 0.5 * tileSize_) / tileSize_) + 1;

  //  std::cout << "numTiles = " << numTiles_ << std::endl;

  tiles_.clear();
  tiles_.resize(numTiles_.x() * numTiles_.y());

  Eigen::Vector2f idxRadius(maxDistance_ / tileSize_, maxDistance_ / tileSize_);

  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      tile.i = i;
      tile.j = j;
      tile.x = i * tileSize_ - offset_.x() + 0.5 * tileSize_;
      tile.y = j * tileSize_ - offset_.y() + 0.5 * tileSize_;
      tile.size = tileSize_;
    }
  }

  trajectory_.clear();

  Eigen::Vector2f e(0.5 * tileSize_, 0.5 * tileSize_);
  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector2f t = poses_[i].col(3).head(2);
    Eigen::Vector2f idx((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_);

    trajectory_.push_back(Eigen::Vector2f((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_));

    //    tiles_[tileIdxToOffset(uint32_t(idx.x()), uint32_t(idx.y()))].indexes.push_back(i);
    //    uint32_t u_min = std::max(int32_t(idx.x() - idxRadius.x()), 0);
    //    uint32_t u_max = std::min(int32_t(std::ceil(idx.x() + idxRadius.x())), numTiles_.x());
    //    uint32_t v_min = std::max(int32_t(idx.y() - idxRadius.y()), 0);
    //    uint32_t v_max = std::min(int32_t(std::ceil(idx.y() + idxRadius.y())), numTiles_.y());

    // FIXME: workaround check all tiles.
    for (uint32_t u = 0; u < uint32_t(numTiles_.x()); ++u) {
      for (uint32_t v = 0; v < uint32_t(numTiles_.y()); ++v) {
        auto& tile = tiles_[tileIdxToOffset(u, v)];
        Eigen::Vector2f q = t - Eigen::Vector2f(tile.x, tile.y);
        q[0] = std::abs(q[0]);
        q[1] = std::abs(q[1]);

        // check for exact overlap (see Behley et al., ICRA, 2015)
        if (std::max(q[0], q[1]) > e[0] + maxDistance_) continue;  // definitely outside.
        if (std::min(q[0], q[1]) < e[0] || (q - e).norm() < maxDistance_) {
          tile.indexes.push_back(i);
        }
      }
    }
  }

  // sanity check:

  for (auto& t : tiles_) {
    std::sort(t.indexes.begin(), t.indexes.end());
    //    std::cout << "Tile has " << t.indexes.size() << " tiles associated." << std::endl;
    for (uint32_t i = 1; i < t.indexes.size(); ++i) {
      if (t.indexes[i - 1] == t.indexes[i]) {
        std::cout << "found duplicate!" << std::endl;
      }
    }
  }

  uint32_t tileCount = 0;
  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      std::sort(tile.indexes.begin(), tile.indexes.end());
      if (tile.indexes.size() > 0) tileCount += 1;
    }
  }

  std::cout << "#tiles  = " << tileCount << std::endl;
}

void KittiReader::retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes,
                           std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels,
                           std::vector<std::string>& images, std::vector<ColorsPtr>& colors,
                           std::vector<LabelsPtr>& imageLabels) {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);

  retrieve(idx.x(), idx.y(), indexes, points, labels, images, colors, imageLabels);
}

void KittiReader::retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels, std::vector<std::string>& images,
                           std::vector<ColorsPtr>& colors, std::vector<LabelsPtr>& imageLabels) {
  indexes.clear();
  points.clear();
  labels.clear();
  images.clear();
  colors.clear();
  imageLabels.clear();

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  indexes = tiles_[tileIdxToOffset(i, j)].indexes;
  for (uint32_t t : indexes) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(velodyne_filenames_[t], *points.back());
      pointsCache_[t] = points.back();
      points.back()->pose = poses_[t];

      uint32_t num_points = points.back()->size();

      colors.push_back(std::shared_ptr<std::vector<glow::vec3>>(new std::vector<glow::vec3>()));
      colorCache_[t] = colors.back();
      if (color_filenames_[t] != "missing") {
        readColors(color_filenames_[t], *colors.back());
      } else {
        // fill with default values from remission.
        auto& vec = *colors.back();
        vec.resize(num_points);
        for (uint32_t i = 0; i < num_points; ++i) {
          float r = points.back()->remissions[i];
          vec[i] = glow::vec3(r, r, r);
        }
      }

      labels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      readLabels(label_filenames_[t], *labels.back());
      labelCache_[t] = labels.back();

      if (num_points != labels.back()->size()) {
        std::cout << "Filename: " << velodyne_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << points.back()->size() << " vs. num. labels = " << labels.back()->size()
                  << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

      imageLabels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      imageLabelCache_[t] = imageLabels.back();
      if (imageLabel_filenames_[t] != "missing") {
        readLabels(imageLabel_filenames_[t], *imageLabels.back());
      } else {
        // fill with unlabeled labels.
        auto& vec = *imageLabels.back();
        vec.resize(num_points);
        for (uint32_t i = 0; i < num_points; ++i) vec[i] = 0;
      }

    } else {
      points.push_back(pointsCache_[t]);
      labels.push_back(labelCache_[t]);
      colors.push_back(colorCache_[t]);
      imageLabels.push_back(imageLabelCache_[t]);
    }

    images.push_back(image_filenames_[t]);
  }

  std::cout << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }
}

const KittiReader::Tile& KittiReader::getTile(const Eigen::Vector3f& position) const {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);
  return tiles_[tileIdxToOffset(idx.x(), idx.y())];
}
const KittiReader::Tile& KittiReader::getTile(uint32_t i, uint32_t j) const { return tiles_[tileIdxToOffset(i, j)]; }

void KittiReader::setTileSize(float size) { tileSize_ = size; }

void KittiReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {
  for (uint32_t i = 0; i < indexes.size(); ++i) {
    if (labels[i]->size() == 0) {
      std::cout << "0 labels?" << std::endl;
      continue;
    }
    std::ofstream out(label_filenames_[indexes[i]].c_str());
    out.write((const char*)&(*labels[i])[0], labels[i]->size() * sizeof(uint32_t));
    out.close();
  }
}

void KittiReader::readPoints(const std::string& filename, Laserscan& scan) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) return;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points;
  std::vector<float>& remissions = scan.remissions;

  points.resize(num_points);
  remissions.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x = values[4 * i];
    points[i].y = values[4 * i + 1];
    points[i].z = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
  }
}

void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open label file. " << std::endl;
    return;
  }

  labels.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  labels.resize(num_points);
  in.read((char*)&labels[0], num_points * sizeof(uint32_t));

  in.close();
}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {
  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to velodyne coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }
}

void KittiReader::readColors(const std::string& filename, std::vector<glow::vec3>& colors) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open colors file. " << std::endl;
    return;
  }

  colors.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (3 * sizeof(uint8_t));
  in.seekg(0, std::ios::beg);

  colors.resize(num_points);
  std::vector<uint8_t> data(3 * num_points);
  in.read((char*)&data[0], data.size());

  for (uint32_t i = 0; i < num_points; ++i) {
    colors[i] = glow::vec3(data[3 * i] / 255.0f, data[3 * i + 1] / 255.0f, data[3 * i + 2] / 255.0f);
  }

  in.close();
}
