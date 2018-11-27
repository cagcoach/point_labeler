#include "Viewport.h"
#include "data/Math.h"
#include "data/draw_utils.h"
#include "data/misc.h"
#include "libicp/icpPointToPoint.h"
#include "CarDialog.h"
#include "Car.h"
#include "MovingCar.h"
#include "CarFactory.h"

#include <glow/GlCapabilities.h>
#include <glow/ScopedBinder.h>
#include <glow/glutil.h>
#include <QtWidgets/QMessageBox>
#include <algorithm>
#include <chrono>
#include <fstream>
#include "rv/Stopwatch.h"

#include <glow/GlState.h>
//#include <voro/voro++.hh>

#include <QApplication>
#include <QEventLoop>
#include <bits/stdc++.h> 
#include <math.h>       /* atan */

//#define PI 3.14159265

using namespace glow;
using namespace rv;


Viewport::Viewport(QWidget* parent, Qt::WindowFlags f)
    : QGLWidget(parent, 0, f),
      parent(parent),
      contextInitialized_(initContext()),
      mAxis(XYZ),
      mMode(NONE),
      mFlags(FLAG_OVERWRITE),
      mCurrentLabel(0),
      mRadius(5),
      buttonPressed(false),
      texLabelColors_(1024, 1, TextureFormat::RGB),
      fbMinimumHeightMap_(100, 100),
      texMinimumHeightMap_(100, 100, TextureFormat::R_FLOAT),
      texTempHeightMap_(100, 100, TextureFormat::R_FLOAT),
      texTriangles_(3 * 100, 1, TextureFormat::RGB) {
  connect(&timer_, &QTimer::timeout, [this]() { this->updateGL(); });
  connect(this, SIGNAL(scanChanged()),this, SLOT(updateCarsInWorld()));

  //  setMouseTracking(true);

  drawingOption_["coordinate axis"] = true;

  conversion_ = RoSe2GL::matrix;

  // FIXME: make max_size depending on memory. (have setting like AvailableMemory that specifies how much memory should
  // be used by program!)
  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
  bufScanIndexes_.resize(max_size);

  bufTempPoints_.reserve(maxPointsPerScan_);
  bufTempRemissions_.reserve(maxPointsPerScan_);
  bufTempLabels_.reserve(maxPointsPerScan_);
  bufTempVisible_.reserve(maxPointsPerScan_);

  uint32_t tempMem = bufTempPoints_.memorySize();
  tempMem += bufTempRemissions_.memorySize();
  tempMem += bufTempLabels_.memorySize();
  tempMem += bufTempVisible_.memorySize();

  std::cout << "temp mem size: " << float(tempMem) / (1000 * 1000) << " MB" << std::endl;

  bufPolygonPoints_.reserve(100);

  bufUpdatedLabels_.resize(maxPointsPerScan_);
  tfUpdateLabels_.attach({"out_label"}, bufUpdatedLabels_);

  bufSelectedPoly_.resize(maxPointsPerScan_);
  tfSelectPoly_.attach({"out_inpoly"}, bufSelectedPoly_);

  bufSelectedBox_.resize(maxPointsPerScan_);
  tfSelectBox_.attach({"out_inbox"}, bufSelectedBox_);


  bufUpdatedVisiblity_.resize(maxPointsPerScan_);
  tfUpdateVisibility_.attach({"out_visible"}, bufUpdatedVisiblity_);

  tfFillTilePoints_.attach({"out_point"}, bufPoints_);
  tfFillTilePoints_.attach({"out_label"}, bufLabels_);
  tfFillTilePoints_.attach({"out_visible"}, bufVisible_);
  tfFillTilePoints_.attach({"out_scanindex"}, bufScanIndexes_);

  initPrograms();
  initVertexBuffers();

  drawingOption_["remission"] = true;
  drawingOption_["color"] = false;
  drawingOption_["single scan"] = false;
  drawingOption_["show all points"] = false;
  drawingOption_["draw heightmap"] = false;
  drawingOption_["draw triangles"] = false;
  drawingOption_["show plane"] = true;

  texLabelColors_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texLabelColors_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texTriangles_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texTriangles_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  texMinimumHeightMap_.setMinifyingOperation(TexMinOp::NEAREST);
  texMinimumHeightMap_.setMagnifyingOperation(TexMagOp::NEAREST);

  texTempHeightMap_.setMinifyingOperation(TexMinOp::NEAREST);
  texTempHeightMap_.setMagnifyingOperation(TexMagOp::NEAREST);

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
  GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                             RenderbufferFormat::DEPTH_STENCIL);
  fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);



  setAutoFillBackground(false);

  cameras_["Default"] = std::make_shared<RoSeCamera>();
  cameras_["CAD"] = std::make_shared<CADCamera>();
  cameras_["CAD2"] = std::make_shared<CADCamera2>(cameraRefPt,projection_,this);
  mCamera = cameras_["Default"];

  glow::_CheckGlError(__FILE__, __LINE__);
}

Viewport::~Viewport() {
  // workaround for strange behaviour with my Nvidia 860m. Even though the buffers fit into memory, they mess up
  // something up. However, I noticed that it does not cause havoc if the buffers are empty or small.

  bufPoints_.assign(std::vector<vec4>());
  bufLabels_.assign(std::vector<uint32_t>());
  bufScanIndexes_.assign(std::vector<vec2>());
  bufVisible_.assign(std::vector<uint32_t>());
}


void Viewport::clear() {
  carsInWorldIdx = -1;
  autoautos->clear();
  carsInWorld_.clear();
  singleScanIdx_=0;
  updateCarsInWorldMutex.unlock();


}
void Viewport::initPrograms() {
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_points.vert"));
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPoints_.link();

  prgDrawCarPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_car_points.vert"));
  prgDrawCarPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawCarPoints_.link();

  prgDrawPose_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_pose.geom"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPose_.link();

  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_labels.vert"));
  prgUpdateLabels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgUpdateLabels_.attach(tfUpdateLabels_);
  prgUpdateLabels_.link();

  prgUpdateVisibility_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/update_visibility.vert"));
  prgUpdateVisibility_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgUpdateVisibility_.attach(tfUpdateVisibility_);
  prgUpdateVisibility_.link();

  prgPolygonPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_polygon.vert"));
  prgPolygonPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgPolygonPoints_.link();

  prgMinimumHeightMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/gen_heightmap.vert"));
  prgMinimumHeightMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/gen_heightmap.frag"));
  prgMinimumHeightMap_.link();

  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/fill_tile_points.vert"));
  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/fill_tile_points.geom"));
  prgFillTilePoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgFillTilePoints_.attach(tfFillTilePoints_);
  prgFillTilePoints_.link();

  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_frustum.geom"));
  prgDrawFrustum_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawFrustum_.link();

  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_heightmap.vert"));
  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_heightmap.geom"));
  prgDrawHeightmap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawHeightmap_.link();

  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/quad.geom"));
  prgAverageHeightMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/average_heightmap.frag"));
  prgAverageHeightMap_.link();


  prgSelectPoly_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/select_poly.vert"));
  prgSelectPoly_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgSelectPoly_.attach(tfSelectPoly_);
  prgSelectPoly_.link();

  prgSelectBox_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/select_box.vert"));
  prgSelectBox_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/empty.frag"));
  prgSelectBox_.attach(tfSelectBox_);
  prgSelectBox_.link();


  first_cars_ = AutoAuto::loadCarModels("../cars");
  more_cars_ = AutoAuto::loadCarModels("../cars/more");
  cars_ = std::make_shared<std::map<std::string, std::shared_ptr<Car>>>();
  cars_->insert(first_cars_->begin(),first_cars_->end());
  cars_->insert(more_cars_->begin(),more_cars_->end());
  autoautos = std::make_shared<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>>();

  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_plane.geom"));
  prgDrawPlane_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPlane_.link();


  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 4, AttributeType::FLOAT, false, sizeof(glow::vec4), nullptr);
  vao_points_.setVertexAttribute(1, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(2, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);

  vao_temp_points_.setVertexAttribute(0, bufTempPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_temp_points_.setVertexAttribute(1, bufTempRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_temp_points_.setVertexAttribute(2, bufTempLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t),
                                      nullptr);
  vao_temp_points_.setVertexAttribute(3, bufTempVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t),
                                      nullptr);

  vao_polygon_points_.setVertexAttribute(0, bufPolygonPoints_, 2, AttributeType::FLOAT, false, sizeof(vec2), nullptr);

  vao_triangles_.setVertexAttribute(0, bufTriangles_, 2, AttributeType::FLOAT, false, sizeof(vec2), nullptr);

  vao_heightmap_points_.setVertexAttribute(0, bufHeightMapPoints_, 2, AttributeType::FLOAT, false, sizeof(glow::vec2),
                                           nullptr);

  vao_car_points_.setVertexAttribute(0, bufCarPoints_, 4, AttributeType::FLOAT, false, sizeof(glow::vec4), nullptr);
}

/** \brief set axis fixed (x = 1, y = 2, z = 3) **/
void Viewport::setFixedAxis(AXIS axis) { mAxis = axis; }

void Viewport::setMaximumScans(uint32_t numScans) {
  maxScans_ = numScans;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
  bufScanIndexes_.resize(max_size);

  uint32_t memTile = 0;  // = bufPoses_.memorySize();
  memTile += bufPoints_.memorySize();
  memTile += bufVisible_.memorySize();
  memTile += bufLabels_.memorySize();
  memTile += bufScanIndexes_.memorySize();

  std::cout << "mem size: " << float(memTile) / (1000 * 1000) << " MB" << std::endl;
}

void Viewport::setPoints(const std::vector<PointcloudPtr>& p, std::vector<LabelsPtr>& l) {
  std::cout << "Setting points..." << std::flush;

  glow::_CheckGlError(__FILE__, __LINE__);

  points_ = p;
  labels_ = l;

  //  Stopwatch::tic();

  {
    ScopedBinder<GlVertexArray> vaoBinder(vao_temp_points_);
    ScopedBinder<GlProgram> programBinder(prgFillTilePoints_);
    ScopedBinder<GlTransformFeedback> feedbackBinder(tfFillTilePoints_);

    glow::_CheckGlError(__FILE__, __LINE__);

    prgFillTilePoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
    prgFillTilePoints_.setUniform(GlUniform<float>("minRange", minRange_));
    prgFillTilePoints_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgFillTilePoints_.setUniform(GlUniform<float>("tileSize", tileSize_));
    prgFillTilePoints_.setUniform(GlUniform<float>("tileBoundary", tileBoundary_));

    glEnable(GL_RASTERIZER_DISCARD);

    tfFillTilePoints_.begin(TransformFeedbackMode::POINTS);

    for (uint32_t t = 0; t < points_.size(); ++t) {
      prgFillTilePoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
      prgFillTilePoints_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[t]->pose));

      uint32_t num_points = points_[t]->size();

      std::vector<uint32_t> visible(num_points, 1);

      for (uint32_t i = 0; i < num_points; ++i) {
        if (std::find(mFilteredLabels.begin(), mFilteredLabels.end(), (*labels_[t])[i]) != mFilteredLabels.end()) {
          visible[i] = 0;
        }
      }

      // copy data from CPU -> GPU.
      bufTempPoints_.assign(points_[t]->points);
      if (points_[t]->hasRemissions())
        bufTempRemissions_.assign(points_[t]->remissions);
      else
        bufTempRemissions_.assign(std::vector<float>(points_[t]->size(), 1.0f));
      bufTempLabels_.assign(*(labels_[t]));
      bufTempVisible_.assign(visible);

      prgFillTilePoints_.setUniform(GlUniform<uint32_t>("scan", t));

      // extract tile points.
      glDrawArrays(GL_POINTS, 0, points_[t]->size());
    }

    uint32_t numCopiedPoints = tfFillTilePoints_.end();

    glDisable(GL_RASTERIZER_DISCARD);

    bufPoints_.resize(numCopiedPoints);
    bufLabels_.resize(numCopiedPoints);
    bufVisible_.resize(numCopiedPoints);
    bufScanIndexes_.resize(numCopiedPoints);

    // get per scan information from  scan indexes.
    scanInfos_.clear();
    scanInfos_.resize(points_.size());
    for (auto& info : scanInfos_) {
      info.start = 0;
      info.size = 0;
    }

    glow::_CheckGlError(__FILE__, __LINE__);

    std::vector<glow::vec2> scanIndexes;
    glow::GlBuffer<glow::vec2> bufReadBuffer{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
    bufReadBuffer.resize(maxPointsPerScan_);

    ScanInfo current;
    int32_t currentScanIndex{-1};

    uint32_t count = 0;
    while (count * maxPointsPerScan_ < bufScanIndexes_.size()) {
      uint32_t read_size = std::min<uint32_t>(maxPointsPerScan_, bufScanIndexes_.size() - count * maxPointsPerScan_);
      bufScanIndexes_.copyTo(count * maxPointsPerScan_, read_size, bufReadBuffer, 0);
      bufReadBuffer.get(scanIndexes, 0, read_size);

      for (uint32_t i = 0; i < read_size; ++i) {
        if (currentScanIndex != int32_t(scanIndexes[i].x)) {
          if (currentScanIndex > -1) scanInfos_[currentScanIndex] = current;
          current.start = count * maxPointsPerScan_ + i;
          current.size = 0;
          currentScanIndex = uint32_t(scanIndexes[i].x);
        }
        current.size += 1;
      }

      ++count;
    }

    std::cout << "copied " << scanInfos_.size() << " scans with " << numCopiedPoints << " points" << std::endl;
    if (numCopiedPoints == bufPoints_.capacity()) {
      QMessageBox::warning(this, "Increase number of scans.",
                           "Its possible that not all scans could be loaded. Please increase the 'max scans' in "
                           "the settings.cfg, but ensure that enough GPU memory is available.");
    }
  }

  glow::_CheckGlError(__FILE__, __LINE__);

  updateLabels();
  updateGL();
}

void Viewport::updateHeightmap() {
  // generate height map.
  if (points_.size() == 0) return;

  uint32_t width = tileSize_ / groundResolution_;
  uint32_t height = tileSize_ / groundResolution_;

  std::vector<glow::vec2> indexes(width * height);
  for (uint32_t x = 0; x < width; ++x) {
    for (uint32_t y = 0; y < height; ++y) {
      indexes[x + y * width] = vec2(float(x + 0.5f) / width, float(y + 0.5f) / height);
    }
  }

  //  std::cout << "w x h: " << width << " x " << height << std::endl;

  //  std::cout << indexes[0] << ", " << indexes[10] << std::endl;
  bufHeightMapPoints_.assign(indexes);

  if (fbMinimumHeightMap_.width() != width || fbMinimumHeightMap_.height() != height) {
    fbMinimumHeightMap_.resize(width, height);
    texMinimumHeightMap_.resize(width, height);
    texTempHeightMap_.resize(width, height);

    // update also depth buffer.
    GlRenderbuffer depthbuffer(fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height(),
                               RenderbufferFormat::DEPTH_STENCIL);
    fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texTempHeightMap_);
    fbMinimumHeightMap_.attach(FramebufferAttachment::DEPTH_STENCIL, depthbuffer);
  }

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texTempHeightMap_);

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);

  glPointSize(1.0f);

  glViewport(0, 0, fbMinimumHeightMap_.width(), fbMinimumHeightMap_.height());

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  fbMinimumHeightMap_.bind();
  prgMinimumHeightMap_.bind();
  vao_points_.bind();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  prgMinimumHeightMap_.setUniform(GlUniform<float>("minHeight", -3.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("maxHeight", 5.0f));
  prgMinimumHeightMap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgMinimumHeightMap_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgMinimumHeightMap_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[0]->pose));

  glDrawArrays(GL_POINTS, 0, bufPoints_.size());

  prgMinimumHeightMap_.release();
  vao_points_.release();

  fbMinimumHeightMap_.attach(FramebufferAttachment::COLOR0, texMinimumHeightMap_);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);

  vao_no_points_.bind();
  prgAverageHeightMap_.bind();
  glActiveTexture(GL_TEXTURE0);
  texTempHeightMap_.bind();

  glDrawArrays(GL_POINTS, 0, 1);

  texTempHeightMap_.release();
  prgAverageHeightMap_.release();
  vao_no_points_.release();
  fbMinimumHeightMap_.release();

  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glDepthFunc(GL_LEQUAL);

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::updateLabels() {
  glow::_CheckGlError(__FILE__, __LINE__);

  if (labels_.size() == 0) return;

  glow::GlBuffer<uint32_t> bufReadLabels{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  glow::GlBuffer<vec2> bufReadIndexes{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STREAM_READ};
  bufReadLabels.resize(maxPointsPerScan_);
  bufReadIndexes.resize(bufReadLabels.size());

  std::vector<uint32_t> labels(bufReadLabels.size());
  std::vector<vec2> indexes(bufReadIndexes.size());

  uint32_t count = 0;
  uint32_t max_size = bufReadLabels.size();
  uint32_t buffer_size = bufLabels_.size();

  labeledCount_ = 0;

  while (count * max_size < bufLabels_.size()) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    bufLabels_.copyTo(count * max_size, size, bufReadLabels, 0);
    bufScanIndexes_.copyTo(count * max_size, size, bufReadIndexes, 0);

    bufReadLabels.get(labels, 0, size);
    bufReadIndexes.get(indexes, 0, size);

    for (uint32_t i = 0; i < size; ++i) {
      uint32_t scanidx = indexes[i].x;
      uint32_t idx = indexes[i].y;
      (*labels_[scanidx])[idx] = labels[i];
      labeledCount_ += (labels[i] != 0);
    }

    count++;
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::setRadius(float value) { mRadius = value; }

void Viewport::setLabel(uint32_t label) { mCurrentLabel = label; }

void Viewport::setLabelColors(const std::map<uint32_t, glow::GlColor>& colors) {
  mLabelColors = colors;

  std::vector<uint8_t> label_colors(3 * 1024, 0);
  for (auto it = mLabelColors.begin(); it != mLabelColors.end(); ++it) {
    if (it->first > 1023) {
      throw std::runtime_error("currently only labels up to 1024 are supported.");
    }
    label_colors[3 * it->first] = it->second.R * 255;
    label_colors[3 * it->first + 1] = it->second.G * 255;
    label_colors[3 * it->first + 2] = it->second.B * 255;
  }
  texLabelColors_.assign(PixelFormat::RGB, PixelType::UNSIGNED_BYTE, &label_colors[0]);
}

void Viewport::setPointSize(int value) {
  pointSize_ = value;
  updateGL();
}

void Viewport::setMode(MODE mode) {
  mMode = mode;

  updateGL();
}

void Viewport::setFlags(int32_t flags) { mFlags = flags; }

void Viewport::setOverwrite(bool value) {
  if (value)
    mFlags = mFlags | FLAG_OVERWRITE;
  else
    mFlags = mFlags & ~FLAG_OVERWRITE;
}

void Viewport::setDrawingOption(const std::string& name, bool value) {
  drawingOption_[name] = value;
  updateGL();
}

void Viewport::setMinRange(float range) { minRange_ = range; }

void Viewport::setMaxRange(float range) { maxRange_ = range; }

void Viewport::setFilteredLabels(const std::vector<uint32_t>& labels) {
  std::vector<uint32_t> labels_before = mFilteredLabels;
  mFilteredLabels = labels;
  std::sort(mFilteredLabels.begin(), mFilteredLabels.end());

  std::vector<uint32_t> difference(std::max(labels_before.size(), labels.size()));
  auto end = std::set_difference(labels_before.begin(), labels_before.end(), mFilteredLabels.begin(),
                                 mFilteredLabels.end(), difference.begin());

  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 1);  // now visible again.

  end = std::set_difference(mFilteredLabels.begin(), mFilteredLabels.end(), labels_before.begin(), labels_before.end(),
                            difference.begin());

  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 0);  // now invisible.

  updateGL();
}

void Viewport::setGroundRemoval(bool value) {
  removeGround_ = value;

  if (removeGround_) updateHeightmap();

  updateGL();
}

void Viewport::setGroundThreshold(float value) {
  groundThreshold_ = value;
  updateGL();
}

void Viewport::setScanIndex(uint32_t idx) {
  singleScanIdx_ = idx;
  updateGL();
  emit scanChanged();
}

void Viewport::updateCarsInWorld(){
int idx=singleScanIdx_;
updateCarsInWorldMutex.lock();
if (carsInWorldIdx != idx){
  for(auto& a:carsInWorld_){
    a.second = *(a.first->getResults()[a.first->getSelectedCar()]->getGlobalPoints(idx));
    if (idx!=singleScanIdx_) break;
  }
}
updateAutoAuto();
updateCarsInWorldMutex.unlock();
}

void Viewport::setLabelVisibility(uint32_t label, bool visible) {
  // update.

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateVisibility_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateVisibility_);

  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("label", label));
  prgUpdateVisibility_.setUniform(GlUniform<uint32_t>("visibility", visible ? 1 : 0));

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufUpdatedVisiblity_.size();
  while (count * max_size < bufVisible_.size()) {
    uint32_t size = std::min<uint32_t>(max_size, bufVisible_.size() - count * max_size);

    tfUpdateVisibility_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, count * max_size, size);
    tfUpdateVisibility_.end();

    bufUpdatedVisiblity_.copyTo(0, size, bufVisible_, count * max_size);
    ++count;
  }

  glDisable(GL_RASTERIZER_DISCARD);
}

void Viewport::initializeGL() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LINE_SMOOTH);

  mCamera->lookAt(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
}

void Viewport::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  float aspect = float(w) / float(h);

  if (projectionMode_ == CameraProjection::perspective) {
    float fov = radians(45.0f);
    *projection_ = glPerspective(fov, aspect, 0.1f, 2000.0f);
  } else {
    float fov = 10.0f;
    if (w <= h)
      *projection_ = glOrthographic(-fov, fov, -fov / aspect, fov / aspect, 0.1f, 2000.0f);
    else
      *projection_ = glOrthographic(-fov * aspect, fov * aspect, -fov, fov, 0.1, 2000.0f);
  }
}

void Viewport::paintGL() {
  glow::_CheckGlError(__FILE__, __LINE__);

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(pointSize_);

  view_ = mCamera->matrix();

  mvp_ = *projection_ * view_ * conversion_;

  if (drawingOption_["coordinate axis"]) {
    // coordinateSytem_->pose = Eigen::Matrix4f::Identity();
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
    prgDrawPose_.setUniform(GlUniform<float>("size", 1.0f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  bool showSingleScan = drawingOption_["single scan"];
  bool showScanRange = drawingOption_["show scan range"];

  if (points_.size() > 0) {
    glPointSize(pointSize_);

    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);

    prgDrawPoints_.setUniform(GlUniform<bool>("useRemission", drawingOption_["remission"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("useColor", drawingOption_["color"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("removeGround", removeGround_));
    prgDrawPoints_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
    prgDrawPoints_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgDrawPoints_.setUniform(GlUniform<float>("tileSize", tileSize_));
    prgDrawPoints_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
    prgDrawPoints_.setUniform(GlUniform<int32_t>("heightMap", 1));

    //    float planeThreshold = planeThreshold_;
    //    prgDrawPoints_.setUniform(GlUniform<bool>("planeRemoval", planeRemoval_));
    //    prgDrawPoints_.setUniform(GlUniform<int32_t>("planeDimension", planeDimension_));
    //    if (planeDimension_ == 0) planeThreshold += tilePos_.x;
    //    if (planeDimension_ == 1) planeThreshold += tilePos_.y;
    //    if (planeDimension_ == 2 && points_.size() > 0) planeThreshold += points_[0]->pose(3, 3);
    //
    //    prgDrawPoints_.setUniform(GlUniform<float>("planeThreshold", planeThreshold));
    //    prgDrawPoints_.setUniform(GlUniform<float>("planeDirection", planeDirection_));

    prgDrawPoints_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
    prgDrawPoints_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
    prgDrawPoints_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
    prgDrawPoints_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));
    //    prgDrawPoints_.setUniform(GlUniform<bool>("carAsBase", drawingOption_["carAsBase"]));
    Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();
    plane_pose(0, 3) = tilePos_.x;
    plane_pose(1, 3) = tilePos_.y;
    plane_pose(2, 3) = points_[0]->pose(2, 3);
    if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
      plane_pose = points_[singleScanIdx_]->pose;
      //      plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
    }

    prgDrawPoints_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.bind();

    prgDrawPoints_.setUniform(mvp_);

    if (showSingleScan)
      glDrawArrays(GL_POINTS, scanInfos_[singleScanIdx_].start, scanInfos_[singleScanIdx_].size);
    else if (showScanRange) {
      uint32_t start = scanInfos_[scanRangeBegin_].start;
      uint32_t end = scanInfos_[scanRangeEnd_].start + scanInfos_[scanRangeEnd_].size;
      glDrawArrays(GL_POINTS, start, end);
    } else
      glDrawArrays(GL_POINTS, 0, bufPoints_.size());

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
    glActiveTexture(GL_TEXTURE1);
    texMinimumHeightMap_.release();
  }

  if (planeRemovalNormal_ && drawingOption_["show plane"]) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // FIXME: state changed.

    ScopedBinder<GlProgram> program_binder(prgDrawPlane_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPlane_.setUniform(mvp_);

    //    prgDrawPlane_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
    //    if (points_.size() > singleScanIdx_)
    //      prgDrawPlane_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));

    //    float planeThreshold = planeThreshold_;
    //    prgDrawPlane_.setUniform(GlUniform<bool>("planeRemoval", planeRemoval_));
    //    prgDrawPlane_.setUniform(GlUniform<int32_t>("planeDimension", planeDimension_));
    //    if (planeDimension_ == 0) planeThreshold += tilePos_.x;
    //    if (planeDimension_ == 1) planeThreshold += tilePos_.y;
    //    if (planeDimension_ == 2 && points_.size() > 0) planeThreshold += points_[0]->pose(3, 3);
    //    prgDrawPlane_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
    //    prgDrawPlane_.setUniform(GlUniform<float>("planeThreshold", planeThreshold));
    //    prgDrawPlane_.setUniform(GlUniform<float>("planeDirection", planeDirection_));
    //    prgDrawPlane_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
    //    prgDrawPlane_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    //    prgDrawPlane_.setUniform(GlUniform<float>("tileSize", tileSize_));

    prgDrawPlane_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
    prgDrawPlane_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
    prgDrawPlane_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));
    prgDrawPlane_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
    //    prgDrawPoints_.setUniform(GlUniform<bool>("carAsBase", drawingOption_["carAsBase"]));
    Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();

    plane_pose(0, 3) = tilePos_.x;
    plane_pose(1, 3) = tilePos_.y;
    if (points_.size() > 0) plane_pose(2, 3) = points_[0]->pose(2, 3);
    if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
      plane_pose = points_[singleScanIdx_]->pose;
      //      plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
    }

    prgDrawPlane_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));
    glDrawArrays(GL_POINTS, 0, 1);

    glDisable(GL_BLEND);
  }

  if (showSingleScan) {
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));
    prgDrawPose_.setUniform(GlUniform<float>("size", 0.5f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (drawingOption_["show camera"]) {
    ScopedBinder<GlProgram> program_binder(prgDrawFrustum_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawFrustum_.setUniform(mvp_);
    prgDrawFrustum_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));
    prgDrawFrustum_.setUniform(GlUniform<int32_t>("width", 1241));
    prgDrawFrustum_.setUniform(GlUniform<int32_t>("height", 376));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (drawingOption_["draw heightmap"]) {
    ScopedBinder<GlProgram> program_binder(prgDrawHeightmap_);
    ScopedBinder<GlVertexArray> vao_binder(vao_heightmap_points_);
    glActiveTexture(GL_TEXTURE0);
    texMinimumHeightMap_.bind();

    prgDrawHeightmap_.setUniform(mvp_);
    prgDrawHeightmap_.setUniform(GlUniform<float>("ground_resolution", groundResolution_));
    prgDrawHeightmap_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
    prgDrawHeightmap_.setUniform(GlUniform<float>("tileSize", tileSize_));

    glDrawArrays(GL_POINTS, 0, bufHeightMapPoints_.size());

    texMinimumHeightMap_.release();
  }

  //Draw Cars
  ScopedBinder<GlProgram> program_binder(prgDrawCarPoints_);
  ScopedBinder<GlVertexArray> vao_binder(vao_car_points_);

  prgDrawCarPoints_.setUniform(mvp_);
  glDrawArrays(GL_POINTS, 0, bufCarPoints_.size());


  glDisable(GL_DEPTH_TEST);
  // Important: QPainter is apparently not working with OpenGL Core Profile < Qt5.9!!!!
  //  http://blog.qt.io/blog/2017/01/27/opengl-core-profile-context-support-qpainter/
  //  QPainter painter(this); // << does not work with OpenGL Core Profile.
  if (mMode == POLYGON || mMode == AUTOAUTO ) {
    ScopedBinder<GlProgram> program_binder(prgPolygonPoints_);

    vao_polygon_points_.bind();

    prgPolygonPoints_.setUniform(GlUniform<int32_t>("width", width()));
    prgPolygonPoints_.setUniform(GlUniform<int32_t>("height", height()));
    prgPolygonPoints_.setUniform(GlUniform<uint32_t>("label", mCurrentLabel));

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    glDrawArrays(GL_LINE_LOOP, 0, bufPolygonPoints_.size());

    glPointSize(7.0f);

    glDrawArrays(GL_POINTS, 0, bufPolygonPoints_.size());
    vao_polygon_points_.release();

    if (drawingOption_["draw triangles"]) {
      vao_triangles_.bind();

      glDrawArrays(GL_LINES, 0, bufTriangles_.size());

      vao_triangles_.release();
    }
    texLabelColors_.release();
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

std::ostream& operator<<(std::ostream& os, const vec2& v) {
  os << "(" << v.x << ", " << v.y << ")";
  return os;
}

void Viewport::wheelEvent(QWheelEvent* event) {
  mChangeCamera = false;

  if (event->modifiers() == Qt::ControlModifier || mMode == PAINT || polygonPoints_.empty()) {
    *cameraRefPt = Eigen::Vector4f(2*event->pos().x()/(float)width()-1,-2*event->pos().y()/(float)height()+1,1,1);
    QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8.;
    float delta = 0.0f;

    if (!numPixels.isNull()) {
      delta = numPixels.y();
    } else if (!numDegrees.isNull()) {
      delta = numDegrees.y() / 15.;
    }

    mCamera->wheelEvent(delta, resolveKeyboardModifier(event->modifiers()));
    mChangeCamera = true;
    polygonPoints_.clear();  // start over again.#
    bufPolygonPoints_.assign(polygonPoints_);
    bufTriangles_.resize(0);
  }
  this->updateGL();
  return;
}

void Viewport::setCameraRefPt(float x, float y){
  std::vector<glow::vec4> inpoints;
  //std::cout<<x<<" "<<y<<std::endl;
  auto mvp = *projection_ * view_ * conversion_;

  for(int rad:{9,27,81,200,300,400}){
    polygonPoints_.clear();
    polygonPoints_.push_back(vec2(x+rad, y+rad));
    polygonPoints_.push_back(vec2(x+rad, y-rad));
    polygonPoints_.push_back(vec2(x-rad, y-rad));
    polygonPoints_.push_back(vec2(x-rad, y+rad));


    //BEGIN BLACKBOX


    std::vector<vec2> points = polygonPoints_;
    for (uint32_t i = 0; i < points.size(); ++i) {
      points[i].y = height() - points[i].y;  // flip y.
    }
    float winding = 0.0f;
    // important: take also last edge into account!
    for (uint32_t i = 0; i < points.size(); ++i) {
      const auto& p = points[(i + 1) % points.size()];
      const auto& q = points[i];
      winding += (p.x - q.x) * (q.y + p.y);
    }
    // invert  order if CW order.
    if (winding > 0) std::reverse(points.begin(), points.end());

    std::vector<Triangle> triangles;
    std::vector<glow::vec2> tris_verts;
    triangulate(points, triangles);
    std::vector<vec3> texContent(3 * 100);
    for (uint32_t i = 0; i < triangles.size(); ++i) {
      auto t = triangles[i];
      texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
      texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
      texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

      tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
      tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
      tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
      tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
      tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
      tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
    }
    numTriangles_ = triangles.size();
    // note: colors are in range [0,1] for FLOAT!
    texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
    bufTriangles_.assign(tris_verts);

    //END BLACKBOX


    
    selectPolygon(inpoints);
    //std::cout<<rad<<std::endl;
    if(inpoints.size()>0) break;

  }

  Eigen::Vector4f campose = conversion_.inverse() * mCamera->getPosition();

  //std::cout<<campose.x()<<" "<<campose.y()<<" "<<campose.x()<<std::endl;
  if(inpoints.size()>0){
  glow::vec4 bestpoint;
  float bestdist=999999;
  for(auto& i:inpoints){
    Eigen::Vector4f p_;
    p_ << i.x,i.y,i.z,1;
    p_-=campose;
    
    float dist = sqrt(p_.x()*p_.x()+p_.y()*p_.y()+p_.z()*p_.z());
    if (dist<bestdist){
      bestdist=dist;
      bestpoint = i;
    }
  }
  //std::cout<<"bestpoint: "<<bestpoint.x<<" "<<bestpoint.y<<" "<<bestpoint.z<<std::endl;
    Eigen::Vector4f bp;
    bp << bestpoint.x,bestpoint.y,bestpoint.z,1;
    bp = mvp * bp;
    //std::cout<<bp.x()/bp.w()<<" "<<bp.y()/bp.w()<<" "<<bp.z()/bp.w()<<" "<<bp.w()/bp.w()<<" "<<std::endl;
    *cameraRefPt = bp/bp.w();
  }else{
    Eigen::Vector4f cp = mCamera->getPosition();
    *cameraRefPt = Eigen::Vector4f::Zero();
  }
  polygonPoints_.clear();

}

void Viewport::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  mChangeCamera = false;
  auto mvp = *projection_ * view_ * conversion_;

  if (event->modifiers() == Qt::ControlModifier) {

    setCameraRefPt(event->x(), event->y());

    //mCamera->lookAt();
    

    if (mCamera->mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                              resolveKeyboardModifier(event->modifiers()))) {
      if (pressedkeys.empty()) {
        timer_.start(1 / 60);
      }
      pressedkeys.insert(Qt::Key_F25);  // abuse F25 for mouse events

      mChangeCamera = true;
      polygonPoints_.clear();  // start over again.#
      bufPolygonPoints_.assign(polygonPoints_);
      bufTriangles_.resize(0);
      return;
    }
  } else if (mMode == PAINT) {
    buttonPressed = true;
    mChangeCamera = false;
    if (event->buttons() & Qt::LeftButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
    else if (event->buttons() & Qt::RightButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);

    updateGL();
  } else if (mMode == POLYGON) {
    if (event->buttons() & Qt::LeftButton) {
      if (polygonPoints_.size() == 100) {
        polygonPoints_.back().x = event->x();
        polygonPoints_.back().y = event->y();
      } else {
        polygonPoints_.push_back(vec2(event->x(), event->y()));
      }

    } else if (event->buttons() & Qt::RightButton) {
      if (polygonPoints_.size() > 2) {
        // finish polygon and label points.

        // 1. determine winding: https://blog.element84.com/polygon-winding-post.html

        std::vector<vec2> points = polygonPoints_;
        for (uint32_t i = 0; i < points.size(); ++i) {
          points[i].y = height() - points[i].y;  // flip y.
        }

        float winding = 0.0f;

        // important: take also last edge into account!
        for (uint32_t i = 0; i < points.size(); ++i) {
          const auto& p = points[(i + 1) % points.size()];
          const auto& q = points[i];

          winding += (p.x - q.x) * (q.y + p.y);
        }

        // invert  order if CW order.
        if (winding > 0) std::reverse(points.begin(), points.end());

        //        if (winding > 0) std::cout << "winding: CW" << std::endl;
        //        else std::cout << "winding: CCW" << std::endl;

        std::vector<Triangle> triangles;
        std::vector<glow::vec2> tris_verts;

        triangulate(points, triangles);

        //        std::cout << "#triangles: " << triangles.size() << std::endl;

        std::vector<vec3> texContent(3 * 100);
        for (uint32_t i = 0; i < triangles.size(); ++i) {
          auto t = triangles[i];
          texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
          texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
          texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

          tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
          tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
          tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
          tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
          tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
          tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
        }

        numTriangles_ = triangles.size();
        // note: colors are in range [0,1] for FLOAT!
        texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
        bufTriangles_.assign(tris_verts);

        
        labelPoints(event->x(), event->y(), 0, mCurrentLabel, false);
      }

      polygonPoints_.clear();
    }

    bufPolygonPoints_.assign(polygonPoints_);

    repaint();
  } else if (mMode == AUTOAUTO) {
    
    if (event->buttons() & Qt::LeftButton) {
      if (polygonPoints_.size() > 1) {
        polygonPoints_.clear();
      }
      if (polygonPoints_.size() == 0) {
        polygonPoints_.push_back(vec2(event->x(), event->y()));
      } else {

        std::vector<vec2> points = polygonPoints_;
        points.push_back(vec2(event->x(), event->y()));

        vec2 direction;
        direction.x = points[1].x - points[0].x;
        direction.y = points[1].y - points[0].y;

        polygonPoints_.push_back(vec2(points[0].x-(0.33333*direction.y), points[0].y+(0.33333*direction.x)));
        polygonPoints_.push_back(vec2(points[1].x-(0.33333*direction.y), points[1].y+(0.33333*direction.x)));
        polygonPoints_.push_back(vec2(points[1].x, points[1].y));
        polygonPoints_.push_back(vec2(points[1].x+(0.33333*direction.y), points[1].y-(0.33333*direction.x)));
        polygonPoints_.push_back(vec2(points[0].x+(0.33333*direction.y), points[0].y-(0.33333*direction.x)));
      }

    } else if (event->buttons() & Qt::RightButton) {
      if (event->modifiers() == Qt::ShiftModifier){
        float x =  2*(event->x()/(float)(width())-0.5);
        float y =  -2*(event->y()/(float)(height())-0.5);
        
        bool found=false;
        for(const auto& ciw:carsInWorld_){
          std::shared_ptr<Car> result;
          Eigen::Matrix4f pose;
          try {
            result = ciw.first->getResults()[ciw.first->getSelectedCar()];
            pose = result->getPosition(singleScanIdx_);
          } catch (const std::out_of_range& oor) {
            continue;
          }
          Eigen::Vector4f v;
          v << pose(0,3),pose(1,3),pose(2,3),1;
          v = mvp*v;
          v /= v.w();
          float distance = (x-v.x())*(x-v.x())+(y-v.y())*(y-v.y());
          if (distance <= 1){
            auto gp = result->getGlobalPoints(singleScanIdx_);
            if (gp == nullptr) break;
            for(const auto& p:*(gp)){
              Eigen::Vector4f v;
              v << p.x,p.y,p.z,1;
              v = mvp*v;
              v /= v.w();
              float distance = (x-v.x())*(x-v.x())+(y-v.y())*(y-v.y());
              if (distance <= 0.001){
                found=true;
                break;
              }
            }
          }
          if(found) {
            std::cout << distance <<": "<< x << " " << y <<" / "<<v.x()<<" "<<v.y()<<std::endl;
            QMessageBox::StandardButton reply;
            std::string message= "Are you sure you want to delete "+result->getModel()+"?";
            reply = QMessageBox::question(this, "Remove Car", QString::fromStdString(message),
                                          QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes) {
              deleteAutoAuto(ciw.first);
              updateAutoAuto();
            } 
            break;
          };
        }

        
        
      }else{
        if (polygonPoints_.size() > 2) {
          // finish polygon and label points.

          // 1. determine winding: https://blog.element84.com/polygon-winding-post.html

          std::vector<vec2> points = polygonPoints_;
          

          for (uint32_t i = 0; i < points.size(); ++i) {
            points[i].y = height() - points[i].y;  // flip y.
          }

          float winding = 0.0f;

          // important: take also last edge into account!
          for (uint32_t i = 0; i < points.size(); ++i) {
            const auto& p = points[(i + 1) % points.size()];
            const auto& q = points[i];

            winding += (p.x - q.x) * (q.y + p.y);
          }

          // invert  order if CW order.
          if (winding > 0) std::reverse(points.begin(), points.end());

          //        if (winding > 0) std::cout << "winding: CW" << std::endl;
          //        else std::cout << "winding: CCW" << std::endl;

          std::vector<Triangle> triangles;
          std::vector<glow::vec2> tris_verts;

          triangulate(points, triangles);

          //        std::cout << "#triangles: " << triangles.size() << std::endl;

          std::vector<vec3> texContent(3 * 100);
          for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto t = triangles[i];
            texContent[3 * i + 0] = vec3(t.i.x / width(), (height() - t.i.y) / height(), 0);
            texContent[3 * i + 1] = vec3(t.j.x / width(), (height() - t.j.y) / height(), 0);
            texContent[3 * i + 2] = vec3(t.k.x / width(), (height() - t.k.y) / height(), 0);

            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.j.x, height() - t.j.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.k.x, height() - t.k.y));
            tris_verts.push_back(vec2(t.i.x, height() - t.i.y));
          }

          numTriangles_ = triangles.size();
          // note: colors are in range [0,1] for FLOAT!
          texTriangles_.assign(PixelFormat::RGB, PixelType::FLOAT, &texContent[0]);
          bufTriangles_.assign(tris_verts);
          
          //QProgressDialog* progress = new QProgressDialog("Matching Cars...", "Abort", 0, cars.size(), this);
          //progress->setWindowModality(Qt::WindowModal);
          applyAutoAuto();
          //delete progress;

        }
      }
      polygonPoints_.clear();
    }

    bufPolygonPoints_.assign(polygonPoints_);

    repaint();
  }

  event->accept();
}

void Viewport::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  
  if (mChangeCamera) {
    pressedkeys.erase(Qt::Key_F25);  // abuse F25 for mouse events
    if (pressedkeys.empty()) {
      timer_.stop();
    }
    updateGL();
    if (mCamera->mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                               resolveKeyboardModifier(event->modifiers()))) {
      // timer_.stop();
      updateGL();  // get the last action.

      return;
    }
  } else if (mMode == PAINT) {
    buttonPressed = false;

    if (event->buttons() & Qt::LeftButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
    else if (event->buttons() & Qt::RightButton)
      labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);

    updateGL();
  } else if (mMode == POLYGON) {
    if (polygonPoints_.size() > 0) {
      polygonPoints_.back().x = event->x();
      polygonPoints_.back().y = event->y();

      bufPolygonPoints_.assign(polygonPoints_);
    }

    repaint();
  } else if (mMode == AUTOAUTO) {
    if (polygonPoints_.size() > 1) {
      polygonPoints_[3] = (vec2(event->x(), event->y()));

      vec2 direction; 
      direction.x = polygonPoints_[3].x - polygonPoints_[0].x;
      direction.y = polygonPoints_[3].y - polygonPoints_[0].y;

      polygonPoints_[1] = (vec2(polygonPoints_[0].x-(0.33333*direction.y), polygonPoints_[0].y+(0.33333*direction.x)));
      polygonPoints_[2] = (vec2(polygonPoints_[3].x-(0.33333*direction.y), polygonPoints_[3].y+(0.33333*direction.x)));
      
      polygonPoints_[4] = (vec2(polygonPoints_[3].x+(0.33333*direction.y), polygonPoints_[3].y-(0.33333*direction.x)));
      polygonPoints_[5] = (vec2(polygonPoints_[0].x+(0.33333*direction.y), polygonPoints_[0].y-(0.33333*direction.x)));

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  }
  event->accept();

}

void Viewport::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    if (mCamera->mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButtonFlip(event->buttons()),
                            resolveKeyboardModifier(event->modifiers()))) {
      return;
    }
  } else if (mMode == PAINT) {
    if (buttonPressed) {
      if (event->buttons() & Qt::LeftButton)
        labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, false);
      else
        labelPoints(event->x(), event->y(), mRadius, mCurrentLabel, true);
    }
    updateGL();
  } else if (mMode == POLYGON) {
    if (polygonPoints_.size() > 0) {
      polygonPoints_.back().x = event->x();
      polygonPoints_.back().y = event->y();

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  } else if (mMode == AUTOAUTO) {
    if (polygonPoints_.size() > 1) {
      polygonPoints_[3] = (vec2(event->x(), event->y()));

      vec2 direction;
      direction.x = polygonPoints_[3].x - polygonPoints_[0].x;
      direction.y = polygonPoints_[3].y - polygonPoints_[0].y;


      polygonPoints_[1] = (vec2(polygonPoints_[0].x-(0.33333*direction.y), polygonPoints_[0].y+(0.33333*direction.x)));
      polygonPoints_[2] = (vec2(polygonPoints_[3].x-(0.33333*direction.y), polygonPoints_[3].y+(0.33333*direction.x)));
      
      polygonPoints_[4] = (vec2(polygonPoints_[3].x+(0.33333*direction.y), polygonPoints_[3].y-(0.33333*direction.x)));
      polygonPoints_[5] = (vec2(polygonPoints_[0].x+(0.33333*direction.y), polygonPoints_[0].y-(0.33333*direction.x)));

      bufPolygonPoints_.assign(polygonPoints_);

      repaint();
    }
  }

  event->accept();
}

void Viewport::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Escape:
      polygonPoints_.clear();  // start over again.#
      bufPolygonPoints_.assign(polygonPoints_);
      repaint();

      return;
    case Qt::Key_Delete:
      // delete last polygon point.

      if (mMode == POLYGON && polygonPoints_.size() > 0) {
        polygonPoints_.pop_back();

        bufPolygonPoints_.assign(polygonPoints_);
        repaint();
      }

      return;
    //    case Qt::Key_P:
    //      // Set plane parameters to car pose
    //      {
    //        planeThresholdNormal_ = 0;
    //        Eigen::Vector4f unit_vect(1.0, 0, 0, 0);
    //        auto normal_vect = points_[singleScanIdx_]->pose * unit_vect;
    //
    //        //        std::cout << "pose_matrix: " << std::endl << points_[singleScanIdx_]->pose << std::endl;
    //        //        std::cout << "normal_vect: " << normal_vect << std::endl;
    //
    //        planeNormal_[0] = normal_vect[0];
    //        planeNormal_[1] = normal_vect[1];
    //        planeNormal_[2] = normal_vect[2];
    //        planeDirectionNormal_ = 1.0;
    //        updateGL();
    //
    //        return;
    //      }
    // camera control
    case Qt::Key_C:
      if (points_.size() > 0) {
        if (points_.size() == 0) return;
        auto mat = conversion_ * points_[singleScanIdx_]->pose.inverse() * conversion_.inverse();
        mCamera->setMatrix(mat);
        updateGL();
      }
      return;
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
    {

      if (event->isAutoRepeat()) return;
      //      std::cout << event->key() << std::endl;
      if (mMode == POLYGON && polygonPoints_.size() > 0) {  // ugly hack!
        event->ignore();
        return;
      }

      if (pressedkeys.empty()) {
        timer_.start(1 / 60);
      }
      pressedkeys.insert(event->key());
      GlCamera::KeyboardKey k = resolveKeyboardKey(event->key());
      if (mCamera->keyPressed(k, resolveKeyboardModifier(event->modifiers()))) {
        event->accept();
      } else {
        event->ignore();
      };
      return;
    }
  }
  // handle event by parent:
  //  std::cout << event->key() << std::endl;
  event->ignore();
}

void Viewport::keyReleaseEvent(QKeyEvent* event) {
  switch (event->key()) {
    // camera control
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
    {
      if (event->isAutoRepeat()) return;
      pressedkeys.erase(event->key());
      if (pressedkeys.empty()) {
        timer_.stop();
      }
      GlCamera::KeyboardKey k = resolveKeyboardKey(event->key());
      if (mCamera->keyReleased(k, resolveKeyboardModifier(event->modifiers()))) {
        event->accept();
      } else {
        event->ignore();
      };
      return;
    }
  }
  event->ignore();
  /*std::cout << "myset contains:";
    for (std::set<int>::iterator it=pressedkeys.begin(); it!=pressedkeys.end(); ++it)
      std::cout << ' ' << *it;
    std::cout << '\n';*/
}

void Viewport::setTileInfo(float x, float y, float tileSize) {
  std::cout << x << ", " << y << ", " << tileSize << std::endl;
  tilePos_ = vec2(x, y);
  tileSize_ = tileSize;
}

void Viewport::labelPoints(int32_t x, int32_t y, float radius, uint32_t new_label, bool remove) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  //  std::cout << "called labelPoints(" << x << ", " << y << ", " << radius << ", " << new_label << ")" << std::flush;
  //  Stopwatch::tic();

  bool showSingleScan = drawingOption_["single scan"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgUpdateLabels_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfUpdateLabels_);

  prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
  if (points_.size() > singleScanIdx_)
    prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("pose", points_[singleScanIdx_]->pose));

  prgUpdateLabels_.setUniform(GlUniform<vec2>("window_pos", glow::vec2(x, y)));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("width", width()));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("height", height()));
  prgUpdateLabels_.setUniform(GlUniform<float>("radius", radius));
  prgUpdateLabels_.setUniform(GlUniform<uint32_t>("new_label", new_label));
  prgUpdateLabels_.setUniform(GlUniform<bool>("overwrite", mFlags & FLAG_OVERWRITE));
  prgUpdateLabels_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgUpdateLabels_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgUpdateLabels_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgUpdateLabels_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgUpdateLabels_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgUpdateLabels_.setUniform(GlUniform<int32_t>("heightMap", 1));
  prgUpdateLabels_.setUniform(GlUniform<bool>("removeLabel", remove));

  //  float planeThreshold = planeThreshold_;
  //  prgUpdateLabels_.setUniform(GlUniform<bool>("planeRemoval", planeRemoval_));
  //  prgUpdateLabels_.setUniform(GlUniform<int32_t>("planeDimension", planeDimension_));
  //  if (planeDimension_ == 0) planeThreshold += tilePos_.x;
  //  if (planeDimension_ == 1) planeThreshold += tilePos_.y;
  //  if (planeDimension_ == 2 && points_.size() > 0) planeThreshold += points_[0]->pose(3, 3);
  //  prgUpdateLabels_.setUniform(GlUniform<float>("planeThreshold", planeThreshold));
  //  prgUpdateLabels_.setUniform(GlUniform<float>("planeDirection", planeDirection_));
  //  prgUpdateLabels_.setUniform(GlUniform<bool>("carAsBase", drawingOption_["carAsBase"]));

  prgUpdateLabels_.setUniform(GlUniform<bool>("planeRemovalNormal", planeRemovalNormal_));
  prgUpdateLabels_.setUniform(GlUniform<Eigen::Vector3f>("planeNormal", planeNormal_));
  prgUpdateLabels_.setUniform(GlUniform<float>("planeThresholdNormal", planeThresholdNormal_));
  prgUpdateLabels_.setUniform(GlUniform<float>("planeDirectionNormal", planeDirectionNormal_));

  Eigen::Matrix4f plane_pose = Eigen::Matrix4f::Identity();
  plane_pose(0, 3) = tilePos_.x;
  plane_pose(1, 3) = tilePos_.y;
  plane_pose(2, 3) = points_[0]->pose(2, 3);
  if (drawingOption_["carAsBase"] && points_.size() > singleScanIdx_) {
    plane_pose = points_[singleScanIdx_]->pose;
    //    plane_pose.col(3) = points_[singleScanIdx_]->pose.col(3);
  }

  prgUpdateLabels_.setUniform(GlUniform<Eigen::Matrix4f>("plane_pose", plane_pose));

  mvp_ = *projection_ * mCamera->matrix() * conversion_;
  prgUpdateLabels_.setUniform(mvp_);

  if (mMode == Viewport::PAINT) prgUpdateLabels_.setUniform(GlUniform<int32_t>("labelingMode", 0));
  if (mMode == Viewport::POLYGON) {
    prgUpdateLabels_.setUniform(GlUniform<int32_t>("labelingMode", 1));
    prgUpdateLabels_.setUniform(GlUniform<int32_t>("numTriangles", numTriangles_));
  }

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.bind();

  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufUpdatedLabels_.size();
  uint32_t buffer_start = 0;
  uint32_t buffer_size = bufLabels_.size();

  if (showSingleScan) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  }

  while (count * max_size < buffer_size) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);

    tfUpdateLabels_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, buffer_start + count * max_size, size);
    tfUpdateLabels_.end();

    bufUpdatedLabels_.copyTo(0, size, bufLabels_, buffer_start + count * max_size);
    std::vector<uint32_t> bufout;
    
    
    count++;
  }

  glDisable(GL_RASTERIZER_DISCARD);

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.release();
  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.release();
  //  std::cout << Stopwatch::toc() << " s." << std::endl;

  emit labelingChanged();
}

glow::GlCamera::KeyboardKey Viewport::resolveKeyboardKey(int key) {
  switch (key) {
    case Qt::Key_A:
      return glow::GlCamera::KeyboardKey::KeyA;
    case Qt::Key_B:
      return glow::GlCamera::KeyboardKey::KeyB;
    case Qt::Key_C:
      return glow::GlCamera::KeyboardKey::KeyC;
    case Qt::Key_D:
      return glow::GlCamera::KeyboardKey::KeyD;
    case Qt::Key_E:
      return glow::GlCamera::KeyboardKey::KeyE;
    case Qt::Key_F:
      return glow::GlCamera::KeyboardKey::KeyF;
    case Qt::Key_G:
      return glow::GlCamera::KeyboardKey::KeyG;
    case Qt::Key_H:
      return glow::GlCamera::KeyboardKey::KeyH;
    case Qt::Key_I:
      return glow::GlCamera::KeyboardKey::KeyI;
    case Qt::Key_J:
      return glow::GlCamera::KeyboardKey::KeyJ;
    case Qt::Key_K:
      return glow::GlCamera::KeyboardKey::KeyK;
    case Qt::Key_L:
      return glow::GlCamera::KeyboardKey::KeyL;
    case Qt::Key_M:
      return glow::GlCamera::KeyboardKey::KeyM;
    case Qt::Key_N:
      return glow::GlCamera::KeyboardKey::KeyN;
    case Qt::Key_O:
      return glow::GlCamera::KeyboardKey::KeyO;
    case Qt::Key_P:
      return glow::GlCamera::KeyboardKey::KeyP;
    case Qt::Key_Q:
      return glow::GlCamera::KeyboardKey::KeyQ;
    case Qt::Key_R:
      return glow::GlCamera::KeyboardKey::KeyR;
    case Qt::Key_S:
      return glow::GlCamera::KeyboardKey::KeyS;
    case Qt::Key_T:
      return glow::GlCamera::KeyboardKey::KeyT;
    case Qt::Key_U:
      return glow::GlCamera::KeyboardKey::KeyU;
    case Qt::Key_V:
      return glow::GlCamera::KeyboardKey::KeyV;
    case Qt::Key_W:
      return glow::GlCamera::KeyboardKey::KeyW;
    case Qt::Key_X:
      return glow::GlCamera::KeyboardKey::KeyX;
    case Qt::Key_Y:
      return glow::GlCamera::KeyboardKey::KeyY;
    case Qt::Key_Z:
      return glow::GlCamera::KeyboardKey::KeyZ;
    case Qt::Key_0:
      return glow::GlCamera::KeyboardKey::Key0;
    case Qt::Key_1:
      return glow::GlCamera::KeyboardKey::Key1;
    case Qt::Key_2:
      return glow::GlCamera::KeyboardKey::Key2;
    case Qt::Key_3:
      return glow::GlCamera::KeyboardKey::Key3;
    case Qt::Key_4:
      return glow::GlCamera::KeyboardKey::Key4;
    case Qt::Key_5:
      return glow::GlCamera::KeyboardKey::Key5;
    case Qt::Key_6:
      return glow::GlCamera::KeyboardKey::Key6;
    case Qt::Key_7:
      return glow::GlCamera::KeyboardKey::Key7;
    case Qt::Key_8:
      return glow::GlCamera::KeyboardKey::Key8;
    case Qt::Key_9:
      return glow::GlCamera::KeyboardKey::Key9;
    case Qt::Key_F1:
      return glow::GlCamera::KeyboardKey::KeyF1;
    case Qt::Key_F2:
      return glow::GlCamera::KeyboardKey::KeyF2;
    case Qt::Key_F3:
      return glow::GlCamera::KeyboardKey::KeyF3;
    case Qt::Key_F4:
      return glow::GlCamera::KeyboardKey::KeyF4;
    case Qt::Key_F5:
      return glow::GlCamera::KeyboardKey::KeyF5;
    case Qt::Key_F6:
      return glow::GlCamera::KeyboardKey::KeyF6;
    case Qt::Key_F7:
      return glow::GlCamera::KeyboardKey::KeyF7;
    case Qt::Key_F8:
      return glow::GlCamera::KeyboardKey::KeyF8;
    case Qt::Key_F9:
      return glow::GlCamera::KeyboardKey::KeyF9;
    case Qt::Key_F10:
      return glow::GlCamera::KeyboardKey::KeyF10;
    case Qt::Key_F11:
      return glow::GlCamera::KeyboardKey::KeyF11;
    case Qt::Key_F12:
      return glow::GlCamera::KeyboardKey::KeyF12;
    case Qt::Key_Escape:
      return glow::GlCamera::KeyboardKey::KeyEsc;
    case Qt::Key_Up:
      return glow::GlCamera::KeyboardKey::KeyUpArrow;
    case Qt::Key_Down:
      return glow::GlCamera::KeyboardKey::KeyDownArrow;
    case Qt::Key_Left:
      return glow::GlCamera::KeyboardKey::KeyLeftArrow;
    case Qt::Key_Right:
      return glow::GlCamera::KeyboardKey::KeyRightArrow;
    case Qt::Key_Space:
      return glow::GlCamera::KeyboardKey::KeySpace;
    case Qt::Key_Enter:
      return glow::GlCamera::KeyboardKey::KeyEnter;
    case Qt::Key_Return:
      return glow::GlCamera::KeyboardKey::KeyEnter;
    default:
      return glow::GlCamera::KeyboardKey::KeyNotSupported;
  }
}
glow::GlCamera::KeyboardModifier Viewport::resolveKeyboardModifier(Qt::KeyboardModifiers modifiers) {
  // currently only single button presses are supported.
  GlCamera::KeyboardModifier modifier = GlCamera::KeyboardModifier::None;

  if (modifiers & Qt::ControlModifier)
    modifier = GlCamera::KeyboardModifier::CtrlDown;
  else if (modifiers & Qt::ShiftModifier)
    modifier = GlCamera::KeyboardModifier::ShiftDown;
  else if (modifiers & Qt::AltModifier)
    modifier = GlCamera::KeyboardModifier::AltDown;

  return modifier;
}

glow::GlCamera::MouseButton Viewport::resolveMouseButton(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;

  if (button & Qt::LeftButton)
    btn = GlCamera::MouseButton::LeftButton;
  else if (button & Qt::RightButton)
    btn = GlCamera::MouseButton::RightButton;
  else if (button & Qt::MiddleButton)
    btn = GlCamera::MouseButton::MiddleButton;

  return btn;
}

glow::GlCamera::MouseButton Viewport::resolveMouseButtonFlip(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;
  if (flipMouseButtons) {
    if (button & Qt::LeftButton)
      btn = GlCamera::MouseButton::LeftButton;
    else if (button & Qt::RightButton)
      btn = GlCamera::MouseButton::MiddleButton;
    else if (button & Qt::MiddleButton)
      btn = GlCamera::MouseButton::RightButton;
  } else {
    if (button & Qt::LeftButton)
      btn = GlCamera::MouseButton::LeftButton;
    else if (button & Qt::RightButton)
      btn = GlCamera::MouseButton::RightButton;
    else if (button & Qt::MiddleButton)
      btn = GlCamera::MouseButton::MiddleButton;
  }
  return btn;
}

void Viewport::centerOnCurrentTile() {
  // have to convert from robotic coordinate system to the opengl system.
  if (points_.size() == 0) return;

  Eigen::Vector4f t = points_[0]->pose.col(3);

  mCamera->lookAt(-tilePos_.y + 20, t.z() + 25, -tilePos_.x + 20, -tilePos_.y, t.z(), -tilePos_.x);
  updateGL();
}

void Viewport::setPlaneRemoval(bool value) {
  planeRemoval_ = value;
  updateGL();
}

void Viewport::setPlaneRemovalParams(float threshold, int32_t dim, float direction) {
  planeThreshold_ = threshold;
  planeDimension_ = dim;
  planeDirection_ = direction;
  updateGL();
}

void Viewport::setPlaneRemovalNormal(bool value) {
  planeRemovalNormal_ = value;
  updateGL();
}

const double PI = std::acos(-1);
void Viewport::setPlaneRemovalNormalParams(float threshold, float A1, float A2, float A3, float direction) {
  planeThresholdNormal_ = threshold;
  Eigen::Vector4f unit_vect(1.0, 0, 0, 0);
  //  auto rotX = glow::glRotateX(A1 * PI / 180);
  //  auto rotY = glow::glRotateY(A2 * PI / 180);
  //  auto rotZ = glow::glRotateZ(A3 * PI / 180);

  //  auto normal_vect = rotX * rotY * rotZ * unit_vect;

  float theta = A1 * PI / 180;
  float phi = A2 * PI / 180;

  //  Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
  //  R(0, 0) = std::sin(theta) * std::cos(phi);
  //  R(0, 1) = std::cos(theta) * std::cos(phi);
  //  R(0, 2) = -std::sin(phi);
  //
  //  R(1, 0) = std::sin(theta) * std::sin(phi);
  //  R(1, 1) = std::cos(theta) * std::sin(phi);
  //  R(1, 2) = std::cos(phi);
  //
  //  R(2, 0) = std::cos(theta);
  //  R(2, 1) = -std::sin(phi);
  //  R(2, 2) = 0;
  //
  //  auto rotX = glow::glRotateX(theta);
  //  auto rotY = glow::glRotateY(phi);
  //  R = rotX * rotY;
  //  auto normal_vect = R * Eigen::Vector4f(1, 0, 0, 0);

  float a = std::tan(theta);
  float b = std::tan(phi);
  Eigen::Vector3f n(a, b, 1);
  n.normalize();

  //  std::cout << "rotation matrix: " << std::endl << rotX * rotY * rotZ << std::endl;
  //  std::cout << "normal_vect: " << normal_vect << std::endl;

  //  planeNormal_[0] = normal_vect[0];
  //  planeNormal_[1] = normal_vect[1];
  //  planeNormal_[2] = normal_vect[2];
  planeNormal_ = n;  // normal_vect.head(3);
  planeDirectionNormal_ = direction;
  updateGL();
}

void Viewport::setFlipMouseButtons(bool value) { flipMouseButtons = value; }

std::vector<std::string> Viewport::getCameraNames() const {
  std::vector<std::string> keys;
  for (auto it = cameras_.begin(); it != cameras_.end(); ++it) keys.push_back(it->first);
  return keys;
}

std::map<std::string, std::shared_ptr<glow::GlCamera>> Viewport::getCameras() const { return cameras_; }

void Viewport::setCamera(const std::shared_ptr<glow::GlCamera>& cam) {
  Eigen::Matrix4f m = mCamera->matrix();
  mCamera = cam;
  mCamera->setMatrix(m);
}

void Viewport::setCameraProjection(const CameraProjection& proj) {
  projectionMode_ = proj;
  resizeGL(width(), height());
  updateGL();
}

void Viewport::setCameraByName(const std::string& name) {
  if (cameras_.find(name) == cameras_.end()) return;

  setCamera(cameras_[name]);
}

void Viewport::selectBox(std::vector<glow::vec4>& outpoints, const Eigen::Matrix4f& centerpose, const Eigen::Vector4f& corner) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  bool showSingleScan = drawingOption_["single scan"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgSelectBox_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfSelectBox_);  

  prgSelectBox_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgSelectBox_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgSelectBox_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgSelectBox_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgSelectBox_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgSelectBox_.setUniform(GlUniform<int32_t>("heightMap", 1));
  
  float planeThreshold = planeThreshold_;
  prgSelectBox_.setUniform(GlUniform<bool>("planeRemoval", planeRemoval_));
  prgSelectBox_.setUniform(GlUniform<int32_t>("planeDimension", planeDimension_));
  
  if (planeDimension_ == 0) planeThreshold += tilePos_.x;
  if (planeDimension_ == 1) planeThreshold += tilePos_.y;
  if (planeDimension_ == 2 && points_.size() > 0) planeThreshold += points_[0]->pose(3, 3);
  prgSelectBox_.setUniform(GlUniform<float>("planeThreshold", planeThreshold));
  prgSelectBox_.setUniform(GlUniform<float>("planeDirection", planeDirection_));

  prgSelectBox_.setUniform(GlUniform<Eigen::Matrix4f>("centerpose_inverse",centerpose.inverse()));
  prgSelectBox_.setUniform(GlUniform<Eigen::Vector4f>("corner",corner));
  
  glActiveTexture(GL_TEXTURE0);
  texTriangles_.bind();

  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufSelectedBox_.size();
  uint32_t buffer_start = 0;
  uint32_t buffer_size = bufLabels_.size();
  

  if (showSingleScan) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  }

  while (count * max_size < buffer_size) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);
    uint32_t bufferpos = buffer_start + count * max_size;
    tfSelectBox_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, bufferpos, size);
    tfSelectBox_.end();

    std::vector<uint32_t> bufout;
    std::vector<glow::vec4> bufpoints;
    
    bufSelectedBox_.get(bufout);
    
    assert(bufSelectedBox_.size == bufSelectedBox_.size);
    for (uint32_t i = 0; i < size; ++i){
      if (bufout[i]){
        std::vector<glow::vec4> buffer(1);
        bufPoints_.get(buffer,bufferpos+i,1);
        outpoints.push_back(buffer[0]);
      }
    }

    count++;
  }
  std::cout << outpoints.size() << std::endl;

  glDisable(GL_RASTERIZER_DISCARD);

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.release();
  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.release();
}

void Viewport::selectPolygon(std::vector<glow::vec4>& inpoints) {
  if (points_.size() == 0 || labels_.size() == 0) return;

  bool showSingleScan = drawingOption_["single scan"];

  ScopedBinder<GlVertexArray> vaoBinder(vao_points_);
  ScopedBinder<GlProgram> programBinder(prgSelectPoly_);
  ScopedBinder<GlTransformFeedback> feedbackBinder(tfSelectPoly_);  

  prgSelectPoly_.setUniform(GlUniform<int32_t>("width", width()));
  prgSelectPoly_.setUniform(GlUniform<int32_t>("height", height()));
  prgSelectPoly_.setUniform(GlUniform<bool>("removeGround", removeGround_));
  prgSelectPoly_.setUniform(GlUniform<float>("groundThreshold", groundThreshold_));
  prgSelectPoly_.setUniform(GlUniform<vec2>("tilePos", tilePos_));
  prgSelectPoly_.setUniform(GlUniform<float>("tileSize", tileSize_));
  prgSelectPoly_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
  prgSelectPoly_.setUniform(GlUniform<int32_t>("heightMap", 1));
  
  float planeThreshold = planeThreshold_;
  prgSelectPoly_.setUniform(GlUniform<bool>("planeRemoval", planeRemoval_));
  prgSelectPoly_.setUniform(GlUniform<int32_t>("planeDimension", planeDimension_));
  
  if (planeDimension_ == 0) planeThreshold += tilePos_.x;
  if (planeDimension_ == 1) planeThreshold += tilePos_.y;
  if (planeDimension_ == 2 && points_.size() > 0) planeThreshold += points_[0]->pose(3, 3);
  prgSelectPoly_.setUniform(GlUniform<float>("planeThreshold", planeThreshold));
  prgSelectPoly_.setUniform(GlUniform<float>("planeDirection", planeDirection_));

  mvp_ = *projection_ * mCamera->matrix() * conversion_;
  prgSelectPoly_.setUniform(mvp_);
  prgSelectPoly_.setUniform(GlUniform<int32_t>("numTriangles", numTriangles_));
  
  glActiveTexture(GL_TEXTURE0);
  texTriangles_.bind();

  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  uint32_t count = 0;
  uint32_t max_size = bufSelectedPoly_.size();
  uint32_t buffer_start = 0;
  uint32_t buffer_size = bufLabels_.size();
  

  if (showSingleScan) {
    buffer_start = scanInfos_[singleScanIdx_].start;
    buffer_size = scanInfos_[singleScanIdx_].size;
  }

  while (count * max_size < buffer_size) {
    uint32_t size = std::min<uint32_t>(max_size, buffer_size - count * max_size);
    uint32_t bufferpos = buffer_start + count * max_size;
    tfSelectPoly_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, bufferpos, size);
    tfSelectPoly_.end();

    std::vector<uint32_t> bufout;
    std::vector<glow::vec4> bufpoints;
    
    bufSelectedPoly_.get(bufout);
    
    assert(bufSelectedPoly_.size == bufSelectedPoly_.size);
    for (uint32_t i = 0; i < size; ++i){
      if (bufout[i]){
        std::vector<glow::vec4> buffer(1);
        bufPoints_.get(buffer,bufferpos+i,1);
        inpoints.push_back(buffer[0]);
      }
    }

    count++;
  }
  std::cout << inpoints.size() << std::endl;

  glDisable(GL_RASTERIZER_DISCARD);

  glActiveTexture(GL_TEXTURE0);
  texTriangles_.release();
  glActiveTexture(GL_TEXTURE1);
  texMinimumHeightMap_.release();
}

void Viewport::applyAutoAuto() {
  

  //QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

  //inverse projection Matrix
  Eigen::Matrix4f mvpinv_ = conversion_.inverse() * mCamera->matrix().inverse() * projection_->inverse();
  Eigen::Matrix4f mvp = *projection_ * mCamera->matrix() * conversion_;
  //std::cout<<"MVPinv\n"<<mvpinv_<<std::endl;
  //std::cout<<"MVP\n"<<mvp<<std::endl;
  //std::cout<< mvpinv_ * mvp << std::endl;

  //convert clicked points to a direction
  Eigen::Vector4f head;
  Eigen::Vector4f tail;
  head<< 2.0 * polygonPoints_[0].x/width() - 1.0, 2.0 * (1.0-polygonPoints_[0].y/height()) - 1.0,0.0, 1.0;
  tail<< 2.0 * polygonPoints_[3].x/width() - 1.0, 2.0 * (1.0-polygonPoints_[3].y/height()) - 1.0,0.0, 1.0;

  Eigen::Vector4f dir = ((mvpinv_ * head) - (mvpinv_ * tail));


  //list on points in the block
  auto pts = std::make_shared<std::vector<glow::vec4>>();
  selectPolygon(*pts);

  if(pts->size()<8){
    std::cout<<"not enough points"<<std::endl;
    return;
  }
  std::vector<float> p_z;
  
  for(const auto& p:*pts){
    Eigen::Vector4f p_; p_ << p.x,p.y,p.z,1;
    //if(zsum==0) std::cout<<"p_\n"<<p_<<std::endl;
    p_ = mvp * p_;
    p_ = p_ / p_.w();
    //if(zsum==0) std::cout<<"mvp_p_\n"<<p_<<std::endl;
    p_z.push_back(p_.z());
  }
  std::sort (p_z.begin(), p_z.end());
  float zsum = (p_z[floor(p_z.size()*0.05)]+p_z[floor(p_z.size()*0.95)])/2.;


  //zsum/=pts->size();
  Eigen::Vector4f center;
  center << 0.5*(head.x()+tail.x()),0.5*(head.y()+tail.y()),zsum,1;
  //std::cout<<"center\n"<<center<<std::endl;
  center = mvpinv_ * center;
  center = center / center.w();
  //std::cout<<"center_trans\n"<<center<<std::endl;

  if (drawingOption_["single scan"]){
    follow(*pts,dir,center);
  }else{
    //std::cout<<pts.size()<<" Points selected."<<std::endl;

    std::vector<std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>>> vect{first_cars_, more_cars_};
    auto a = std::make_shared<AutoAuto>(vect);
    (*autoautos)[a.get()] = a; //take a normal pointer as ID for the shared_ptr


    //AutoAuto a(cars_);
    connect(a.get(), SIGNAL(carProgressUpdate(float)), this, SLOT(updateProgressbar(float)));
    connect(a.get(), SIGNAL(carProgressFinished(AutoAuto*)),this, SLOT(afterAutoAuto(AutoAuto*)));
    //std::cout<<"dir\n"<<dir<<std::endl;
    a->matchPosition(pts,dir,center);
  }
}

void Viewport::afterAutoAuto(AutoAuto* a_) {
  //std::cout<<"SIGNAL"<<std::endl;
  auto a = (*autoautos)[a_]; //get shared_ptr of a_
  auto matched = a->getResults();
  //std::cout<<"MAX: "<<matched[0]->getModel()<<": "<<matched[0]->getInlier()<<" Inliers"<<std::endl;
  //carPoints_= *(matched[0]->getGlobalPoints());
  //std::cout<<"MAX: "<<matched[0]->getModel()<<": "<<carPoints_.size()<<" Points"<<std::endl;
  //bufCarPoints_.assign(carPoints_);
  //Eigen::Matrix4f pose = matched[0]->getPosition();
  //mCamera->lookAt(-pose(1,3) + 5, pose(2,3) + 1, -pose(0,3) + 5, -pose(1,3), pose(2,3), -pose(0,3));
  //updateGL();
  tempAutoAuto(a,a->getSelectedCar());
  CarDialog* cardialog = new CarDialog(a, this);
  connect(cardialog, SIGNAL(changeCar(std::shared_ptr<AutoAuto>, int)),this, SLOT(tempAutoAuto(std::shared_ptr<AutoAuto>, int)));
  connect(cardialog, SIGNAL(saveCar(std::shared_ptr<AutoAuto>)),this, SLOT(addAutoAutoToWorld(std::shared_ptr<AutoAuto>)));
  connect(cardialog, SIGNAL(discardCar(std::shared_ptr<AutoAuto>)),this, SLOT(deleteAutoAuto(std::shared_ptr<AutoAuto>)));
  connect(cardialog, SIGNAL(windowClosed()),this, SLOT(updateAutoAuto()));
  connect(cardialog, SIGNAL(continueCar(std::shared_ptr<AutoAuto>)),this,SLOT(continueAutoAuto(std::shared_ptr<AutoAuto>)));
  connect(this, SIGNAL(scanChanged()),cardialog, SLOT(viewChanged()));
  cardialog->show();
}

void Viewport::continueAutoAuto(std::shared_ptr<AutoAuto> a){
  a->matchPosition();
}

void Viewport::tempAutoAuto(std::shared_ptr<AutoAuto> a, int id){
  auto matched = a->getResults();
  std::vector<glow::vec4> temppoints = *(matched[id]->getGlobalPoints(singleScanIdx_));
  carPoints_.clear();
  for (auto const& c:carsInWorld_){
    carPoints_.insert(carPoints_.end(),c.second.begin(),c.second.end());
  }
  carPoints_.insert(carPoints_.end(),temppoints.begin(),temppoints.end());
  bufCarPoints_.assign(carPoints_);
  updateGL();
}

void Viewport::addAutoAutoToWorld(std::shared_ptr<AutoAuto> a){
  std::string emo;
  switch(rand() % 5){
    case 0: emo="\U0001F697"; break; //Automobile
    case 1: emo="\U0001F693"; break; //Police Car
    case 2: emo="\U0001F3CE"; break; //Racing Car
    case 3: emo="\U0001F699"; break; //Recreational Vehicle
    case 4: emo="\U0001F695"; break; //Taxi
  }
  std::cout<<emo<<"  "<<std::flush; 
  carsInWorld_[a] = *(a->getResults()[a->getSelectedCar()]->getGlobalPoints(singleScanIdx_));
  updateAutoAuto();
  emit labelingChanged();
}

void Viewport::updateAutoAuto(){
  carPoints_.clear();
  for (auto const& c:carsInWorld_){
    carPoints_.insert(carPoints_.end(),c.second.begin(),c.second.end());
  }
  bufCarPoints_.assign(carPoints_);
  updateGL();
}

void Viewport::deleteAutoAuto(std::shared_ptr<AutoAuto> a){
  std::cout<<"DELETE"<<std::endl;
  autoautos->erase(a.get());
  carsInWorld_.erase(a);
}

void Viewport::updateProgressbar(float progress){
  if (progressdiag == nullptr && progress<1){
    progressdiag = new QProgressDialog("Operation in progress.", "Cancel", 0, 100);
    progressdiag->setCancelButton(0);
    progressdiag->show();
    //std::cout<<"SHOW"<<std::endl;
  }else{ if(progressdiag == nullptr) return;}
  if (progress >= 1){
    delete progressdiag;
    progressdiag = nullptr;
    //std::cout<<"HIDE"<<std::endl;

  } else{
    progressdiag->setValue(progress*100);
    //std::cout<<"UPDATE"<<std::endl;
  }
}

std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> Viewport::getAutoAutos(){
  return autoautos;
}

std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>> Viewport::getCars(){
  return cars_;
}

void Viewport::follow(const std::vector<glow::vec4>& pts_, const Eigen::Vector4f dir_, const Eigen::Vector4f center_){
  Eigen::Vector3f f; f<<dir_.x(),dir_.y(),dir_.z();
  //Eigen::Matrix4f mvp = *projection_ * view_ * conversion_;
  auto pointcloud = std::make_shared<std::vector<glow::vec4>>();
  std::vector<double> pointcloud_;
  std::shared_ptr<MovingCar> mcar = std::make_shared<MovingCar>("_generated",pointcloud);
  std::map<int,Eigen::Matrix4f> globalPoses;
  int initpose = singleScanIdx_;

  f=f.normalized();
  Eigen::Vector3f u; u<<0,0,1;
  Eigen::Vector3f s = f.cross(u).normalized();
  u = s.cross(f).normalized();

  Eigen::Matrix4f rottransmat;
  rottransmat << s.x(), u.x(),-f.x(), center_.x(),
                 s.y(), u.y(),-f.y(), center_.y(),
                 s.z(), u.z(),-f.z(), center_.z(),
                 0.,0.,0.,1.;

  Eigen::Matrix4f orientation;
  orientation << 0,1, 0, 0,
                 0, 0, 1, 0,
                1, 0, 0, 0,
                 0, 0, 0, 1;

  rottransmat*=orientation;
  globalPoses[initpose] =rottransmat;
  auto singlecloud = std::make_shared<std::vector<glow::vec4>>();
  for(auto const& value: pts_) {
    Eigen::Vector4f v;
    v << value.x, value.y, value.z, 1;
    v = rottransmat.inverse() * v;
    pointcloud_.push_back(v.x());
    pointcloud_.push_back(v.y());
    pointcloud_.push_back(v.z());
    pointcloud->push_back(vec4(v.x(),v.y(),v.z(),1));
    singlecloud->push_back(vec4(value.x,value.y,value.z,1));
  }
  mcar->setOriginalPoints(singlecloud,singleScanIdx_);

  Eigen::Vector4f ahead,ahead_,toside;

  //Eigen::Matrix4f toside,toside_,offset;
  ahead_ << -1,0,0,0; //1 Meter ahead
  //toside_ = Eigen::Matrix4f::Identity();
  toside << 0,1,0,0;
  


  std::vector<float> xlist;
  std::vector<float> ylist;
  std::vector<float> zlist;

  for(auto const& value: pts_) {
    Eigen::Vector4f v; 
    v << value.x, value.y, value.z, 1;
    v = rottransmat.inverse() * v;
    xlist.push_back(v.x());
    ylist.push_back(v.y());
    zlist.push_back(v.z());
  }
  std::sort (xlist.begin(), xlist.end());
  std::sort (ylist.begin(), ylist.end());
  std::sort (zlist.begin(), zlist.end());


  Eigen::Vector4f vec20; 
  vec20 << 
    xlist[floor(xlist.size()*0.2)],
    ylist[floor(ylist.size()*0.2)],
    zlist[floor(zlist.size()*0.2)],
    1;
  Eigen::Vector4f vec80;
  vec80 << 
    xlist[floor(xlist.size()*0.8)],
    ylist[floor(ylist.size()*0.8)],
    zlist[floor(zlist.size()*0.8)],
    1;

  std::cout<<"vec80\n"<<vec80<<"\nvec20\n"<<vec20<<std::endl;

  Eigen::Vector4f corner; corner << std::max(fabs(vec80.x()),fabs(vec20.x())),
                                    std::max(fabs(vec80.y()),fabs(vec20.y())),
                                    std::max(fabs(vec80.z()),fabs(vec20.z())),
                                    1;
  float offset = corner.y();
  corner *= 2.5; //(0.5/0.3)*1.2;

  std::cout<<"corner\n"<<corner<<std::endl;

  
  int scancounter_forwards = singleScanIdx_;
  int scancounter_backwards = singleScanIdx_;
  int8_t forwards = -1;


  std::map<float,Eigen::Matrix4f> rotations; 
  for (float i=-1.5; i <= 6.5; i+=0.1){
      float maxside=(abs(i)<sqrt(3))?2-sqrt(4-i*i):1;
      maxside=round(maxside*10.)/10.;

    for (float s=-maxside; s <= maxside; s+=0.1){

      float alpha = 0;
      
        Eigen::Matrix4f rotation, offsetA, offsetB;
        if (round(s*10)!=0){
          alpha = PI-2*atan(i/s);
        }else{
          alpha = 0;
        }
        rotation <<
          cos(alpha), -sin(alpha), 0,0,
          sin(alpha), cos(alpha),0,0,
          0,0,1,0,
          0,0,0,1;
        offsetA <<
          1,0,0,0,
          0,1,0,offset,
          0,0,1,0,
          0,0,0,1;
        offsetB <<
          1,0,0,0,
          0,1,0,-offset,
          0,0,1,0,
          0,0,0,1;

        rotations[1000*i+s] = offsetA * rotation * offsetB;
    }
  }

  while(singleScanIdx_+1<(uint32_t)(scanInfos_.size())){    
    if (scancounter_forwards==-1 && scancounter_backwards==-1 ) break;
    if (scancounter_forwards!=-1 && scancounter_backwards!=-1 ) forwards=0-(forwards);
    int scan;
    if (forwards==1){
      if (scancounter_forwards+1>=(int32_t)(scanInfos_.size())){
        scancounter_forwards=-1;
        forwards=-1;
        continue;
      }
      scan = scancounter_forwards += 1;
    }else{
      if (scancounter_backwards-1<0){
        scancounter_backwards=-1;
        forwards=1;
        continue;
      }
      scan = scancounter_backwards -= 1;
    }
    rottransmat = globalPoses[scan-forwards];

    ahead = rottransmat * ahead_;
    //toside = rottransmat * toside_;

    rottransmat.col(3)+=ahead*1.5*forwards;

 
    setScanIndex(scan);

    std::vector<glow::vec4> newpts;
    selectBox(newpts, rottransmat, corner);
    if (newpts.size() <= 5) {
      if (forwards==1){
        scancounter_forwards=-1;
        forwards=-1;
        continue;
      }else{
        scancounter_backwards=-1;
        forwards=1;
        continue;
      }
    }
    std::vector<double> scanPoints;

    for(auto const& value:newpts) {
      scanPoints.push_back(value.x);
      scanPoints.push_back(value.y);
      scanPoints.push_back(value.z);
    }

    float min_i=0;
    float min_s=0;
    float valmin_i=999999;

    //IcpPointToPoint icp(scanPoints.data(),scanPoints.size()/3,3,(float)1e-5);
    IcpPointToPoint icp(pointcloud_.data(),pointcloud_.size()/3,3,(float)1e-5);
    std::cout<<"<"<<std::flush;
    ctpl::thread_pool pool{(int)std::thread::hardware_concurrency()};
    std::mutex findmin;

    for (float i=-1.5; i <= 5.5; i+=0.1){
      float maxside=(abs(i)<sqrt(3))?2-sqrt(4-i*i):0.5;
      maxside=round(maxside*10.)/10.;

      for (float s=-maxside; s <= maxside; s+=0.1){
        pool.push([&findmin,&min_i, &min_s, &valmin_i, &rottransmat, &rotations,i,s,&forwards,&toside,&ahead,&scanPoints,&icp, this](int){
          auto rottransmat2 = rottransmat;
         
          rottransmat2 *= rotations[1000*i+s];
          rottransmat2.col(3)+=toside * s;
          rottransmat2.col(3)+=ahead*i*forwards;
          //rottransmat2.col(3)+=toside*s;
          rottransmat2 = rottransmat2.inverse();
          FLOAT r_tmp[] = {
            rottransmat2(0,0),rottransmat2(0,1),rottransmat2(0,2),
            rottransmat2(1,0),rottransmat2(1,1),rottransmat2(1,2),
            rottransmat2(2,0),rottransmat2(2,1),rottransmat2(2,2)
          };
          FLOAT t_tmp[] = {
            rottransmat2(0,3),
            rottransmat2(1,3),
            rottransmat2(2,3)
          };

          Matrix rt_(3,3,r_tmp);
          Matrix tt_(3,1,t_tmp);
          
          float result = icp.getSqDistance(scanPoints.data(),std::min(5000,(int)scanPoints.size()/3),rt_,tt_,-1);
          //std::cout<<"FWD: "<<i<<" SIDE: "<<s<<" Alpha: "<<alpha*180/PI<<" result: "<<result<<std::endl;
          findmin.lock();
          if(result<valmin_i){
            valmin_i = result;
            min_i = i;
            min_s = s;
          }
          findmin.unlock();
        });
      }

    } 
    pool.stop(true);
    std::cout<<">"<<std::endl;

    rottransmat *= rotations[1000*min_i+min_s];
    rottransmat.col(3)+=toside * min_s;
    rottransmat.col(3)+=ahead*min_i*forwards;
    //rottransmat.col(3)+=ahead*min_s;
    auto rtmi = rottransmat.inverse();

    FLOAT r[] = {
      rtmi(0,0),rtmi(0,1),rtmi(0,2),
      rtmi(1,0),rtmi(1,1),rtmi(1,2),
      rtmi(2,0),rtmi(2,1),rtmi(2,2)
    };
    FLOAT t[] = {
      rtmi(0,3),
      rtmi(1,3),
      rtmi(2,3)
    };

    Matrix r_(3,3,r);
    Matrix t_(3,1,t);

    icp.max_iter=300;
    icp.fit(scanPoints.data(),std::min(20000,(int)scanPoints.size()/3),r_,t_,0.05);
    std::vector<bool> activePTS = icp.getInDistance(scanPoints.data(),std::min(20000,(int)scanPoints.size()/3),r_,t_,0.02);

    //DemoCar
    FLOAT r2[9];
    FLOAT t2[3];
    r_.getData(r2,0,0,2,2);
    t_.getData(t2,0,0,2,0);

    Eigen::Matrix4f newrotmat; newrotmat <<
      r2[0], r2[1],r2[2], t2[0],
      r2[3], r2[4],r2[5], t2[1],
      r2[6], r2[7],r2[8], t2[2],
      0.,0.,0.,1.;

    rottransmat = newrotmat.inverse();
    auto singlecloud = std::make_shared<std::vector<glow::vec4>>();

    for(uint i=0;i<activePTS.size();i++){
        Eigen::Vector4f v;
        Eigen::Vector4f v2;
        v << newpts[i].x, newpts[i].y, newpts[i].z, 1;
        v = rottransmat.inverse() * v;
      //if(!activePTS[i]){
        pointcloud_.push_back(v.x());
        pointcloud_.push_back(v.y());
        pointcloud_.push_back(v.z());
      //}
        pointcloud->push_back(vec4(v.x(),v.y(),v.z(),1));
        singlecloud->push_back(vec4(newpts[i].x,newpts[i].y,newpts[i].z,1));
    }
    mcar->setOriginalPoints(singlecloud,scan);
    globalPoses[scan] = rottransmat;

  }
  
  mcar->setPosition(globalPoses);
  mcar->setInitPose(initpose);
  std::vector<std::shared_ptr<std::map<std::string, std::shared_ptr<Car>>>> vect;
  auto a = std::make_shared<AutoAuto>(vect);
  a->setSelectedpts(pts_);
  a->setDir(dir_);
  a->addResult(std::dynamic_pointer_cast<Car>(mcar));
  (*autoautos)[a.get()] = a;
  //addAutoAutoToWorld(a);
  setScanIndex(initpose);

  updateGL();
  afterAutoAuto(a.get());

}