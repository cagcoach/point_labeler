/* \brief a view, which can be used to label points.
 *
 *
 */
#ifndef POINTVIEW_H_
#define POINTVIEW_H_

#include <stdint.h>
#include <list>
#include <set>
#include <vector>

#include <glow/glbase.h>

#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <QtOpenGL/QGLWidget>
#include <QProgressDialog>


#include <GL/gl.h>
#include <GL/glu.h>

#include <glow/GlBuffer.h>
#include <glow/GlColor.h>
#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlRenderbuffer.h>
#include <glow/GlShaderCache.h>
#include <glow/GlTexture.h>
#include <glow/GlVertexArray.h>
#include <glow/util/GlCamera.h>
#include <glow/util/RoSeCamera.h>
#include "CADCamera.h"
#include "AutoAuto.h"
#include "Car.h"


#include "common.h"

#include "data/geometry.h"

/** \brief Canvas for visualizing point clouds.
 *
 *  All operations working with the points, labels, etc. exploit OpenGL to run as fast as possible.
 *  Here, we exploit transform feedbacks to fill/update buffers and framebuffers to fill textures.
 *
 *  After setting the points with setPoints, all operations are performed in GPU memory. Thus, we
 *  have to transfer the labels after labeling a tile back to the host memory.
 *
 *  \author behley
 **/
class Viewport : public QGLWidget {
  Q_OBJECT
 public:
  enum AXIS { XYZ, X, Y, Z };

  enum MODE { NONE, PAINT, POLYGON, AUTOAUTO };

  enum FLAGS { FLAG_OVERWRITE = 1, FLAG_OTHER = 2 };

  enum class CameraProjection { perspective, orthographic };

  Viewport(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~Viewport();

  void setMaximumScans(uint32_t numScans);

  /** \brief set points and initialize buffers with data inside the tile. **/
  void setPoints(const std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels);

  /** \brief update all labels with GPU labels. **/
  void updateLabels();

  void setLabelVisibility(uint32_t label, bool visible);

  void setDrawingOption(const std::string& name, bool value);

  void setMinRange(float range);
  void setMaxRange(float range);

  void setGroundRemoval(bool value);
  void setGroundThreshold(float value);

  void setScanIndex(uint32_t idx);

  void setTileInfo(float x, float y, float tileSize);

  void centerOnCurrentTile();

  void setPlaneRemoval(bool value);
  void setPlaneRemovalParams(float threshold, int32_t dim, float direction);

  uint32_t loadedPointCount() const { return bufPoints_.size(); }
  uint32_t labeledPointCount() const { return labeledCount_; }

  std::map<std::string, std::shared_ptr<glow::GlCamera>> getCameras() const;

  /** \brief set camera names **/
  std::vector<std::string> getCameraNames() const;

  /** \brief set camera explcitly. **/
  void setCamera(const std::shared_ptr<glow::GlCamera>& cam);

  /** \brief set camera by name. **/
  void setCameraByName(const std::string& name);

  void setCameraProjection(const CameraProjection& proj);
  std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> getAutoAutos();
  std::shared_ptr<std::map<std::string, Car>> getCars();
  


 signals:
  void labelingChanged();

 public slots:
  /** \brief set axis fixed **/
  void setFixedAxis(AXIS axis);

  void setRadius(float value);
  /** \brief label used when in PAINT **/
  void setLabel(uint32_t label);
  void setLabelColors(const std::map<uint32_t, glow::GlColor>& colors);
  void setPointSize(int value);

  void setMode(MODE mode);
  void setFlags(int32_t flags);
  void setOverwrite(bool value);

  void setFilteredLabels(const std::vector<uint32_t>& labels);

  void keyPressEvent(QKeyEvent*);
  void keyReleaseEvent(QKeyEvent*);
  void setFlipMouseButtons(bool value);

  void afterAutoAuto(AutoAuto* a_);
  void updateProgressbar(float progress);
  void tempAutoAuto(std::shared_ptr<AutoAuto>, int);
  void addAutoAutoToWorld(std::shared_ptr<AutoAuto>);
  void updateAutoAuto();

 protected:
  bool initContext() {
    // enabling core profile
    QGLFormat corefmt;
    corefmt.setVersion(5, 0);  // getting highest compatible format...
    corefmt.setProfile(QGLFormat::CoreProfile);
    setFormat(corefmt);

    // version info.
    QGLFormat fmt = this->format();
    std::cout << "OpenGL Context Version " << fmt.majorVersion() << "." << fmt.minorVersion() << " "
              << ((fmt.profile() == QGLFormat::CoreProfile) ? "core profile" : "compatibility profile") << std::endl;

    makeCurrent();
    glow::inititializeGLEW();

    return true;
  }

  QWidget * parent;

  void initPrograms();
  void initVertexBuffers();

  void updateHeightmap();

  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();

  void wheelEvent(QWheelEvent*);
  void mousePressEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);

  glow::GlCamera::KeyboardModifier resolveKeyboardModifier(Qt::KeyboardModifiers modifiers);
  glow::GlCamera::KeyboardKey resolveKeyboardKey(int key);
  glow::GlCamera::MouseButton resolveMouseButton(Qt::MouseButtons button);
  glow::GlCamera::MouseButton resolveMouseButtonFlip(Qt::MouseButtons button);

  //  void drawPoints(const std::vector<Point3f>& points, const std::vector<uint32_t>& labels);
  void labelPoints(int32_t x, int32_t y, float radius, uint32_t label, bool remove);
  void selectPolygon(std::vector<glow::vec4>& inpoints);
  void applyAutoAuto();

  bool contextInitialized_;
  std::map<uint32_t, glow::GlColor> mLabelColors;

  std::vector<PointcloudPtr> points_;
  std::vector<LabelsPtr> labels_;
  glow::RoSeCamera rosecam;
  CADCamera cadcam;
  std::shared_ptr<glow::GlCamera> mCamera;
  std::map<std::string, std::shared_ptr<glow::GlCamera>> cameras_;
  bool mChangeCamera{false};

  AXIS mAxis;
  MODE mMode;
  int32_t mFlags;

  uint32_t mCurrentLabel;
  float mRadius;
  std::vector<uint32_t> mFilteredLabels;

  /** selected endpoint **/

  bool buttonPressed;
  QTimer timer_;

  // shaders, etc.
  uint32_t maxScans_{50};
  uint32_t maxPointsPerScan_{150000};
  std::vector<Eigen::Matrix4f> bufPoses_;

  // todo: rename to TilePoints, TileRemissions, TileLabels, TileVisible, TileScanIndexes.
  glow::GlBuffer<glow::vec4> bufPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufVisible_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<glow::vec2> bufScanIndexes_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<glow::vec4> bufCarPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};


  // buffers used for copying points to tile buffers.
  glow::GlBuffer<Point3f> bufTempPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<float> bufTempRemissions_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufTempLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<uint32_t> bufTempVisible_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTransformFeedback tfUpdateLabels_;
  glow::GlBuffer<uint32_t> bufUpdatedLabels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTransformFeedback tfSelectPoly_;
  glow::GlBuffer<uint32_t> bufSelectedPoly_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};


  glow::GlTransformFeedback tfUpdateVisibility_;
  glow::GlBuffer<uint32_t> bufUpdatedVisiblity_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlBuffer<glow::vec2> bufHeightMapPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  glow::GlTransformFeedback tfFillTilePoints_;

  glow::GlTextureRectangle texLabelColors_;

  glow::GlVertexArray vao_no_points_;
  glow::GlVertexArray vao_points_;
  glow::GlVertexArray vao_polygon_points_;
  glow::GlVertexArray vao_triangles_;
  glow::GlVertexArray vao_temp_points_;
  glow::GlVertexArray vao_heightmap_points_;
  glow::GlVertexArray vao_car_points_;

  glow::GlProgram prgDrawPose_;
  glow::GlProgram prgDrawPoints_;
  glow::GlProgram prgUpdateLabels_;
  glow::GlProgram prgUpdateVisibility_;
  glow::GlProgram prgPolygonPoints_;
  glow::GlProgram prgFillTilePoints_;
  glow::GlProgram prgDrawFrustum_;
  glow::GlProgram prgDrawHeightmap_;
  glow::GlProgram prgDrawCarPoints_;

  glow::GlFramebuffer fbMinimumHeightMap_;
  glow::GlTexture texMinimumHeightMap_;
  glow::GlTexture texTempHeightMap_;
  glow::GlProgram prgMinimumHeightMap_;
  glow::GlProgram prgAverageHeightMap_;

  glow::GlProgram prgSelectPoly_;

  int32_t pointSize_{1};

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t", Eigen::Matrix4f::Identity()};

  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f conversion_{glow::RoSe2GL::matrix};

  std::map<std::string, bool> drawingOption_;

  float minRange_{0.0f}, maxRange_{100.0f};

  bool removeGround_{false};
  float groundThreshold_{-1.6f};
  float groundResolution_{0.5f};


  //void loadCarModels();

  //AutoAuto autoAuto{"../cars"};
  std::shared_ptr<std::map<AutoAuto*, std::shared_ptr<AutoAuto>>> autoautos;
  std::shared_ptr<std::map<std::string, Car>> cars_;
  std::map<std::shared_ptr<AutoAuto>, std::vector<glow::vec4>> carsInWorld_;
  std::vector<glow::vec2> polygonPoints_;
  std::vector<glow::vec4> carPoints_;

  glow::GlBuffer<glow::vec2> bufPolygonPoints_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlTextureRectangle texTriangles_;
  glow::GlBuffer<glow::vec2> bufTriangles_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  uint32_t numTriangles_{0};

  uint32_t singleScanIdx_{0};

  glow::vec2 tilePos_;
  float tileSize_;
  float tileBoundary_{2.0f};

  struct ScanInfo {
    uint32_t start;
    uint32_t size;
  };
  std::set<int> pressedkeys;
  std::vector<ScanInfo> scanInfos_;

  bool planeRemoval_{false};
  float planeThreshold_{0.0f};
  int32_t planeDimension_{0};
  float planeDirection_{1.0f};

  uint32_t labeledCount_{0};
  bool flipMouseButtons{false};

  CameraProjection projectionMode_{CameraProjection::perspective};
  QProgressDialog* progressdiag{nullptr};

};

#endif /* POINTVIEW_H_ */
