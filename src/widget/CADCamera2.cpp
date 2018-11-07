#include "CADCamera2.h"

#include <iostream>
#include <QCursor>

CADCamera2::CADCamera2(std::shared_ptr<Eigen::Vector4f> refpt_,std::shared_ptr<Eigen::Matrix4f> projection_, QWidget* refwidget_){
  refpt = refpt_;
  refwidget = refwidget_;
  projection =  projection_;
}

const Eigen::Matrix4f& CADCamera2::matrix() {
  mutex_.lock();

  auto end = std::chrono::system_clock::now();
  double dt = 0.0001 * std::chrono::duration_cast<std::chrono::milliseconds>(end - startTime_).count();

  if (dt > 0 && startdrag_) {
    // apply velocity & reset timer...
    rotate(turnVel_ * dt, 0.0f);
    translate(forwardVel_ * dt, upVel_ * dt, sideVel_ * dt);
    startTime_ = end;
  }

  // recompute the view matrix (Euler angles) Remember: Inv(AB) = Inv(B)*Inv(A)
  // Inv(translate*rotateYaw*rotatePitch) = Inv(rotatePitch)*Inv(rotateYaw)*Inv(translate)
  view_ = glow::glRotateX(-pitch_);
  view_ = view_ * glow::glRotateY(-yaw_);
  view_ = view_ * glow::glTranslate(-x_, -y_, -z_);

  mutex_.unlock();

  return view_;
}

void CADCamera2::setMatrix(const Eigen::Matrix4f& m) {
  mutex_.lock();

  Eigen::Vector4f view_dir = m.transpose() * Eigen::Vector4f(0, 0, -1, 0);
  Eigen::Vector4f new_cam = m.inverse() * Eigen::Vector4f(0, 0, 0, 1);
  Eigen::Vector4f new_ref = new_cam + view_dir;

  mutex_.unlock();

  lookAt(new_cam.x(), new_cam.y(), new_cam.z(), new_ref.x(), new_ref.y(), new_ref.z());
}

// since we are using here a different representation of the camera model we
// have to re-implement everything to allow an appropriate modification.
void CADCamera2::setPosition(float x, float y, float z) {
  mutex_.lock();

  x_ = x;
  y_ = y;
  z_ = z;

  mutex_.unlock();
}

Eigen::Vector4f CADCamera2::getPosition() const { return Eigen::Vector4f(x_, y_, z_, 1.0f); }

void CADCamera2::lookAt(float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  // = std::acos(-dir.y()) - M_PI_2 = M_PI - std::acos(dir.y()) - M_PI_2; // in [-pi/2,pi/2]
  pitch_ = std::asin(dir.y());
  yaw_ = atan2(-x, -z);

  mutex_.unlock();
}

void CADCamera2::lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  x_ = x_cam;
  y_ = y_cam;
  z_ = z_cam;

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  pitch_ = std::asin(dir.y());  // = std::acos(-dir.y()) - M_PI_2 in [-pi/2,pi/2]
  yaw_ = atan2(-x, -z);

  mutex_.unlock();

  //  lookAt(x_ref, y_ref, z_ref);
}

void CADCamera2::setYaw(float yaw) {
  mutex_.lock();
  yaw_ = yaw;
  mutex_.unlock();
}

void CADCamera2::setPitch(float pitch) {
  mutex_.lock();
  pitch_ = pitch;
  mutex_.unlock();
}

void CADCamera2::getCameraParameters(float& x, float& y, float& z, float& yaw, float& pitch) {
  mutex_.lock();

  x = x_;
  y = y_;
  z = z_;
  yaw = yaw_;
  pitch = pitch_;
 
  mutex_.unlock();
}

bool CADCamera2::mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  startx_ = x;
  starty_ = y;
  startyaw_ = yaw_;
  startpitch_ = pitch_;
  startcx_ = x_;
  startcy_ = y_;
  startcz_ = z_;
  startTime_ = std::chrono::system_clock::now();
  startdrag_ = true;
  //std::cout<<x<<" "<<y<<" "<<std::endl;
 
  if(*refpt!=Eigen::Vector4f::Zero()){
    refp<<2*x/(float)refwidget->width()-1,-2*y/(float)refwidget->height()+1,refpt->z(),1;
    refp = projection->inverse()*refp;
    refp /= refp.w();
    //std::cout<<refp<<std::endl;
    my_factor = std::abs(refp.z()/refwidget->height());
  }else{
    my_factor = (startcy_ > 50 ? 25 : (startcy_ > 1 ? startcy_ * 0.5 : .5))/refwidget->height();
  }


  if (btn == MouseButton::MiddleButton || btn == MouseButton::RightButton){
    refwidget->setCursor(Qt::BlankCursor);
    virtCursor = refwidget->cursor().pos();
    std::cout<<(int)btn<<std::endl;
    updateCursorLater=true;
  }
  // std::cout<<static_cast<std::underlying_type<MouseButton>::type>(btn)<<std::endl;
  return true;
}

bool CADCamera2::mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) {  
  refwidget->setCursor(Qt::ArrowCursor); 
  std::cout<<(int)btn<<std::endl;
  if(updateCursorLater){
    QPoint VPposition=refwidget->mapToGlobal(QPoint(0,0));
    if (virtCursor.x()<VPposition.x()){
      virtCursor.setX (VPposition.x());
    } else if(virtCursor.x()>VPposition.x()+refwidget->width()){
      virtCursor.setX (VPposition.x()+refwidget->width());
    }

    if (virtCursor.y()<VPposition.y()){
      virtCursor.setY (VPposition.y());
    } else if(virtCursor.y()>VPposition.y()+refwidget->height()){
      virtCursor.setY (VPposition.y()+refwidget->height());
    }

    refwidget->cursor().setPos(virtCursor);
    updateCursorLater=false;
  }
  return true; 
}

void CADCamera2::translate(float forward, float up, float sideways) {
  // forward = -z, sideways = x , up = y. Remember: inverse of yaw is applied, i.e., we have to apply yaw (?)
  // Also keep in mind: sin(-alpha) = -sin(alpha) and cos(-alpha) = -cos(alpha)
  // We only apply the yaw to move along the yaw direction;
  //  x' = x*cos(yaw) - z*sin(yaw)
  //  z' = x*sin(yaw) + z*cos(yaw)
  float s = std::sin(yaw_);
  float c = std::cos(yaw_);

  x_ += sideways * c - forward * s;
  y_ += up;
  z_ -= sideways * s + forward * c;
}

/*****************************************************************************/

void CADCamera2::rotate(float yaw, float pitch) {
  yaw_ += yaw;
  pitch_ += pitch;
  if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
  if (pitch_ > M_PI_2) pitch_ = M_PI_2;
}

bool CADCamera2::mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  mutex_.lock();

  // TODO: expose parameters:
  static const float MIN_MOVE = 0;
  //  static const float WALK_SENSITIVITY = 0.5f;
  //  static const float TURN_SENSITIVITY = 0.01f;
  //  static const float SLIDE_SENSITIVITY = 0.5f;
  static const float SLIDEX_SENSITIVITY = 1.f;
  static const float SLIDEY_SENSITIVITY = 1.f;
  static const float SLIDEU_SENSITIVITY = 1.f;
  //  static const float RAISE_SENSITIVITY = 0.5f;

  static const float LOOK_SENSITIVITY = 0.01f;
  static const float FREE_TURN_SENSITIVITY = 0.01f;

  float dx = x - startx_;
  float dy = y - starty_;

  if (dx > 0.0f) dx = std::max(0.0f, dx - MIN_MOVE);
  if (dx < 0.0f) dx = std::min(0.0f, dx + MIN_MOVE);
  if (dy > 0.0f) dy = std::max(0.0f, dy - MIN_MOVE);
  if (dy < 0.0f) dy = std::min(0.0f, dy + MIN_MOVE);

  // idea: if the velocity changes, we have to reset the start_time and update the camera parameters.

  if (btn == MouseButton::RightButton) {
    // translate(TURN_SENSITIVITY * dx,0,TURN_SENSITIVITY * dy);
    float sideways = SLIDEX_SENSITIVITY *  dx * (-1);
    float forward = SLIDEY_SENSITIVITY *  dy * std::sin(pitch_) * (-1);
    float up =  SLIDEU_SENSITIVITY * dy * std::cos(pitch_);
    float s = std::sin(yaw_);
    float c = std::cos(yaw_);

    //float factor = (startcy_ > 50 ? 25 : (startcy_ > 1 ? startcy_ * 0.5 : .5));
    //float upfactor = (startcy_ > 10 ? 5 : (startcy_ > 1 ? startcy_ * 0.5 : .5));

    // std::cout<<x_<<"\t"<<y_<<"\t"<<z_<<"\t"<<factor<<std::endl;

    //x_ = startcx_ + (sideways * c - forward * s) * factor;
    //y_ = startcy_ + up * upfactor;
    //z_ = startcz_ - (sideways * s - forward * c * (-1)) * factor;

    x_ += (sideways * c - forward * s) * my_factor;
    y_ += up * my_factor;
    z_ -= (sideways * s - forward * c * (-1)) * my_factor;
    virtCursor+=QPoint(dx/2,dy/2);
    refwidget->cursor().setPos(refwidget->mapToGlobal(QPoint(startx_,starty_)));

    //} else if (btn == MouseButton::LeftButton) {

  } else if (btn == MouseButton::MiddleButton) {
 

    // ensure valid values.

    float sideways1 = refp.x();
    float forward1 = refp.y() * std::sin(pitch_);
    float up1 =  refp.y() * std::cos(pitch_);

    float forward = -refp.z() * std::cos(pitch_);
    float up = refp.z() * std::sin(pitch_);
    float s = std::sin(yaw_);
    float c = std::cos(yaw_);

    x_ += (sideways1 * c) + (forward1 * s) - (forward * s);
    y_ += up1-up;
    z_ += -(sideways1 * s) + (forward1 * c) - (forward * c);

    yaw_ -= FREE_TURN_SENSITIVITY * dx;
    pitch_ -= LOOK_SENSITIVITY * dy;

    if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
    if (pitch_ > M_PI_2) pitch_ = M_PI_2;

    sideways1 = refp.x();
    forward1 = refp.y() * std::sin(pitch_);
    up1 =  refp.y() * std::cos(pitch_);

    forward = -refp.z() * std::cos(pitch_);
    up = refp.z() * std::sin(pitch_);
    s = std::sin(yaw_);
    c = std::cos(yaw_);

    x_ -= (sideways1 * c) + (forward1 * s) - (forward * s);
    y_ -= up1-up;
    z_ -= -(sideways1 * s) + (forward1 * c) - (forward * c);

    refwidget->cursor().setPos(refwidget->mapToGlobal(QPoint(startx_,starty_)));

  }

  mutex_.unlock();

  return true;
}

bool CADCamera2::wheelEvent(float delta, KeyboardModifier modifier) {
  mutex_.lock();
  static const float ZOOM_SENSITIVITY = 3.f;
  // move along the viewing direction specified by yaw and pitch.

  float forward = ZOOM_SENSITIVITY * delta * std::cos(pitch_);
  float up = ZOOM_SENSITIVITY * delta * std::sin(pitch_) * (-1);
  float s = std::sin(yaw_);
  float c = std::cos(yaw_);

  x_ -= forward * s;
  y_ -= up;
  z_ += forward * c * (-1);
  // TODO: implement me!


  if(*refpt!=Eigen::Vector4f::Zero()){
    Eigen::Vector4f refp;
    refp<<refpt->x(),-refpt->y(),1,1;
    refp = projection->inverse()*refp;
    refp /= refp.w();
    //std::cout<<refp.x()/refp.z()<<std::endl;
    //std::cout<<refp.y()/refp.z()<<std::endl;
    //my_factor = std::abs(refp.z()/refwidget->height());
    //std::cout<<*projection<<std::endl;
  
    float sideways = refp.x()/refp.z()* ZOOM_SENSITIVITY * delta * (-1);
    float forward = refp.y()/refp.z()* ZOOM_SENSITIVITY * delta * std::sin(pitch_) * (-1);
    float up =  refp.y()/refp.z()* ZOOM_SENSITIVITY * delta * std::cos(pitch_);
    float s = std::sin(yaw_);
    float c = std::cos(yaw_);

    x_ += (sideways * c - forward * s);
    y_ += up;
    z_ -= (sideways * s - forward * c * (-1));

  }

  mutex_.unlock();

  return true;
}

bool CADCamera2::keyPressed(KeyboardKey key, KeyboardModifier modifier) {
  float factor = (y_ > 50 ? 50 : (y_ > 1 ? y_ : 1));
  switch (key) {
    case KeyboardKey::KeyA:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = -10 * factor;
      return true;
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = 10 * factor;
      return true;
    case KeyboardKey::KeyW:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = 10 * factor;
      return true;
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = -10 * factor;
      return true;
    default:
      return false;
  }
}

bool CADCamera2::keyReleased(KeyboardKey key, KeyboardModifier modifier) {
  switch (key) {
    case KeyboardKey::KeyA:
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();

      sideVel_ = 0;
      if (forwardVel_ == 0) startdrag_ = false;
      return true;
    case KeyboardKey::KeyW:
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();

      forwardVel_ = 0;
      if (sideVel_ == 0) startdrag_ = false;
      return true;
    default:
      return false;
  }
}

/* namespace rv */
