#include "ImageViewer.h"
#include "data/label_utils.h"

ImageViewer::ImageViewer(QWidget* parent, Qt::WindowFlags f) : QWidget(parent, f) {}

void ImageViewer::setImage(const std::string& filename) {
  currentImage_ = QPixmap(QString::fromStdString(filename));
  update();
}

QRgb convert(const glow::GlColor& c) { return qRgb(c.R * 255, c.G * 255, c.B * 255); }

// if(label == 0) vs_out.label = 40;
// if(label == 1) vs_out.label = 48;
// if(label == 2) vs_out.label = 50;
// if(label == 4) vs_out.label = 51;
// if(label == 13) vs_out.label = 10;
// if(label == 8) vs_out.label = 70;
// if(label == 11) vs_out.label = 30;
// if(label == 18) vs_out.label = 11;
// if(label == 17) vs_out.label = 15;

void ImageViewer::keyPressEvent(QKeyEvent* event) {
  showLabels_ = !showLabels_;
  update();
}

void ImageViewer::setLabels(const std::string& filename) {
  std::map<uint32_t, glow::GlColor> colors;
  getLabelColors("labels.xml", colors);

  currentLabels_ = QPixmap(QString::fromStdString(filename));
  QImage img = currentLabels_.toImage();
  for (int32_t x = 0; x < img.width(); ++x) {
    for (int32_t y = 0; y < img.height(); ++y) {
      uint32_t label = qRed(img.pixel(x, y));
      if (label == 0) img.setPixel(x, y, convert(colors[40]));
      if (label == 1) img.setPixel(x, y, convert(colors[48]));
      if (label == 2) img.setPixel(x, y, convert(colors[50]));
      if (label == 4) img.setPixel(x, y, convert(colors[51]));
      if (label == 13) img.setPixel(x, y, convert(colors[10]));
      if (label == 8) img.setPixel(x, y, convert(colors[70]));
      if (label == 11) img.setPixel(x, y, convert(colors[30]));
      if (label == 18) img.setPixel(x, y, convert(colors[11]));
      if (label == 17) img.setPixel(x, y, convert(colors[15]));
    }
  }
  currentLabels_.fromImage(img);
  update();
}

void ImageViewer::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  if (showLabels_) painter.drawPixmap(0, 0, width(), height(), showLabels_ ? currentLabels_ : currentImage_);
}

void ImageViewer::resizeEvent(QResizeEvent* event) { update(); }
