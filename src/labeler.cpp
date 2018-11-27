#include <QtWidgets/QApplication>
#include "widget/Mainframe.h"
#include <execinfo.h>
#include <signal.h>

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}


int main(int argc, char** argv) {
  signal(SIGSEGV, handler);
  QApplication app(argc, argv);
  app.setWindowIcon(QIcon("../assets/icon.png"));

  Mainframe frame;
  frame.show();

  return app.exec();
}
