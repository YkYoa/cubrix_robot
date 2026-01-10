#include <QApplication>
#include <QSocketNotifier>
#include <rclcpp/rclcpp.hpp>
#include "ar_ui/param_editor_window.h"

#include <csignal>
#include <unistd.h>
#include <sys/socket.h>

// Signal handling via socketpair for Qt integration
static int signal_fd[2];

static void signalHandler(int)
{
  // Write a byte to wake up the Qt event loop
  char c = 1;
  (void)write(signal_fd[1], &c, 1);
}

static void setupUnixSignalHandlers()
{
  // Create socketpair for signal-safe communication with Qt
  if (socketpair(AF_UNIX, SOCK_STREAM, 0, signal_fd) != 0) {
    return;
  }

  // Install signal handlers for SIGINT (Ctrl+C) and SIGTERM
  struct sigaction sa;
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  
  // Setup Unix signal handling
  setupUnixSignalHandlers();
  
  // Create socket notifier to receive signals in Qt event loop
  QSocketNotifier* sn = new QSocketNotifier(signal_fd[0], QSocketNotifier::Read, &app);
  QObject::connect(sn, &QSocketNotifier::activated, [&]() {
    // Consume the signal byte
    char c;
    (void)read(signal_fd[0], &c, 1);
    
    // Trigger clean shutdown
    QApplication::quit();
  });
  
  ar_ui::ParamEditorWindow window;
  window.show();
  
  int ret = app.exec();
  
  // Cleanup
  close(signal_fd[0]);
  close(signal_fd[1]);
  rclcpp::shutdown();
  return ret;
}
