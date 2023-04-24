#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include <QToolButton>
#include <QLineEdit>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/empty.hpp"
#include "uav_interfaces/srv/set_clock_params.hpp"

using namespace std::chrono_literals;

namespace suas_rviz_plugins {

class ControlPanelNode: public rclcpp::Node
{
public:
  /*!
   * \brief Initialize the publishers and subscribers
   */
  ControlPanelNode();

  /*!
  * \brief Call service in dynamics to start / stop the dynamics
  */
  void startStopDynamics();

  /*!
  * \brief Call service in dynamics to reset the state
  */
  void resetState();

  /*!
  * \brief Call service to set the clock parameters
  * \param clock_scale Rate at which the sim will be run
  * \return true=> service was successfully called
  */
  bool setClockParams(float clock_scale);

private:
  /*!
  * \param reset_client Client to the uav state reset service
  * \param kf_reset_client Client to the kalman filter reset service
  * \param toggle_client Client to the uav state toggle service
  * \param kf_toggle_client Client to the kalman filter toggle service
  * \param clock_client Client to update the clock parameters
  */
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr kf_reset_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr toggle_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr kf_toggle_client;
  rclcpp::Client<uav_interfaces::srv::SetClockParams>::SharedPtr clock_client;

};

class ControlPanel: public rviz_common::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  ControlPanel( QWidget* parent = 0 );

  // Declare internal slots
protected Q_SLOTS:
  /*!
  * \brief starts / stops the dynamics when clicked
  */
  void toggleDynamicsClicked();

  /*!
  * \brief resets the dynamics when clicked
  */
  void resetClicked();

  /*!
  * \brief Spins the ros node
  */
  void spin();

  /*!
  * \brief Processes the clock scale
  */
  void process_clock_scale();

  // Define member variables
protected:
  // Buttons for publishing and clearing the path
  QToolButton * toggle_button_; // toggles the dynamic execution
  QToolButton * reset_button_; // resets the dynamics states
  bool start_state_; // true => dynamics waiting for start, false => dynamics waiting for stop/pause

  // Text boxes for clock values
  QLineEdit * clock_scale_box_; // Text box for entry of sim speed (1.0 is real-time)
  bool new_clock_received_; // true=> a new clock value was input
  float clock_scale_; // holds the scale for the clock

  // Timer for calling ROS spin
  QTimer* spin_timer_;

  // The number of times the panel has called spin
  int spin_count_;

  // Ros node and executor
  std::shared_ptr<ControlPanelNode> node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

};

} // end suas_rviz_plugins namespace

#endif // CONTROL_PANEL_H
