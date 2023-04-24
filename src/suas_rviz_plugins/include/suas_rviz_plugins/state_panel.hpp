#ifndef STATE_PANEL_H
#define STATE_PANEL_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include <QToolButton>
#include <QLineEdit>
#include <QSlider>
#include <QVBoxLayout>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "uav_interfaces/srv/set_clock_params.hpp"
#include "uav_interfaces/msg/uav_state.hpp"

using namespace std::chrono_literals;

namespace suas_rviz_plugins {

class StatePanelNode: public rclcpp::Node
{
public:
  /*!
   * \brief Initialize the publishers and subscribers
   */
  StatePanelNode();

  /*!
  * \brief Call service in dynamics to reset the state to the desired value
  */
  void resetState();

  // Create states
  std::shared_ptr<float> north;
  std::shared_ptr<float> east;
  std::shared_ptr<float> down;
  std::shared_ptr<float> phi;
  std::shared_ptr<float> theta;
  std::shared_ptr<float> psi;
  std::shared_ptr<float> alpha;
  std::shared_ptr<float> beta;


private:
  /*!
  * \param publisher for the uav state
  */
  rclcpp::Publisher<uav_interfaces::msg::UavState>::SharedPtr pub_state;
};

class StateEditor: public QWidget {
  Q_OBJECT

public:
  StateEditor(std::shared_ptr<StatePanelNode> node, const std::string title,
              std::shared_ptr<float> value, QVBoxLayout * layout,
              float max_float);

protected Q_SLOTS:
  /*!
  * \brief changes the state
  */
  void stateChanged();

  /*!
  * \brief changes the state based upon the slider
  */
  void sliderChanged(int value);

protected:
  std::shared_ptr<StatePanelNode> node_;
  std::shared_ptr<float> value;
  float slider_to_value;
  float value_to_slider;
  QLineEdit * box_; // Text box for entry of sim speed (1.0 is real-time)
  QSlider * slider_; // Slider for the north position
};

class StatePanel: public rviz_common::Panel
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
  StatePanel( QWidget* parent = 0 );

  // Declare internal slots
protected Q_SLOTS:

  /*!
  * \brief Spins the ros node
  */
  void spin();

  // Define member variables
protected:
  // Text boxes for state values
//   QLineEdit * north_box_; // Text box for entry of sim speed (1.0 is real-time)
//   QSlider * north_slider_; // Slider for the north position
  std::vector<std::shared_ptr<StateEditor> > editors;

  // Timer for calling ROS spin
  QTimer* spin_timer_;

  // The number of times the panel has called spin
  int spin_count_;

  // Ros node and executor
  std::shared_ptr<StatePanelNode> node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

};

} // end suas_rviz_plugins namespace

#endif // STATE_PANEL_H
