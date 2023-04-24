#ifndef WAYPOINT_PANEL_H
#define WAYPOINT_PANEL_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include <QToolButton>
#include <QLineEdit>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace suas_rviz_plugins {

class WaypointPanelNode: public rclcpp::Node
{
public:
  /*!
   * \brief Initialize the publishers and subscribers
   */
  WaypointPanelNode();

  /*!
  * \brief Publish the latest path on the command message
  */
  void publishCommandPath();

  /*!
  * \brief Publish the latest path on the visualization message
  */
  void publishVisualizationPath();

  /*!
  * \brief Clears all of the nodes from the current path
  */
  void clearPath();

  /*!
  * \brief Provides a pose to be appended to the end of the path
  * \param msg Goal pose to be appended
  */
  void goal_callback(const geometry_msgs::msg::PoseStamped & msg);

  /*!
  * \brief The z-value to associate with the waypoint
  */
  float z_value;


private:
  /*!
  * \param path_pub_viz_ Publishes the resulting path as it is created for visualization
  * \param path_pub_cmd_ Publishes the resulting path when the publish button is clicked
  * \param pub_goal_ Publishes the latest received goal
  * \param sub_goal_ Subscribes to a poseStamped used to create the waypoints
  * \param path_ Stores the most up-to-date path information
  * \param latest_goal_ Stores the latest goal
  * \param transform_listener Generic transform listener
  * \param tf_buffer Buffer used to transform the published goal to ned frame
  */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_viz_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

class WaypointPanel: public rviz_common::Panel
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
  WaypointPanel( QWidget* parent = 0 );


  // Declare internal slots
protected Q_SLOTS:
  /*!
  * \brief Publishes the path with the latest points
  */
  void publishClicked();

  /*!
  * \brief Clears the path of all waypoints
  */
  void clearClicked();

  /*!
  * \brief Reads in the z_value from the text box
  */
  void processNewZ();

  /*!
  * \brief Spins the ros node
  */
  void spin();

  // Define member variables
protected:
  // Buttons for publishing and clearing the path
  QToolButton * publish_button_; // Publish the path
  QToolButton * clear_button_; // Erase all data in the path

  // Defines the z_value when creating an element in the path
  QLineEdit * z_value_box_; // Text box for entry
  float z_value_;           // Latest value

  // Timer for calling ROS spin
  QTimer* spin_timer_;

  // The number of times the panel has called spin
  int spin_count_;

  // Ros node and executor
  std::shared_ptr<WaypointPanelNode> node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

};

} // end suas_rviz_plugins namespace

#endif // WAYPOINT_PANEL_H
