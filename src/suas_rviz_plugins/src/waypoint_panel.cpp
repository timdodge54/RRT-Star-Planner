#include <stdio.h>
#include <iostream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "suas_rviz_plugins/waypoint_panel.hpp"

using std::placeholders::_1;

bool is_number(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
}

namespace suas_rviz_plugins
{

WaypointPanelNode::WaypointPanelNode()
  : Node("waypoint_panel_node"), z_value(0.0)
{
  // Create the path publishers
  this->path_pub_viz_ = this->create_publisher<nav_msgs::msg::Path>("wp_panel/waypoint_viz", 1);
  this->path_pub_cmd_ = this->create_publisher<nav_msgs::msg::Path>("wp_panel/waypoint_cmd", 1);
  this->pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("wp_panel/goal_pose", 1);
  this->sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10,
    std::bind(&WaypointPanelNode::goal_callback, this, _1));
  path_.header.frame_id = "ned"; // All points are transformed into ned


  // Initialize tf
  this->tf_buffer =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->transform_listener =
    std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer));
}

void WaypointPanelNode::publishVisualizationPath() {
  path_.header.stamp = this->now();
  this->path_pub_viz_->publish(path_);
}

void WaypointPanelNode::publishCommandPath() {
  path_.header.stamp = this->now();
  this->path_pub_cmd_->publish(this->path_);
  this->pub_goal_->publish(this->latest_goal_);
}

void WaypointPanelNode::clearPath() {
  this->path_.poses.clear();
}

void WaypointPanelNode::goal_callback(const geometry_msgs::msg::PoseStamped & msg) {
  geometry_msgs::msg::PoseStamped input = msg;
  input.pose.position.z = this->z_value;

  // Convert to the ned frame
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = tf_buffer->transform<geometry_msgs::msg::PoseStamped>(input, path_.header.frame_id, tf2::durationFromSec(1.0));
    this->path_.poses.push_back(pose_out);
    this->latest_goal_ = pose_out;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not transform to " <<  path_.header.frame_id << ", "
      << ex.what() << ", ignoring input pose");
  }
}

WaypointPanel::WaypointPanel( QWidget* parent )
  : rviz_common::Panel( parent ), z_value_(100.0), spin_count_(0)
{
  // Create the ros node
  node_ = std::make_shared<WaypointPanelNode>();
  executor_.add_node(node_);
  node_->z_value = this->z_value_;

  // Create the z-value text box
  QHBoxLayout* z_value_layout = new QHBoxLayout;
  z_value_layout->addWidget( new QLabel( "z-value:" ));
  z_value_box_ = new QLineEdit;
  std::stringstream text;
  text << this->z_value_;
  z_value_box_->insert(text.str().c_str());
  z_value_layout->addWidget( z_value_box_ );

  // Create the publish button
  publish_button_ = new QToolButton();
  publish_button_->setText("Publish");
  publish_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Create the clear button
  clear_button_ = new QToolButton();
  clear_button_->setText("Clear");
  clear_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Create the button layout
  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( publish_button_);
  button_layout->addWidget( clear_button_);

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( z_value_layout );
  layout->addLayout( button_layout );
  setLayout( layout );

  // Create the timer
  spin_timer_ = new QTimer( this );

  // Next we make signal/slot connections.
  connect( publish_button_, SIGNAL( clicked() ), this, SLOT( publishClicked() ) );
  connect( clear_button_, SIGNAL( clicked() ), this, SLOT( clearClicked() ) );
  connect( spin_timer_, SIGNAL( timeout() ), this, SLOT( spin() ));
  connect( z_value_box_, SIGNAL( editingFinished() ), this, SLOT( processNewZ() ));

  // Start the timer (time in milliseconds)
  spin_timer_->start( 100 );
}

void WaypointPanel::publishClicked() {
  node_->publishCommandPath();
}

void WaypointPanel::clearClicked() {
  node_->clearPath();
}

void WaypointPanel::processNewZ() {

  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = z_value_box_->text().toFloat(&converted);

  // If possible then store the z_value
  if(converted) {
    this->z_value_ = output;
  }

  // Put the z_value back in the box
  std::stringstream text;
  text << this->z_value_;
  z_value_box_->clear();
  z_value_box_->insert(text.str().c_str());

  // Set the z-value in the node
  this->node_->z_value = this->z_value_;
}

void WaypointPanel::spin() {
  executor_.spin_some();
  node_->publishVisualizationPath();
}

} // end namespace suas_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(suas_rviz_plugins::WaypointPanel,rviz_common::Panel )
