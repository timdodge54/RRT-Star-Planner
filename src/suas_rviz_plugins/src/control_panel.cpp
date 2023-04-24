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

#include "suas_rviz_plugins/control_panel.hpp"

using std::placeholders::_1;

namespace suas_rviz_plugins
{

ControlPanelNode::ControlPanelNode()
  : Node("control_panel_node")
{
  this->reset_client =
    this->create_client<std_srvs::srv::Empty>("reset_state");
  this->kf_reset_client =
    this->create_client<std_srvs::srv::Empty>("kalman_filter_reset_srv_topic");
  this->toggle_client =
    this->create_client<std_srvs::srv::Empty>("toggle_execution");
  this->kf_toggle_client =
    this->create_client<std_srvs::srv::Empty>("kalman_filter_start_pause_srv_topic");
  this->clock_client =
    this->create_client<uav_interfaces::srv::SetClockParams>("set_clock_params");
}

void ControlPanelNode::startStopDynamics() {
  if(toggle_client->wait_for_service(1s)) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = toggle_client->async_send_request(request);
    RCLCPP_DEBUG(this->get_logger(), "Toggle service sent");
  } else {
    RCLCPP_WARN(this->get_logger(), "Toggle service not available");
  }
  if(kf_toggle_client->wait_for_service(1s)) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = kf_toggle_client->async_send_request(request);
    RCLCPP_DEBUG(this->get_logger(), "Toggle service sent to kalman filter");
  }
}

void ControlPanelNode::resetState() {
  if(reset_client->wait_for_service(1s)) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = reset_client->async_send_request(request);
    RCLCPP_DEBUG(this->get_logger(), "Reset service sent");
  } else {
    RCLCPP_WARN(this->get_logger(), "Reset service not available");
  }
  if(kf_reset_client->wait_for_service(1s)) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = kf_reset_client->async_send_request(request);
    RCLCPP_DEBUG(this->get_logger(), "Reset service sent to kalman filter");
  }
}

bool ControlPanelNode::setClockParams(float clock_scale) {
  // Check for service
  if(clock_client->wait_for_service(1s)) {
    // Create the service request
    auto request = std::make_shared<uav_interfaces::srv::SetClockParams::Request>();
    request->time_scale = clock_scale;
    auto result = clock_client->async_send_request(request);

    // Indicate that service call was successfully made
    RCLCPP_DEBUG(this->get_logger(), "Clock set sent");
    return true;
  } else {

    // Indicate that the service call was not made
    RCLCPP_WARN(this->get_logger(), "Clock set service not available");
    return false;
  }
}

ControlPanel::ControlPanel( QWidget* parent )
  : rviz_common::Panel( parent ),
  start_state_(true),
  new_clock_received_(false),
  clock_scale_(1.0),
  spin_count_(0)
{
  // Create the ros node
  node_ = std::make_shared<ControlPanelNode>();
  executor_.add_node(node_);

  // Create the start/stop button
  toggle_button_ = new QToolButton();
  toggle_button_->setText("Start");
  toggle_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Create the reset button
  reset_button_ = new QToolButton();
  reset_button_->setText("Reset");
  reset_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Create the clock scale input
  QHBoxLayout* time_scale = new QHBoxLayout;
  time_scale->addWidget( new QLabel( "Sim speed:" ));
  clock_scale_box_ = new QLineEdit;
  std::stringstream text;
  text << this->clock_scale_;
  clock_scale_box_->insert(text.str().c_str());
  time_scale->addWidget( clock_scale_box_ );


  // Create the button layout
  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( toggle_button_);
  button_layout->addWidget( reset_button_);

  // Create overall layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( button_layout );
  layout->addLayout( time_scale );
  setLayout( layout );

  // Create the timer
  spin_timer_ = new QTimer( this );

  // Next we make signal/slot connections.
  connect( toggle_button_, SIGNAL( clicked() ), this, SLOT( toggleDynamicsClicked() ) );
  connect( reset_button_, SIGNAL( clicked() ), this, SLOT( resetClicked() ) );
  connect( spin_timer_, SIGNAL( timeout() ), this, SLOT( spin() ));
  connect( clock_scale_box_, SIGNAL( editingFinished() ), this, SLOT( process_clock_scale() ));

  // Start the timer (time in milliseconds)
  spin_timer_->start( 100 );
}

void ControlPanel::toggleDynamicsClicked() {
  node_->startStopDynamics();
  this->start_state_ = !this->start_state_;

  if(this->start_state_) {
    this->toggle_button_->setText("Start");
  } else {
    this->toggle_button_->setText("Pause");
  }
}

void ControlPanel::resetClicked() {
  node_->resetState();
}

void ControlPanel::process_clock_scale() {
  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = clock_scale_box_->text().toFloat(&converted);

  // If possible then store the value
  if(converted && output > 0.0f) {
    this->clock_scale_ = output;
    new_clock_received_ = true;
  }

  // Put the z_value back in the box
  std::stringstream text;
  text << this->clock_scale_;
  clock_scale_box_->clear();
  clock_scale_box_->insert(text.str().c_str());
}

void ControlPanel::spin() {
  executor_.spin_some();

  // Set the clock variables
  if(this->new_clock_received_){
    bool result = this->node_->setClockParams(this->clock_scale_);
    this->new_clock_received_ = !result;
  }
}

} // end namespace suas_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(suas_rviz_plugins::ControlPanel, rviz_common::Panel )
