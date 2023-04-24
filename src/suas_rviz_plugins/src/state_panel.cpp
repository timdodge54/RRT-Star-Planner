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

#include "suas_rviz_plugins/state_panel.hpp"

using std::placeholders::_1;

namespace suas_rviz_plugins
{

StatePanelNode::StatePanelNode()
  : Node("control_panel_node")
{
  this->pub_state = this->create_publisher<uav_interfaces::msg::UavState>("/uav_state",1);

  // Initialize states
  north = std::make_shared<float>(0.0);
  east  = std::make_shared<float>(0.0);
  down  = std::make_shared<float>(0.0);
  phi   = std::make_shared<float>(0.0);
  theta = std::make_shared<float>(0.0);
  psi   = std::make_shared<float>(0.0);
  alpha = std::make_shared<float>(0.0);
  beta  = std::make_shared<float>(0.0);
}

void StatePanelNode::resetState() {
  // Create and publish the state
  uav_interfaces::msg::UavState state;
  state.pose.header.frame_id = "ned";
  state.pose.header.stamp = this->get_clock()->now();
  state.pose.pose.position.x = *(this->north);
  state.pose.pose.position.y = *(this->east);
  state.pose.pose.position.z = *(this->down);
  state.phi = *(this->phi);
  state.theta = *(this->theta);
  state.psi = *(this->psi);
  state.alpha = *(this->alpha);
  state.beta = *(this->beta);
  this->pub_state->publish(state);
}

StateEditor::StateEditor(std::shared_ptr<StatePanelNode> node, const std::string title,
                         std::shared_ptr<float> value, QVBoxLayout * layout, float max_float)
  : node_(node), value(value)
{
  // Create the north text
  QHBoxLayout* state_val = new QHBoxLayout;
  state_val->addWidget( new QLabel( title.c_str() ));
  box_ = new QLineEdit;
  std::stringstream text;
  text << "0";
  box_->insert(text.str().c_str());
  state_val->addWidget( box_ );

  // Create the slider
  int max_slider_val = 1000;
  this->slider_ = new QSlider(Qt::Horizontal);
  this->slider_->setMinimum(-max_slider_val);
  this->slider_->setMaximum(max_slider_val);
  this->slider_->setValue(0);

  // Create the slider conversions
  float max_slider_float = static_cast<float>(max_slider_val);
  this->slider_to_value = max_float / max_slider_float;
  this->value_to_slider = max_slider_float / max_float;

  // Update the layout
  layout->addLayout(state_val);
  layout->addWidget(this->slider_);

  // Next we make signal/slot connections.
  connect( box_, SIGNAL( editingFinished() ), this, SLOT( stateChanged() ));
  connect( slider_, &QSlider::sliderMoved, this, &StateEditor::sliderChanged);
}

void StateEditor::stateChanged() {
  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = this->box_->text().toFloat(&converted);
  std::stringstream text;
  text << output;
  box_->clear();
  box_->insert(text.str().c_str());

  // Set the slider position
  int value = static_cast<int>(output*this->value_to_slider);
  this->slider_->setSliderPosition(value);

  // Set the state
  *(this->value) = output;
  this->node_->resetState();
}

void StateEditor::sliderChanged(int value) {
  // Read in slider position
  //int output = this->north_slider_->sliderPosition();
  float output = static_cast<float>(value) * this->slider_to_value;

  // Update the text box
  std::stringstream text;
  text << output;
  box_->clear();
  box_->insert(text.str().c_str());

  // Set the state
  *(this->value) = output;
  this->node_->resetState();
}

StatePanel::StatePanel( QWidget* parent )
  : rviz_common::Panel( parent ),
  spin_count_(0)
{
  // Create the ros node
  node_ = std::make_shared<StatePanelNode>();
  executor_.add_node(node_);

  // Create overall layout
  QVBoxLayout* layout = new QVBoxLayout;
  editors.push_back(std::make_shared<StateEditor>(this->node_, "north:", this->node_->north, layout,1000.) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "east :", this->node_->east , layout,1000.) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "down :", this->node_->down , layout,1000.) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "phi  :", this->node_->phi  , layout,3.14) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "theta:", this->node_->theta, layout,3.14) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "psi  :", this->node_->psi  , layout,3.14) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "alpha:", this->node_->alpha, layout,3.14) );
  editors.push_back(std::make_shared<StateEditor>(this->node_, "beta :", this->node_->beta , layout,3.14) );
  setLayout( layout );

  // Create the timer
  spin_timer_ = new QTimer( this );

  // Next we make signal/slot connections.
  connect( spin_timer_, SIGNAL( timeout() ), this, SLOT( spin() ));

  // Start the timer (time in milliseconds)
  spin_timer_->start( 100 );
}

void StatePanel::spin() {
  executor_.spin_some();
}

} // end namespace suas_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(suas_rviz_plugins::StatePanel, rviz_common::Panel )
