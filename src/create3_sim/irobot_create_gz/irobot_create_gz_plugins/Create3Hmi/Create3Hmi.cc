/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "Create3Hmi.hh"

#include <gz/common/Console.hh>
#include <gz/msgs/int32.pb.h>

#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>

#ifdef IROBOT_CREATE_USE_IGNITION_GUI
namespace create3_gui = ignition::gui;
namespace create3_msgs = ignition::msgs;
namespace create3_transport = ignition::transport;
#define CREATE3HMI_ERR ignerr
#define CREATE3HMI_LOG ignlog
#else
namespace create3_gui = gz::gui;
namespace create3_msgs = gz::msgs;
namespace create3_transport = gz::transport;
#define CREATE3HMI_ERR gzerr
#define CREATE3HMI_LOG gzlog
#endif

namespace irobot_create_gz_plugins
{
Create3Hmi::Create3Hmi()
  : create3_gui::Plugin()
{
  this->create3_button_pub_ = create3_transport::Node::Publisher();
  this->create3_button_pub_ =
    this->node_.Advertise<create3_msgs::Int32>(this->create3_button_topic_);
}

Create3Hmi::~Create3Hmi()
{
}

void Create3Hmi::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Create3 HMI";
  }

  if (_pluginElem)
  {
    auto namespaceElem = _pluginElem->FirstChildElement("namespace");
    if (nullptr != namespaceElem && nullptr != namespaceElem->GetText())
      this->SetNamespace(namespaceElem->GetText());
  }

  this->connect(
    this, SIGNAL(AddMsg(QString)), this, SLOT(OnAddMsg(QString)),
    Qt::QueuedConnection);
}

void Create3Hmi::OnCreate3Button(const int button)
{
  create3_msgs::Int32 button_msg;

  button_msg.set_data(button);

  if (!this->create3_button_pub_.Publish(button_msg)) {
    CREATE3HMI_ERR << "gz::msgs::Int32 message couldn't be published at topic: " <<
      this->create3_button_topic_ << std::endl;
  }
}

QString Create3Hmi::Namespace() const
{
  return QString::fromStdString(this->namespace_);
}

void Create3Hmi::SetNamespace(const QString &_name)
{
  this->namespace_ = _name.toStdString();
  this->create3_button_topic_ = this->namespace_ + "/create3_buttons";

  CREATE3HMI_LOG << "A new robot name has been entered, publishing on topic: '" <<
      this->create3_button_topic_ << " ' " <<std::endl;

  // Update publisher with new topic.
  this->create3_button_pub_ = create3_transport::Node::Publisher();
  this->create3_button_pub_ =
      this->node_.Advertise<create3_msgs::Int32>
      (this->create3_button_topic_);
  if (!this->create3_button_pub_)
  {
    create3_gui::App()->findChild<create3_gui::MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Error when advertising topic: " +
        this->create3_button_topic_), 4000);
    CREATE3HMI_ERR << "Error when advertising topic: " <<
      this->create3_button_topic_ << std::endl;
  }else {
    create3_gui::App()->findChild<create3_gui::MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Advertising topic: '<b>" +
        this->create3_button_topic_ + "</b>'"), 4000);
  }
  this->NamespaceChanged();
}

}  // namespace irobot_create_gz_plugins

// Register this plugin
#ifdef IROBOT_CREATE_USE_IGNITION_GUI
IGNITION_ADD_PLUGIN(
  irobot_create_gz_plugins::Create3Hmi,
  create3_gui::Plugin)
#else
GZ_ADD_PLUGIN(
  irobot_create_gz_plugins::Create3Hmi,
  create3_gui::Plugin)
#endif
