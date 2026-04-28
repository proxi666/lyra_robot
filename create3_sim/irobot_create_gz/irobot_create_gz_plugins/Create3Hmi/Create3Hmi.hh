/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_GZ__IROBOT_CREATE_GZ_PLUGINS__CREATE3HMI__CREATE3HMI_HH_
#define IROBOT_CREATE_GZ__IROBOT_CREATE_GZ_PLUGINS__CREATE3HMI__CREATE3HMI_HH_

#include <gz/gui/qt.h>

#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

#ifdef IROBOT_CREATE_USE_IGNITION_GUI
namespace create3_gui = ignition::gui;
namespace create3_transport = ignition::transport;
#else
namespace create3_gui = gz::gui;
namespace create3_transport = gz::transport;
#endif

namespace irobot_create_gz_plugins
{

class Create3Hmi : public create3_gui::Plugin
{
  Q_OBJECT

  // \brief Name
  Q_PROPERTY(
    QString name
    READ Namespace
    WRITE SetNamespace
    NOTIFY NamespaceChanged
  )

public:
  /// \brief Constructor
  Create3Hmi();
  /// \brief Destructor
  virtual ~Create3Hmi();
  /// \brief Called by Ignition GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  // \brief Get the robot name as a string, for example
  /// '/echo'
  /// \return Name
  Q_INVOKABLE QString Namespace() const;

public slots:
  /// \brief Callback in Qt thread when the robot name changes.
  /// \param[in] _name variable to indicate the robot name to
  /// publish the Button commands.
  void SetNamespace(const QString &_name);

signals:
  /// \brief Notify that robot name has changed
  void NamespaceChanged();

protected slots:
  /// \brief Callback trigged when the button is pressed.
  void OnCreate3Button(const int button);

private:
  create3_transport::Node node_;
  create3_transport::Node::Publisher create3_button_pub_;
  std::string namespace_ = "";
  std::string create3_button_topic_ = "/create3_buttons";
};

}  // namespace irobot_create_gz_plugins

#endif  // IROBOT_CREATE_GZ__IROBOT_CREATE_GZ_PLUGINS__CREATE3HMI__CREATE3HMI_HH_
