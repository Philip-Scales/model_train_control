#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <QObject>
#include <QTimer>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "ros/ros.h"

//Communicator is responsible for translating signals emitted by the Handler 
//(in response to user interactions with the UI) into ROS messages that are published to the appropriate topics. 
//It also handles incoming ROS messages if necessary (e.g., to update the UI based on changes in the system state).

class Communicator : public QObject { 
    Q_OBJECT

public:
    Communicator(QObject *_parent = nullptr);

public slots: 
    void reportDirStopClicked();
    void reportDirForwardClicked();
    void reportDirBackwardClicked();
    void reportPoint1ButtonClicked();
    void reportPoint2ButtonClicked();
    void reportShortWhistleClicked();
    void reportStationDepartClicked();
    void reportStationArriveClicked();

    void run();

    void handleSpeedChange(double _new_speed);
    void handleLocoSelected(const QString &loco_name);

private:
    ros::NodeHandle *m_ros_node_handle;
    ros::Publisher m_action_publisher;
    ros::Publisher m_twist_publisher;
    ros::Publisher m_translation_emergency_stop_publisher;
    ros::Publisher m_pub_throttle_slider;
    ros::Publisher m_pub_point_command;
    ros::Publisher m_pub_sound_id;
    ros::Publisher m_pub_action;
    ros::Publisher m_pub_loco_change;  // new publisher for selected loco

    QTimer *m_emergency_stop_timer = nullptr;

private slots:
    void onTimeout();


};

#endif //COMMUNICATOR_H
