#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <malloc.h>


class mainControl
{
public:
    mainControl();
    ~mainControl();

    void callback_localization_POS_T(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void callback_make_path(const std_msgs::String::ConstPtr& msg);
    void callback_load_path(const std_msgs::String::ConstPtr& msg);
    void callback_tracking(const std_msgs::String::ConstPtr& msg);
    void callback_currIndex_init(const std_msgs::String::ConstPtr& msg);
    void path_making();
    void path_loading();
    void tracking();

    void getDesiredSpeednSteering(double desired_x, double desired_y, double dt);

    // POS_T topic subscribe variable
    double gps_x;
    double gps_y;
    double gps_heading;
    double pos_x;
    double pos_y;
    double pos_heading;

    double get_position;
    double get_heading;
    double POS_init;
    double GPS_init;
    double vehicle_speed;

    double msg_count_test;

    // make path & load path topic subscribe variable
    bool m_bMakePathFlag;
    FILE *pFileWayPoint, *pFileWayPointLoad;
    int m_index;
    double **making_path_node;
    double twoPointDist;
    double way_point_gap; // parameter

    bool m_bLoadPathFlag;
    double **path_node;
    int nr_data;

    // tracking waypoint variable
    bool m_bTrackingWPFlag;
    int m_curr_index;
    float m_globalspeed; // parameter
    double distTemp[10];
    double min;
    double controlPointX;
    double controlPointY;

    double desired_x;
    double desired_y;

    double m_wheel_base; // parameter
    int number_Of_Lookahead_Wp; // parameter
    double acceleration; // parameter

    double scooter_speed; // km/h
    double vd;
    double scooter_steering; // degree
    
};

#endif