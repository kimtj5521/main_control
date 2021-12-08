#include "main.h"

mainControl::mainControl()
{
    m_bMakePathFlag = false;
    m_index = 0;
    twoPointDist = 0.0;
    way_point_gap = 0.0;

    m_bLoadPathFlag = false;
    nr_data = 0;

    desired_x = 0.0;
    desired_y = 0.0;

    making_path_node = nullptr;
    making_path_node = new double*[500000];
    for(int i=0; i<500000; i++){
        making_path_node[i] = new double[3];
    }

    m_bTrackingWPFlag = false;
    m_curr_index = 0;
    m_globalspeed = 0.0;
    distTemp[10] = {0,};
    
    m_wheel_base = 0.0;
    number_Of_Lookahead_Wp = 0;

    acceleration = 0.0;
    vd = 0.0;
}

mainControl::~mainControl()
{
    for(int k=0; k<3; k++){
        delete making_path_node[k];
    }
    delete making_path_node;

    for(int m=0; m<3; m++){
        delete path_node[m];
    }
    delete path_node;
}

void mainControl::callback_localization_POS_T(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    pos_x = msg->data[0];
    pos_y = msg->data[1];
    pos_heading = msg->data[2];
    gps_x = msg->data[3];
    gps_y = msg->data[4];
    gps_heading = msg->data[5];

    get_position = msg->data[6];
    get_heading = msg->data[7];
    POS_init = msg->data[8];
    GPS_init = msg->data[9];
    vehicle_speed = msg->data[10];

    msg_count_test = msg->data[11];
}

void mainControl::callback_make_path(const std_msgs::String::ConstPtr& msg)
{
    if(m_bMakePathFlag == false){
        pFileWayPoint = fopen("way_point.txt", "w+t");
        m_index = 0;

        for(int i=0; i<500000; i++){
            memset(making_path_node[i], 0, sizeof(double)*3);
        }

        m_bMakePathFlag = true;
        std::cout << "path making..." << std::endl;
    }
    else{
        m_bMakePathFlag = false;
        fclose(pFileWayPoint);
        std::cout << "path making is done..." << std::endl;
    }
    
}

void mainControl::callback_load_path(const std_msgs::String::ConstPtr& msg)
{
    pFileWayPointLoad = fopen("way_point.txt", "r");
    m_bLoadPathFlag = true;
    nr_data = 0;
}

void mainControl::callback_tracking(const std_msgs::String::ConstPtr& msg)
{
    if(m_bTrackingWPFlag == false){
        m_bTrackingWPFlag = true;
        std::cout << "start tracking..." << std::endl;
    }
    else{
        m_bTrackingWPFlag = false;
        std::cout << "stop tracking..." << std::endl;

        scooter_speed = 0;
        scooter_steering = 0;
    }
    
}

void mainControl::callback_currIndex_init(const std_msgs::String::ConstPtr& msg)
{
    m_curr_index = 0;
}

void mainControl::path_making()
{
    if(m_bMakePathFlag == true){
        if((m_index == 0) || (twoPointDist > way_point_gap)){
            making_path_node[m_index][0] = pos_x;
            making_path_node[m_index][1] = pos_y;
            making_path_node[m_index][2] = pos_heading;

            if(pFileWayPoint != NULL){
                fprintf(pFileWayPoint, "%d %lf %lf %lf\n",
                    m_index, 
                    making_path_node[m_index][0],
                    making_path_node[m_index][1], 
                    making_path_node[m_index][2]);
            }
            m_index++;
        }

        twoPointDist = sqrt(pow((pos_x - making_path_node[m_index-1][0]), 2) + pow((pos_y - making_path_node[m_index-1][1]), 2));
    }
}

void mainControl::path_loading()
{
    if(m_bLoadPathFlag == true){
        if(pFileWayPointLoad != NULL){
            int index_;
            double x_, y_, heading_;
            for(int i=0; ; i++){
                if(fscanf(pFileWayPointLoad, "%d %lf %lf %lf", &index_, &x_, &y_, &heading_) == EOF){
                    break;
                }
                nr_data++;
            }
            m_index = nr_data;
            std::cout << "number of index : " << m_index << std::endl;

            rewind(pFileWayPointLoad);
            path_node = nullptr;
            path_node = new double*[nr_data];
            for(int k=0; k<nr_data; k++){
                path_node[k] = new double[3];
                if(fscanf(pFileWayPointLoad, "%d %lf %lf %lf", &index_, &path_node[k][0], &path_node[k][1], &path_node[k][2]) == EOF){
                    break;
                }
            }
        }

        m_bLoadPathFlag = false;
        fclose(pFileWayPointLoad);
    }
}

void mainControl::getDesiredSpeednSteering(double desired_x, double desired_y, double dt)
{
    ///////////////////////////////////////////////////////////
    // speed
    ///////////////////////////////////////////////////////////

    // km/h -> m/s : /3.6
    // m/s -> km/h : *3.6
    // m_globalspeed : km/h
    // vehicle_speed : m/s

    //float k1 = 1;
    double k2 = 0.8;
    double HZ = 1/dt; // 1/0.01 = 100

    // error of P-controll
    double Vcmd_error = m_globalspeed/3.6 - vehicle_speed;

    std::cout << "vehicle_speed : " << vehicle_speed << std::endl;
    std::cout << "Vcmd_error : " << Vcmd_error << std::endl;

    // Vel accumulated P-control
    if(Vcmd_error/HZ > acceleration/HZ){
        vd += acceleration/HZ;
    }
    else if(Vcmd_error/HZ < -acceleration/HZ){
        vd += -acceleration/HZ;
    }
    // else if (Vcmd_error/HZ < 0){
    //     vd += 0;
    // }
    else{
        vd += Vcmd_error/HZ;
        std::cout << "slow..........." << std::endl;
    }

    if(vd < -0.3 ){
        vd = -0.3;
    }

    //vd += Vcmd_error/HZ;

    std::cout << "vd : " << vd << std::endl;

    // I-controll
    //vd += Vcmd_error;
    //float Vcmd_buf = m_tramSpeed + k1*Vcmd_error + k2*vd;
    double Vcmd_buf = (vehicle_speed + k2*vd)*3.6;

    std::cout << "Vcmd_buf : " << Vcmd_buf << std::endl;

    // Vcmd bound
    if(Vcmd_buf > m_globalspeed ){
        scooter_speed = m_globalspeed;
    }
    else if(Vcmd_buf < 0){
        scooter_speed = 0;
    }
    else{
        scooter_speed = Vcmd_buf;
    }

    std::cout << "After bound cut scooter_speed : " << scooter_speed << std::endl;
    std::cout << "--------------------------" << std::endl;

    ///////////////////////////////////////////////////////////
    // steering
    ///////////////////////////////////////////////////////////
    double dxd = desired_x;
    double dyd = desired_y;

    double alpha;
    alpha = atan2 (dyd, dxd);

    int sign_alpha;
    sign_alpha = (alpha >0)? 1:((alpha <0)? -1: 0);

    double d;
    d = sqrt(dxd*dxd + dyd*dyd);

    double kappa;
    kappa = 2*sin(alpha)/d;
    kappa = fabs(kappa);

    //WB : vehicle wheel base, SR : steering ratio
    //const double WB = 1.15;
    //const float SR = 1;

    //thd = atan(kappa*WB*sign_alpha);
    scooter_steering = atan(kappa*m_wheel_base*sign_alpha);
    scooter_steering = scooter_steering * 180.0/M_PI;
}

void mainControl::tracking()
{
    if(m_bTrackingWPFlag == true)
    {
        if(m_curr_index < m_index -1)
        {
            int tmp_idx = 0;

            if(m_curr_index == 0)
            {
                for(int j=0; j<10; j++){
                    distTemp[j] = sqrt(pow((path_node[j][0] - pos_x), 2) + pow((path_node[j][1] - pos_y), 2) );
                }
                for(int k=0; k<10; k++){
                    min = distTemp[0];
                    if(min > distTemp[k]){
                        min = distTemp[k];
                        tmp_idx = k;
                    }
                }
                // m_globalspeed = 2; // 2m/s
            }
            else
            {
                if((m_curr_index+10) >m_index-1){
                    int gap = m_index-1 - m_curr_index;
                    for(int h =0 ; h<gap; h++){
                        distTemp[h] = sqrt(pow((path_node[m_curr_index+h][0] - pos_x),2)+pow((path_node[m_curr_index+h][1] - pos_y),2));
                    }
                    for(int k = 0; k<gap; k++){
                        min = distTemp[0];
                        if(min > distTemp[k]){
                            min = distTemp[k];
                            tmp_idx = k;
                        }
                    }
                    if(m_curr_index >= (m_index-3)){
                        //m_globalspeed = 0;
                        scooter_speed = 0;
                        return;
                    }
                }
                else{
                    for(int l= 0; l< 10; l++){
                        distTemp[l] = sqrt(pow((path_node[m_curr_index+l][0] - pos_x),2)+pow((path_node[m_curr_index+l][1] - pos_y),2));
                    }
                    for(int k = 0; k<10; k++){
                        min = distTemp[0];
                        if(min > distTemp[k]){
                            min = distTemp[k];
                            tmp_idx = k;
                        }
                    }
                }
            }

            m_curr_index = m_curr_index + tmp_idx; // current vehicle position is tempIndex Way-point
            std::cout << "curr index: " << m_curr_index << std::endl;
            std::cout << "tmp_idx: " << tmp_idx << std::endl;
            
            ///////////////////////////////////////////////////////
            if((m_curr_index + number_Of_Lookahead_Wp) > (m_index-1)){
                controlPointX = path_node[m_index-1][0];
                controlPointY = path_node[m_index-1][1];
            }
            else{
                controlPointX = path_node[m_curr_index+number_Of_Lookahead_Wp][0];
                controlPointY = path_node[m_curr_index+number_Of_Lookahead_Wp][1];
            }

            // coordinate change to body frame 
            desired_x = (controlPointX - pos_x)*cos(pos_heading*M_PI/180.0) + (controlPointY- pos_y)*sin(pos_heading*M_PI/180.0);
            desired_y = -(controlPointX - pos_x)*sin(pos_heading*M_PI/180.0) + (controlPointY - pos_y)*cos(pos_heading*M_PI/180.0);
            
            getDesiredSpeednSteering(desired_x, desired_y, 0.01);
        }
        else
        {
            scooter_speed = 0.0;
        }
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_control");
    ros::NodeHandle nh;

    mainControl main_ctl;

    ros::Subscriber sub_pos_t = nh.subscribe<std_msgs::Float64MultiArray>("/POS_T", 10, &mainControl::callback_localization_POS_T, &main_ctl);
    ros::Subscriber sub_making_path = nh.subscribe<std_msgs::String>("/make_path", 10, &mainControl::callback_make_path, &main_ctl);
    ros::Subscriber sub_load_path = nh.subscribe<std_msgs::String>("/load_path", 10, &mainControl::callback_load_path, &main_ctl);
    ros::Subscriber sub_tracking = nh.subscribe<std_msgs::String>("/tracking", 10, &mainControl::callback_tracking, &main_ctl);
    ros::Publisher pub_scooter_speed = nh.advertise<std_msgs::Float64>("/cmd_scooter_speed", 10);
    ros::Publisher pub_scooter_steering = nh.advertise<std_msgs::Float64>("/cmd_scooter_steering", 10);
    ros::Subscriber sub_currIndex_init = nh.subscribe("/init", 10, &mainControl::callback_currIndex_init, &main_ctl);
    
    std_msgs::Float64 msg_scooter_speed;
    std_msgs::Float64 msg_scooter_steering;

    nh.getParam("/wheel_base", main_ctl.m_wheel_base);
    nh.getParam("/number_Of_Lookahead_Wp", main_ctl.number_Of_Lookahead_Wp);
    nh.getParam("/way_point_gap", main_ctl.way_point_gap);
    nh.getParam("/m_globalspeed", main_ctl.m_globalspeed);
    nh.getParam("/acceleration", main_ctl.acceleration);
    
    ros::Rate loop_rate(100);

    while(ros::ok()){
        ros::spinOnce();

        main_ctl.path_making();
        main_ctl.path_loading();
        main_ctl.tracking();
        msg_scooter_speed.data = main_ctl.scooter_speed;
        msg_scooter_steering.data = main_ctl.scooter_steering;

        pub_scooter_speed.publish(msg_scooter_speed);
        pub_scooter_steering.publish(msg_scooter_steering);

        loop_rate.sleep();
    }

    return 0;

}