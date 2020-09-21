#ifndef _QC_ODOMETRY_MESSAGES_H
#define _QC_ODOMETRY_MESSAGES_H


//导航给路径规划
struct qc_odometry_odometry_laserpoints_message {
	float x;
	float y;
	float z;
    float h;
    float theta; 
	int num_laser_readings;
    //int num_gates;
	float* lx;
	float* ly;
    //float* gx;
    //float* gy;
    float minDisObestacle_angle;  //离飞机最近障碍物的信息，角度和距离，机体坐标系
    float minDisObestacle_dis;
    int num_gates;
    float x1_left;
    float y1_left;
    float x1_right;
    float y1_right;
    float x2_left;
    float y2_left;
    float x2_right;
    float y2_right;
    float x3_left;
    float y3_left;
    float x3_right;
    float y3_right;
    float x4_left;
    float y4_left;
    float x4_right;
    float y4_right;
    float x5_left;
    float y5_left;
    float x5_right;
    float y5_right;
    float windowx_left;
	float windowy_left;
    float windowx_right;
	float windowy_right;
};

#define QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME "qc_odometry_odometry_laserpoints_message"
#define QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_FMT  "{float, float, float, float, float, int,<float:6>,<float:6>,float,float,int,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float}"
#define QC_ODOMETRY_ODOMETRY_MESSAGE_LASERPOINTS_TYPE IPC_VARIABLE_LENGTH


//导航给飞控
struct qc_odometry_flightcontrol_message {
	float x;  // 飞机当前的位置
	float y;
	float z;   //高度
	float pitch;
	float roll;
	float yaw; //偏航角
    float minDisObestacle_angle;  //离飞机最近障碍物的信息，角度和距离，机体坐标系
    float minDisObestacle_dis;
	float vx; //飞机在全局坐标系下的速度
	float vy;
	float vz;
	float avx; ///加速度
	float avy;
	float h;   //飞机实际据地面障碍物的高度
    float gyro_yaw; //偏航角速度
	float time;
};

#define QC_ODOMETRY_FLIGHTCONTROL_MESSAGE_NAME "qc_odometry_flightcontrol_message"
#define QC_ODOMETRY_FLIGHTCONTROL_MESSAGE_FMT "{float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float}"

//***********************************路径规划给飞控***********************//
struct qc_flightpoint_message
 {
	float x;
	float y;
	float z;
	float yaw;
	int Hover;
    int Launch;
	int Land;
	
	
};

#define QC_FLIGHTPOINT_MESSAGE_NAME "qc_flightpoint_message" 
#define QC_FLIGHTPOINT_MESSAGE_FMT "{float,float,float,float,int,int,int}"


#endif // _QC_ODOMETRY_MESSAGES_H
