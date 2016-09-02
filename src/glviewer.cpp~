
#include <math.h>
#include <GL/glut.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <FTGL/ftgl.h>
#include <signal.h>

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "include/transformation3.hh"
#include "include/transformation2.hh"
#include "include/qc_odometry_messages.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <px_comm/OpticalFlow.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#define NAV_USING_GROUND_TILE          33
#define NAV_USING_SLAM                 34
#define NAV_USING_OPTICAL_FLOW         35


using namespace std;

struct timeval tvstart;
struct timeval tvend;

double roll, yaw, pitch;

double gametime=0;
float Robot_direction[10], Robot_position_x[10], Robot_position_y[10];
float vehicle_position_x=0.0,vehicle_position_y=0.0,vehicle_position_z=0.0,vehicle_direction=0.0,vehicle_wingangle=0.0;
float goal_position_x=0.0,goal_position_y=0.0,goal_position_z=0.0;
float position_record_x[20000], position_record_y[20000], position_record_z[20000];
int position_record_state[20000];
float goal_record_x[20000], goal_record_y[20000], goal_record_z[20000];
int goal_record_state[20000];
float obstacle_position_x[4],obstacle_position_y[4],obstacle_direction[4];
float angle=-1.5775,deltaAngle = 0.0,ratio;
float convertMat[3][3];
float x=11.0f,y=0.5f,z=0.0f;
float lx_FrontBack=-1.0f,ly=-70.0f,lx=70.0f,lz=70.0f,lz_FrontBack=0.0f,lx_LeftRight=0.0f,lz_LeftRight=-1.0f,lx_UpDown=0.0f,ly_UpDown=0.0f,lz_UpDown=-1.0f;
GLint snowman_display_list;
int deltaMove_FrontBack = 0, deltaMove_LeftRight = 0, deltaMove_UpDown=0;
bool overlook=false,fu=false,yang=false,Robot_move=false,vehicle_move=false,vehicle_land=false,vehicle_collision=false,track_clear=false;
int window_width=640, window_height=360;
int move_record=0;
int goal_move_record=0;

px_comm::OpticalFlow local_optical_data;

nav_msgs::Odometry x_sub_CurrPose;

geometry_msgs::PoseStamped x_sub_GoalPose;

void x_position_Callback(const nav_msgs::Odometry& CurrPose)
{
  x_sub_CurrPose = CurrPose;
  
  local_optical_data.state_style = NAV_USING_GROUND_TILE;
  
  vehicle_position_x = 10.0f - x_sub_CurrPose.pose.pose.position.x;
  vehicle_position_y = x_sub_CurrPose.pose.pose.position.z;
  vehicle_position_z = x_sub_CurrPose.pose.pose.position.y;

  if(move_record % 2 == 0){
    position_record_x[move_record/2] = vehicle_position_x;
    position_record_y[move_record/2] = vehicle_position_y;
    position_record_z[move_record/2] = vehicle_position_z;
    position_record_state[move_record/2] = local_optical_data.state_style;
  }
  if(move_record++/2 >= 20000)
    move_record = 0;

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(x_sub_CurrPose.pose.pose.orientation, orientation);
  tfScalar curr_yaw, curr_pitch, curr_roll;
  tf::Matrix3x3(orientation).getEulerYPR(curr_yaw, curr_pitch, curr_roll);

  local_optical_data.pitch = curr_pitch;
  local_optical_data.yaw = curr_yaw;
  local_optical_data.roll  = curr_roll;
  pitch = local_optical_data.pitch;
  yaw = -local_optical_data.yaw;
  roll = -local_optical_data.roll;

	convertMat[0][0]=cos(pitch)*cos(yaw);
	convertMat[1][0]=cos(pitch)*sin(yaw);
	convertMat[2][0]=-sin(pitch);
	convertMat[0][1]=sin(pitch)*cos(yaw)*sin(roll)-sin(yaw)*cos(roll);
	convertMat[1][1]=sin(pitch)*sin(yaw)*sin(roll)+cos(yaw)*cos(roll);
	convertMat[2][1]=cos(pitch)*sin(roll);
	convertMat[0][2]=sin(pitch)*cos(yaw)*cos(roll)+sin(yaw)*sin(roll);
	convertMat[1][2]=sin(pitch)*sin(yaw)*cos(roll)-cos(yaw)*sin(roll);
	convertMat[2][2]=cos(pitch)*cos(roll);
}

void x_goal_Callback(const geometry_msgs::PoseStamped& CurrPose)
{
  x_sub_GoalPose = CurrPose;
  
  local_optical_data.state_style = NAV_USING_GROUND_TILE;
  
  goal_position_x = 10.0f - x_sub_GoalPose.pose.position.x;
  goal_position_y = x_sub_GoalPose.pose.position.z;
  goal_position_z = x_sub_GoalPose.pose.position.y;

  if(goal_move_record % 5 == 0){
    goal_record_x[goal_move_record/2] = goal_position_x;
    goal_record_y[goal_move_record/2] = goal_position_y;
    goal_record_z[goal_move_record/2] = goal_position_z;
    goal_record_state[goal_move_record/2] = NAV_USING_OPTICAL_FLOW;
  }
  if(goal_move_record++/5 >= 20000)
    goal_move_record = 0;
}

const float R=0.03;

void change_direction(int id){
	for(int i=0;i<10;i++)
		Robot_direction[i]=Robot_direction[i]>0? (Robot_direction[i]-3.141): (Robot_direction[i]+3.141);
	glutTimerFunc(20000,change_direction,0);
}

void error_direction(int id){
	srand((unsigned)time(0));
	for(int i=0;i<10;i++)
		Robot_direction[i]+=(rand()/(double)(RAND_MAX/(2*3.141/9.0))-3.141/9.0);
	glutTimerFunc(5000,error_direction,0);
}

void changeSize(int w, int h){
	window_width=w;
	window_height=h;
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if(h == 0)
		h = 1;
	ratio = 1.0f * w / h;
	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);
	// Set the clipping volume
	gluPerspective(45,ratio,1,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
		      x,y,z,
			  0.0f,1.0f,0.0f);
}

GLuint createDL() {
	GLuint snowManDL;
	// Create the id for the list
	snowManDL = glGenLists(1);
	// start list
	glNewList(snowManDL,GL_COMPILE);
	// call the function that contains the rendering commands
	// endList
	glEndList();
	return(snowManDL);
}

void initScene() {
	glEnable(GL_DEPTH_TEST);
	snowman_display_list = createDL();
}

void RenderBone(float x0, float y0, float z0, float x1, float y1, float z1 ,GLdouble radius)
{
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );
	glPushMatrix();
	// ÆœÒÆµœÆðÊŒµã
	glTranslated( x0, y0, z0 );
	// ŒÆËã³€¶È
	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;
	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;
	// ŒÆËã±ä»»ŸØÕó
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
		up_x,   up_y,   up_z,   0.0,
		dir_x,  dir_y,  dir_z,  0.0,
		0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );
	// Ô²ÖùÌå²ÎÊý
	//GLdouble radius= 2.0;		// °ëŸ¶
	GLdouble slices = 30.0;		//	¶ÎÊý
	GLdouble stack = 6.0;		// µÝ¹éŽÎÊý
	gluCylinder( quad_obj, radius, radius, bone_length, slices, stack ); 
	glPopMatrix();
}

void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M, GLfloat N)
{
 float step_z = M_PI/M;
 float step_xy = 2*M_PI/N;
 float x[4],y[4],z[4];

 float angle_z = 0.0;
 float angle_xy = 0.0;
 int i=0, j=0;
 glBegin(GL_QUADS);
  for(i=0; i<M; i++)
  {
   angle_z = i * step_z;
   
   for(j=0; j<N; j++)
   {
    angle_xy = j * step_xy;

    x[0] = radius * sin(angle_z) * cos(angle_xy);
    y[0] = radius * sin(angle_z) * sin(angle_xy);
    z[0] = radius * cos(angle_z);

    x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);
    y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
    z[1] = radius * cos(angle_z + step_z);

    x[2] = radius*sin(angle_z + step_z)*cos(angle_xy + step_xy);
    y[2] = radius*sin(angle_z + step_z)*sin(angle_xy + step_xy);
    z[2] = radius*cos(angle_z + step_z);

    x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
    y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
    z[3] = radius * cos(angle_z);

    for(int k=0; k<4; k++)
    {
     glVertex3f(xx+x[k], yy+y[k],zz+z[k]);
    }
   }
  }
 glEnd();
}

void orientMe(float ang) {
	lx_FrontBack = sin(ang);
	lz_FrontBack = -cos(ang);
	lx_LeftRight = -cos(ang);
	lz_LeftRight = -sin(ang);
	glLoadIdentity();
	gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
		      x,y,z,
			  0.0f,1.0f,0.0f);
}

void moveMeFlat_LeftRight(int i) {
	lx_LeftRight = -cos(angle);
	lz_LeftRight = -sin(angle);
	x = x + i*(lx_LeftRight)*0.03;
	z = z + i*(lz_LeftRight)*0.03;
	glLoadIdentity();
	gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
		      x,y,z,
			  0.0f,1.0f,0.0f);
}

void moveMeFlat_FrontBack(int i) {
	lx_FrontBack = sin(angle);
	lz_FrontBack = -cos(angle);
	x = x + i*(lx_FrontBack)*0.03;
	z = z + i*(lz_FrontBack)*0.03;
	glLoadIdentity();
	gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
		      x,y,z,
			  0.0f,1.0f,0.0f);
}

void moveMeFlat_UpDown(int i) {
	lx_FrontBack = sin(angle);
	lz_FrontBack = -cos(angle);
	lx_LeftRight = -cos(angle);
	lz_LeftRight = -sin(angle);
	ly_UpDown = 1.0f;
	if(i>0)// && y<3.0f)
		y = y + i*(ly_UpDown)*0.03;
	if(i<0)// && y>0.1f)
		y = y + i*(ly_UpDown)*0.03;
	if(y>0.2)
		vehicle_land=false;
	glLoadIdentity();
	gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
		      x,y,z,
			  0.0f,1.0f,0.0f);
}

void moveMeFlat_overlook(void) {
	glLoadIdentity();
	gluLookAt(0.0f, 28.0f, 0.0f, 
		      0.0f ,28.0f -0.1f,0.0f ,
			  0.0f,0.0f,-1.0f);
}

void moveMeFlat_fuyang(void) {

if(lx>1)
	if(fu){
		glLoadIdentity();
		gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx--), y - 1*sin(0.01*ly--), z - 1*lz_FrontBack*sin(0.01*lz--), 
		  	     x,y,z,
			  	0.0f,cos(0.01*ly),0.0f);
	}
if(lx<290)
	if(yang){
		glLoadIdentity();
		gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx++), y - 1*sin(0.01*ly++), z - 1*lz_FrontBack*sin(0.01*lz++), 
		  	     x,y,z,
			  	0.0f,cos(0.01*ly),0.0f);
	}
}

void clear_track(){
	if(track_clear == true){
		for(int i=0;i<20000;i++){
			position_record_x[i] = 0;
			position_record_y[i] = 0;
			position_record_z[i] = 0;

			goal_record_x[i] = 0;
			goal_record_y[i] = 0;
			goal_record_z[i] = 0;
		}
	}
}

Eigen::Vector3d vehicle2Global(float x,float y,float z)
{
	Eigen::Vector3d vector;
	vector(0)=convertMat[0][0]*x+convertMat[0][1]*y+convertMat[0][2]*z;
	vector(1)=convertMat[1][0]*x+convertMat[1][1]*y+convertMat[1][2]*z;
	vector(2)=convertMat[2][0]*x+convertMat[2][1]*y+convertMat[2][2]*z;
	return vector;
}

void renderScene(void) {
//Calculate time
	gettimeofday(&tvend,NULL);
	double avgtime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
	if(Robot_move==true)
		gametime+=avgtime;
	gettimeofday(&tvstart,NULL);

	if(fu || yang)
		moveMeFlat_fuyang();
	if(overlook)
		moveMeFlat_overlook();
	if (deltaMove_UpDown)
		moveMeFlat_UpDown(deltaMove_UpDown);
	if (deltaMove_FrontBack)
		moveMeFlat_FrontBack(deltaMove_FrontBack);
	if (deltaMove_LeftRight)
		moveMeFlat_LeftRight(deltaMove_LeftRight);
	if (deltaAngle) {
		angle += deltaAngle;
		orientMe(angle);
	}
	if(track_clear)
		clear_track();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// Draw ground
	glColor3f(0.2f, 0.4f, 0.6f);
	glLineWidth(1);
	glBegin(GL_LINES);
		for(int i = 26; i >-40 ; i--)
			for(int j=-30; j < 30; j++) {
				glVertex3f(j, 0.0f, i);
				glVertex3f(j+1.0f, 0.0f, i);
				glVertex3f(j, 0.0f, i);
				glVertex3f(j, 0.0f, i-1.0f);
			}
	glEnd();
//Line Red Line
	glColor3f(0.9f, 0.1f, 0.1f);
	glLineWidth(4);
	glBegin(GL_LINES);
		glVertex3f(-10.0f, 0.0f, 10.0f);
		glVertex3f(10.0f, 0.0f, 10.0f);
	glEnd();
//Line Green Line
	glColor3f(0.1f, 0.9f, 0.1f);
	glLineWidth(4);
	glBegin(GL_LINES);
		glVertex3f(-10.0f, 0.0f, -10.0f);
		glVertex3f(10.0f, 0.0f, -10.0f);
	glEnd();
//Line White Line
	glColor3f(0.9f, 0.9f, 0.9f);
	glLineWidth(4);
	glBegin(GL_LINES);
		glVertex3f(-10.0f, 0.0f, 10.0f);
		glVertex3f(10.0f, 0.0f, 10.0f);
		glVertex3f(-10.0f, 0.0f, -10.0f);
		glVertex3f(-10.0f, 0.0f, 10.0f);
		glVertex3f(10.0f, 0.0f, -10.0f);
		glVertex3f(10.0f, 0.0f, 10.0f);
		glVertex3f(10.0f, 0.0f, 0.0f);
		glVertex3f(-10.0f, 0.0f, 0.0f);
	glEnd();
//Start Circle
//	RenderBone(0,0,0,0,0.05,0,1.5);
//iRobot
//Vehicle Body
	glColor3f(0.8f, 0.95f, 0.8f);
	for(int i=0;i<4;i++)
	{
		Eigen::Vector3d v=vehicle2Global(0.05303*cos(0.7854+i*1.5708),0.05303*sin(0.7854+i*1.5708),-0.0125);
		Eigen::Vector3d vv=vehicle2Global(0.05303*cos(0.7854+i*1.5708),0.05303*sin(0.7854+i*1.5708),-0.01375);
		RenderBone(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)),  vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)), R);
	}
	glColor3f(0.1f, 0.95f, 0.1f);
	glLineWidth(2);
	glBegin(GL_LINE_LOOP);
		for(int i=0;i<4;i++)
		{
			Eigen::Vector3d v=vehicle2Global(0.10783*cos(0.7854+i*1.5708),0.10783*sin(0.7854+i*1.5708),-0.01625);
			Eigen::Vector3d vv=vehicle2Global(0.10783*cos(0.7854+(i+1)*1.5708),0.10783*sin(0.7854+(i+1)*1.5708),-0.01625);
			
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
			glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));
		}
	glEnd();
	//Wings
	glColor3f(0.8f, 0.95f, 0.6f);
	glBegin(GL_LINES);
		for(int i=0;i<4;i++)
		{
			Eigen::Vector3d v=vehicle2Global(0.05303*cos(0.7854+i*1.5708)+R*cos(6.2832/120*vehicle_wingangle++),0.05303*sin(0.7854+i*1.5708)+R*sin(6.2832/120*vehicle_wingangle++),-0.01625);
			Eigen::Vector3d vv=vehicle2Global(0.05303*cos(0.7854+i*1.5708)-R*cos(6.2832/120*vehicle_wingangle++),0.05303*sin(0.7854+i*1.5708)-R*sin(6.2832/120*vehicle_wingangle++),-0.01625);
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2),(vehicle_position_z+v(1)));
                        glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2),(vehicle_position_z+vv(1)));	
		}
	glEnd();
	//Body
	glColor3f(0.8f, 0.6f, 0.8f);
		for(int i=0;i<4;i++)
		{
			Eigen::Vector3d v=vehicle2Global(0.02651*cos(0.7854+i*1.5708),0.02651*sin(0.7854+i*1.5708),0);
			Eigen::Vector3d vv=vehicle2Global(0.02651*cos(0.7854+i*1.5708),0.02651*sin(0.7854+i*1.5708),-0.03125);
			RenderBone(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)), vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)),0.01);
		}
	//Top
	glColor3f(0.5f, 0.0f, 0.2f);
	glBegin(GL_POLYGON);
	      for(int i=0;i<4;i++)
	      {
	      	Eigen::Vector3d v=vehicle2Global(0.02651*cos(0.7854+i*1.5708),0.02651*sin(0.7854+i*1.5708),-0.03125);
	      	glVertex3f( vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
	      }  
	glEnd();
	//Leg
	glColor3f(0.8f, 0.95f, 0.6f);
	glBegin(GL_LINES);
		for(int i=0;i<4;i++)
		{
			Eigen::Vector3d v=vehicle2Global(0.02651*cos(0.7854+i*1.5708),0.02651*sin(0.7854+i*1.5708),0);
			Eigen::Vector3d vv=vehicle2Global(0.03276*cos(0.7854+i*1.5708),0.03276*sin(0.7854+i*1.5708),0);
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
			glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));
		}
	glEnd();
	//Leg
	glColor3f(0.8f, 0.95f, 0.6f);
	glBegin(GL_LINES);
		for(int i=0;i<4;i++)
		{
			Eigen::Vector3d v=vehicle2Global(0.05303*cos(0.7854+i*1.5708),0.05303*sin(0.7854+i*1.5708),-0.01625);
			Eigen::Vector3d vv=vehicle2Global(0.10783*cos(0.7854+i*1.5708),0.10783*sin(0.7854+i*1.5708),-0.01625);
			
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
			glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));			
			
			vv=vehicle2Global(0.08497*cos(0.45707+i*1.5708),0.08497*sin(0.45707+i*1.5708),-0.01625);
			
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
			glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));
			
			vv=vehicle2Global(0.08497*cos(1.11373+i*1.5708),0.08497*sin(1.11373+i*1.5708),-0.01625);
			
			glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
			glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));
		}
	glEnd();
	//Shadow
	glColor3f(0.5f, 0.0f, 0.2f);
	glLineWidth(2);
	glBegin(GL_LINES);
		glVertex3f(vehicle_position_x,vehicle_position_y,vehicle_position_z);glVertex3f(vehicle_position_x,0.0f,vehicle_position_z);
	glEnd();


	//direction
	glColor3f(0.8f, 0.8f, 0.2f);
	glLineWidth(4);
	glBegin(GL_LINES);
		Eigen::Vector3d v=vehicle2Global(0,0,-0.0325);
		Eigen::Vector3d vv=vehicle2Global(0.01875,0,-0.0325);
		
		glVertex3f(vehicle_position_x-v(0), vehicle_position_y-v(2), (vehicle_position_z+v(1)));
		glVertex3f(vehicle_position_x-vv(0), vehicle_position_y-vv(2), (vehicle_position_z+vv(1)));
	glEnd();

//Vehicle trace
	glColor3f(1.0f, 1.0f, 0.0f);
	glLineWidth(1);
	glBegin(GL_LINES);
		for(int i = 0; i < 19999 ; i++){
			if(position_record_x[i] && position_record_y[i] && position_record_z[i] && position_record_y[i+1] && position_record_z[i+1] && position_record_state[i] == NAV_USING_GROUND_TILE){
				glVertex3f(position_record_x[i], position_record_y[i], position_record_z[i]);
				glVertex3f(position_record_x[i+1], position_record_y[i+1], position_record_z[i+1]);
			}
		}
	glEnd();
//Goal trace
	glColor3f(0.0f, 1.0f, 1.0f);
	glLineWidth(1);
	glBegin(GL_LINES);
		for(int i = 0; i < 19999 ; i++){
			if(goal_record_x[i] && goal_record_y[i] && goal_record_z[i] && goal_record_y[i+1] && goal_record_z[i+1] && goal_record_state[i] == NAV_USING_OPTICAL_FLOW){
				glVertex3f(goal_record_x[i], goal_record_y[i], goal_record_z[i]);
				glVertex3f(goal_record_x[i+1], goal_record_y[i+1], goal_record_z[i+1]);
			}
		}
	glEnd();
	glColor3f(0.0f, 1.0f, 1.0f);
	glLineWidth(1);
	glBegin(GL_LINES);
		for(int i = 0; i < 19999 ; i++){
			if(position_record_x[i] && position_record_y[i] && position_record_z[i] && position_record_y[i+1] && position_record_z[i+1] && position_record_state[i] == NAV_USING_OPTICAL_FLOW){
				glVertex3f(position_record_x[i], position_record_y[i], position_record_z[i]);
				glVertex3f(position_record_x[i+1], position_record_y[i+1], position_record_z[i+1]);
			}
		}
	glEnd();
	glColor3f(1.0f, 0.0f, 1.0f);
	glLineWidth(1);
	glBegin(GL_LINES);
		for(int i = 0; i < 19999 ; i++){
			if(position_record_x[i] && position_record_y[i] && position_record_z[i] && position_record_y[i+1] && position_record_z[i+1] && position_record_state[i] == NAV_USING_SLAM){
				glVertex3f(position_record_x[i], position_record_y[i], position_record_z[i]);
				glVertex3f(position_record_x[i+1], position_record_y[i+1], position_record_z[i+1]);
			}
		}
	glEnd();

	//Goal Point
	glColor3f(1.0f, 0.2f, 0.2f);
		drawSphere(goal_position_x, goal_position_y, goal_position_z, 0.03, 20, 20);
//Text Score
	float score=0;
	for(int i=0;i<10;i++){
		if(Robot_position_y[i]<=-10.0f)
			score+=2000;
		else if(Robot_position_x[i]<=-10.0f || Robot_position_y[i]>=10.0f || Robot_position_x[i]>=10.0f)
			score+=-1000;
	}
	if(vehicle_collision==true)
		score=0;
	FTGLPixmapFont font("/home/darc/catkin_ws/src/glviewer/font/arial.ttf");
	FTPoint position_label_x(window_width-260.0f,window_height-40.0f,0.0f);
	font.FaceSize(13);
	char tempchar[10];
	sprintf(tempchar,"%f",score);
	const char *label_x=tempchar;
	font.Render(label_x,8,position_label_x);

	FTPoint position_text_x(window_width-330.0f,window_height-40.0f,0.0f);
	if(vehicle_collision==true){
		const char *positiontext_x="Failed:";
		font.Render(positiontext_x,7,position_text_x);
	}else{
		const char *positiontext_x="Score:";
		font.Render(positiontext_x,6,position_text_x);
	}
//Text time
	FTPoint position_label_time(window_width-260.0f,window_height-55.0f,0.0f);
	font.FaceSize(13);
	char timechar[10];
	sprintf(timechar,"%f",avgtime*1000);
	const char *label_time=timechar;
	font.Render(label_time,8,position_label_time);

	FTPoint position_text_time(window_width-330.0f,window_height-55.0f,0.0f);
	const char *timetext_x="Time:";
	font.Render(timetext_x,5,position_text_time);
//Text Position
	FTPoint position_label_position_x(window_width-260.0f,window_height-70.0f,0.0f);
	font.FaceSize(13);
	char positionchar[10];
	sprintf(positionchar,"%f",vehicle_position_x-10.0f);
	const char *label_position=positionchar;
	font.Render(label_position,8,position_label_position_x);

	FTPoint position_label_position_y(window_width-180.0f,window_height-70.0f,0.0f);
	font.FaceSize(13);
	sprintf(positionchar,"%f",vehicle_position_z);
	const char *label_position_y=positionchar;
	font.Render(label_position_y,8,position_label_position_y);

	FTPoint position_text_position(window_width-360.0f,window_height-70.0f,0.0f);
	const char *positiontext_x="Vehicle position:";
	font.Render(positiontext_x,17,position_text_position);
//Text Height
	FTPoint position_label_height(window_width-260.0f,window_height-85.0f,0.0f);
	font.FaceSize(13);
	char heightchar[10];
	sprintf(heightchar,"%f",vehicle_position_y);
	const char *label_height=heightchar;
	font.Render(label_height,8,position_label_height);

	FTPoint position_text_height(window_width-310.0f,window_height-85.0f,0.0f);
	const char *heighttext_x="Height:";
	font.Render(heighttext_x,7,position_text_height);

	glutSwapBuffers();

	ros::spinOnce();

}

void pressKey(unsigned char key, int x, int y) {

	switch (key) {
		case 'f' : fu=true;break;
		case 'r' : yang=true;break;
		case 'o' : overlook=true;break;
		case 'u' : deltaMove_UpDown = 1;break;
		case 'j' : deltaMove_UpDown = -1;break;
		case 'q' : deltaAngle = -0.01f;break;
		case 'e' : deltaAngle = 0.01f;break;
		case 'a' : deltaMove_LeftRight = 1;break;//deltaAngle = -0.01f;break;
		case 'd' : deltaMove_LeftRight = -1;break;//deltaAngle = 0.01f;break;
		case 'w' : deltaMove_FrontBack = 1;break;
		case 's' : deltaMove_FrontBack = -1;break;
		case 'c' : track_clear = true;break;
		case '1' : Robot_move = true;break;
		case '2' : Robot_move = false;break;
	}
}

void releaseKey(unsigned char key, int x, int y) {

	switch (key) {
		case 'f' : fu=false;break;
		case 'r' : yang=false;break;
		case 'o' : overlook=false;break;
		case 'u' : 
		case 'j' : deltaMove_UpDown = 0;break;
		case 'q' : 
		case 'e' : deltaAngle = 0.0f;break;
		case 'a' : 
		case 'd' : deltaMove_LeftRight = 0;break;//deltaAngle = 0.0f;break;
		case 'w' : 
		case 's' : deltaMove_FrontBack = 0;break;
		case 'c' : track_clear = false;break;		
		case '1' : Robot_move = true;break;
		case '2' : Robot_move = false;break;
	}
}

void processNormalKeys(unsigned char key, int x, int y) {

	if (key == 27) 
		exit(0);
}

void SetLight(void){
	const GLfloat AmbientLight[4]={1,1,1,-1};
	const GLfloat LightPosition[4]={10,10,10,0};
	glLightfv(GL_LIGHT0,GL_AMBIENT,AmbientLight);
	glLightfv(GL_LIGHT0,GL_POSITION,LightPosition);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
}



int main(int argc, char **argv)
{
	gettimeofday(&tvstart,NULL);
//initial iRobot
	srand((unsigned)time(0));
	for(int i=0;i<10;i++){
		Robot_position_x[i]=rand()/(double)(RAND_MAX/2.0)-1.0;
		Robot_position_y[i]=rand()/(double)(RAND_MAX/2.0)-1.0;
		Robot_direction[i]=rand()/(double)(RAND_MAX/6.282)-3.141;
	}
	int j=0,mark=0,count=0;	

	pitch = 0;
	yaw = 0;
	roll = 0;
	convertMat[0][0]=1;
	convertMat[1][0]=0;
	convertMat[2][0]=0;
	convertMat[0][1]=0;
	convertMat[1][1]=1;
	convertMat[2][1]=0;
	convertMat[0][2]=0;
	convertMat[1][2]=0;
	convertMat[2][2]=1;

	bool goon=true;
	while(goon){
		for(int i=0;i<10;i++){
			Robot_position_x[i]=rand()/(double)(RAND_MAX/2.0)-1.0;
			Robot_position_y[i]=rand()/(double)(RAND_MAX/2.0)-1.0;
		}
		for(int i=0;i<10;i++)
			for(j=i+1;j<10;j++){
				count+=1;
				if(sqrt(pow(Robot_position_x[i]-Robot_position_x[j],2)+pow(Robot_position_y[i]-Robot_position_y[j],2))>0.35)
					mark+=1;
			}
		if(mark==count) goon=false;
		mark=0;count=0;
	}

//initial ROS
        ros::init(argc, argv, "glviewer");

	ros::NodeHandle x_handle;

	ros::Subscriber sub_position = x_handle.subscribe("/crazyflie/pose", 1000, x_position_Callback);
	ros::Subscriber sub_goal = x_handle.subscribe("/crazyflie/goal", 1000, x_goal_Callback);

//initial vehicle
	vehicle_position_x=10.0;vehicle_position_y=0.0;vehicle_direction=rand()/(double)(RAND_MAX/6.282)-3.141;
//initial obstacle
	obstacle_position_x[0]=5; obstacle_position_x[1]=0; obstacle_position_x[2]=-5; obstacle_position_x[3]=0;
	obstacle_position_y[0]=0; obstacle_position_y[1]=5; obstacle_position_y[2]=0; obstacle_position_y[3]=-5;
	obstacle_direction[0]=3.141;obstacle_direction[1]=3.141/2;obstacle_direction[2]=0;obstacle_direction[3]=3.141/2*3;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(640,360);
	glutCreateWindow("DARCFlie Realtime Visualization");

	initScene();

	glutIgnoreKeyRepeat(1);
	glutKeyboardFunc(pressKey);
	glutKeyboardUpFunc(releaseKey);

	glutDisplayFunc(renderScene);
	glutIdleFunc(renderScene);
	
	glutReshapeFunc(changeSize);
	glutTimerFunc(20000,change_direction,0);
	glutTimerFunc(5000,error_direction,0);
	glutMainLoop();

	return(0);
}
