
#include <math.h>
#include <GL/glut.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <FTGL/ftgl.h>
#include <signal.h>

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "nav_msgs/Odometry.h"

#include "utility/LPfilter.h"

#include "glcolor.h"

#define NAV_USING_GROUND_TILE          33
#define NAV_USING_SLAM                 34
#define NAV_USING_OPTICAL_FLOW         35
#define VEHICLE_SCALE(_scale)  (vehicle_scale * _scale)

typedef enum VEHICLE_TYPE_ENUM {
  VEHICLE_BALL,
  VEHICLE_QUAD,
  VEHICLE_BI_UP,
  VEHICLE_BI_DOWN,
} VEHICLE_TYPE;




using namespace std;
using namespace Eigen;

struct timeval tvstart;
struct timeval tvend;

VEHICLE_TYPE vehicle_type = VEHICLE_QUAD;
VEHICLE_TYPE vehicle_goal_type = VEHICLE_BALL;

// Vehicle property in meters
const float vehicle_scale = 1.0f;
const float vehicle_propeller_radius = VEHICLE_SCALE(0.03);
const float vehicle_bi_arm_length = VEHICLE_SCALE(0.12);
const float vehicle_bi_servo_length = VEHICLE_SCALE(0.04);
const float vehicle_bi_motoro_length = VEHICLE_SCALE(0.02);

string FONT_FILE_PATH =  "/home/USERNAME/catkin_ws/src/glviewer/font/arial.ttf";

double gametime=0;
float vehicle_position_x=0.0,vehicle_position_y=0.0,vehicle_position_z=0.0,vehicle_direction=0.0,vehicle_wingangle=0.0;
float goal_position_x=0.0,goal_position_y=0.0,goal_position_z=0.0;
float position_record_x[20000], position_record_y[20000], position_record_z[20000];
int position_record_state[20000];
float goal_record_x[20000], goal_record_y[20000], goal_record_z[20000];
int goal_record_state[20000];
float angle=-1.5775,deltaAngle = 0.0,fuyang=0;
float x=11.0f,y=0.5f,z=0.0f;
float lx_FrontBack=-1.0f,ly=-70.0f,lx=70.0f,lz=70.0f,lz_FrontBack=0.0f,lx_LeftRight=0.0f,lz_LeftRight=-1.0f,lx_UpDown=0.0f,ly_UpDown=0.0f,lz_UpDown=-1.0f;
GLint snowman_display_list;
int deltaMove_FrontBack = 0, deltaMove_LeftRight = 0, deltaMove_UpDown=0;
bool overlook=false,Robot_move=false,vehicle_move=false,vehicle_land=false,vehicle_collision=false,track_clear=false;
int window_width=640, window_height=360;
int move_record=0;
int goal_move_record=0;
LPfilter fuyangFilter(20, 0.01), deltaAngleFilter(20, 0.01);

/***** Rotation matrix ******
 B : body frame
 W : world frame
 LS: left servo frame
 RS: right servo frame
 GO: goal frame
***************************/
Quaterniond R_BW(1,0,0,0), R_GOW(1,0,0,0), R_LSB, R_RSB, R_WGL(0,0,1,0);
Vector3d COG, COG_GOAL;

nav_msgs::Odometry x_sub_CurrPose;

geometry_msgs::PoseStamped x_sub_GoalPose;
geometry_msgs::Quaternion vehicle_bi_left_angle_msg, vehicle_bi_right_angle_msg;

void x_position_Callback(const nav_msgs::Odometry& CurrPose)
{
  x_sub_CurrPose = CurrPose;
  
  vehicle_position_x = 10.0f - x_sub_CurrPose.pose.pose.position.x;
  vehicle_position_y = x_sub_CurrPose.pose.pose.position.z;
  vehicle_position_z = -x_sub_CurrPose.pose.pose.position.y;

  COG << vehicle_position_x, vehicle_position_y, vehicle_position_z;

  if(move_record % 2 == 0){
    position_record_x[move_record/2] = vehicle_position_x;
    position_record_y[move_record/2] = vehicle_position_y;
    position_record_z[move_record/2] = vehicle_position_z;
    position_record_state[move_record/2] = NAV_USING_GROUND_TILE;
  }
  if(move_record++/2 >= 20000)
    move_record = 0;

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(x_sub_CurrPose.pose.pose.orientation, orientation);
  tf::quaternionTFToEigen(orientation, R_BW);
  // from convert to openGL frame
  R_BW = R_WGL * R_BW * R_WGL;
}

void x_goal_Callback(const geometry_msgs::PoseStamped& CurrPose)
{
  x_sub_GoalPose = CurrPose;
  
  goal_position_x = 10.0f - x_sub_GoalPose.pose.position.x;
  goal_position_y = x_sub_GoalPose.pose.position.z;
  goal_position_z = -x_sub_GoalPose.pose.position.y;

  COG_GOAL << goal_position_x, goal_position_y, goal_position_z;

  if(goal_move_record % 5 == 0){
    goal_record_x[goal_move_record/2] = goal_position_x;
    goal_record_y[goal_move_record/2] = goal_position_y;
    goal_record_z[goal_move_record/2] = goal_position_z;
    goal_record_state[goal_move_record/2] = NAV_USING_OPTICAL_FLOW;
  }
  if(goal_move_record++/5 >= 20000)
    goal_move_record = 0;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(x_sub_GoalPose.pose.orientation, orientation);
  tf::quaternionTFToEigen(orientation, R_GOW);
  // from convert to openGL frame
  R_GOW = R_WGL * R_GOW * R_WGL;
}

void left_angle_Callback(const geometry_msgs::Quaternion& msg)
{
  vehicle_bi_left_angle_msg = msg;
  tf::quaternionMsgToEigen(vehicle_bi_left_angle_msg, R_LSB);
  //R_LSB = R_LSB.conjugate();
}

void right_angle_Callback(const geometry_msgs::Quaternion& msg)
{
  vehicle_bi_right_angle_msg = msg;
  tf::quaternionMsgToEigen(vehicle_bi_right_angle_msg, R_RSB);
  //R_RSB = R_RSB.conjugate();
}

double saturate(double input, double min, double max){
  if(input < min)
    return min;
  if(input > max)
    return max;
  return input;
}

Eigen::Vector3d local2Global(float x,float y,float z, Quaterniond R_LW)
{
  Eigen::Vector3d vector;
  Vector3d from(x,y,z);
  vector = R_LW*from;
  return vector;
}

void changeSize(int w, int h){
  window_width=w;
  window_height=h;
  // Prevent a divide by zero, when window is too short
  // (you cant make a window of zero width).
  if(h == 0)
    h = 1;
  float ratio = 1.0f * w / h;
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

void drawCylinder(float x0, float y0, float z0, float x1, float y1, float z1 ,GLdouble radius)
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

  GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
		      up_x,   up_y,   up_z,   0.0,
		      dir_x,  dir_y,  dir_z,  0.0,
		      0.0,    0.0,    0.0,    1.0 };
  glMultMatrixd( m );
  //GLdouble radius= 2.0;		
  GLdouble slices = 30.0;   
  GLdouble stack = 6.0;	    
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

void drawRectangle3D(Vector3d cog, Vector3d width, Quaterniond quat){
  Quaterniond Rtemp(0,0,0.707,0.707);
  quat = Rtemp * quat * Rtemp;
  Vector3d R_vet;
  for(int i = 0; i < 2; i++){
    glBegin(GL_POLYGON);
    R_vet = quat * Vector3d(pow(-1,i)*width.x()/2, width.z()/2, width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(pow(-1,i)*width.x()/2, -width.z()/2, width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(pow(-1,i)*width.x()/2, -width.z()/2, -width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(pow(-1,i)*width.x()/2, width.z()/2, -width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    glEnd();
  }
  for(int i = 0; i < 2; i++){
    glBegin(GL_POLYGON);
    R_vet = quat * Vector3d(width.x()/2, pow(-1,i)*width.z()/2, width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(-width.x()/2, pow(-1,i)*width.z()/2, width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(-width.x()/2, pow(-1,i)*width.z()/2, -width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(width.x()/2, pow(-1,i)*width.z()/2, -width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    glEnd();
  }
  for(int i = 0; i < 2; i++){
    glBegin(GL_POLYGON);
    R_vet = quat * Vector3d(width.x()/2, width.z()/2, pow(-1,i)*width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(-width.x()/2, width.z()/2, pow(-1,i)*width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(-width.x()/2, -width.z()/2, pow(-1,i)*width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    R_vet = quat * Vector3d(width.x()/2, -width.z()/2, pow(-1,i)*width.y()/2);
    glVertex3f(cog.x()+R_vet.x(), cog.y()-R_vet.y(), cog.z()+R_vet.z());
    glEnd();
  }
}

void drawVehicleQuad(Vector3d cog, Quaterniond Rot){
  double cog_x=cog(0), cog_y=cog(1), cog_z=cog(2);
  //Prop-guard circle
  glColor3f(0.8f, 0.95f, 0.8f);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.05303)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.05303)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.0125), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.05303)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.05303)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.01375), Rot);
      drawCylinder(cog_x-v(0), cog_y-v(2), (cog_z+v(1)),  cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)), vehicle_propeller_radius);
    }
  //Prop-guard Square
  glColor3f(0.1f, 0.95f, 0.1f);
  glLineWidth(2);
  glBegin(GL_LINE_LOOP);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.10783)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.10783)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.01625), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.10783)*cos(0.7854+(i+1)*1.5708),VEHICLE_SCALE(0.10783)*sin(0.7854+(i+1)*1.5708),VEHICLE_SCALE(-0.01625), Rot);
			
      glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));
    }
  glEnd();
  //Propellers
  glColor3f(0.8f, 0.95f, 0.6f);
  glBegin(GL_LINES);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.05303)*cos(0.7854+i*1.5708)+vehicle_propeller_radius*cos(6.2832/120*vehicle_wingangle++),VEHICLE_SCALE(0.05303)*sin(0.7854+i*1.5708)+vehicle_propeller_radius*sin(6.2832/120*vehicle_wingangle++),VEHICLE_SCALE(-0.01625), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.05303)*cos(0.7854+i*1.5708)-vehicle_propeller_radius*cos(6.2832/120*vehicle_wingangle++),VEHICLE_SCALE(0.05303)*sin(0.7854+i*1.5708)-vehicle_propeller_radius*sin(6.2832/120*vehicle_wingangle++),VEHICLE_SCALE(-0.01625), Rot);
      glVertex3f(cog_x-v(0), cog_y-v(2),(cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2),(cog_z+vv(1)));	
    }
  glEnd();
  //Body 4 cylinder
  glColor3f(0.8f, 0.6f, 0.8f);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.02651)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.02651)*sin(0.7854+i*1.5708),VEHICLE_SCALE(0), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.02651)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.02651)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.03125), Rot);
      drawCylinder(cog_x-v(0), cog_y-v(2), (cog_z+v(1)), cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)),VEHICLE_SCALE(0.01));
    }
  //Top square
  glColor3f(0.5f, 0.0f, 0.2f);
  glBegin(GL_POLYGON);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.02651)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.02651)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.03125), Rot);
      glVertex3f( cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
    }  
  glEnd();
  //Leg tiny legs
  glColor3f(0.8f, 0.95f, 0.6f);
  glBegin(GL_LINES);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.02651)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.02651)*sin(0.7854+i*1.5708),VEHICLE_SCALE(0), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.03276)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.03276)*sin(0.7854+i*1.5708),VEHICLE_SCALE(0), Rot);
      glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));
    }
  glEnd();
  //Prop-guard innerlines
  glColor3f(0.8f, 0.95f, 0.6f);
  glBegin(GL_LINES);
  for(int i=0;i<4;i++)
    {
      Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0.05303)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.05303)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.01625), Rot);
      Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.10783)*cos(0.7854+i*1.5708),VEHICLE_SCALE(0.10783)*sin(0.7854+i*1.5708),VEHICLE_SCALE(-0.01625), Rot);
			
      glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));			
			
      vv=local2Global(VEHICLE_SCALE(0.08497)*cos(0.45707+i*1.5708),VEHICLE_SCALE(0.08497)*sin(0.45707+i*1.5708),VEHICLE_SCALE(-0.01625), Rot);
			
      glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));
			
      vv=local2Global(VEHICLE_SCALE(0.08497)*cos(1.11373+i*1.5708),VEHICLE_SCALE(0.08497)*sin(1.11373+i*1.5708),VEHICLE_SCALE(-0.01625), Rot);
			
      glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
      glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));
    }
  glEnd();
  //Shadow
  glColor3f(0.5f, 0.0f, 0.2f);
  glLineWidth(2);
  glBegin(GL_LINES);
  glVertex3f(cog_x,cog_y,cog_z);glVertex3f(cog_x,0.0f,cog_z);
  glEnd();


  //direction
  glColor3f(0.8f, 0.8f, 0.2f);
  glLineWidth(4);
  glBegin(GL_LINES);
  Eigen::Vector3d v=local2Global(VEHICLE_SCALE(0),VEHICLE_SCALE(0),VEHICLE_SCALE(-0.0325), Rot);
  Eigen::Vector3d vv=local2Global(VEHICLE_SCALE(0.01875),VEHICLE_SCALE(0),VEHICLE_SCALE(-0.0325), Rot);
		
  glVertex3f(cog_x-v(0), cog_y-v(2), (cog_z+v(1)));
  glVertex3f(cog_x-vv(0), cog_y-vv(2), (cog_z+vv(1)));
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
  lx = saturate(lx, 1, 290);
  glLoadIdentity();
  gluLookAt(x - 1*lx_FrontBack*sin(0.01*lx), y - 1*sin(0.01*ly), z - 1*lz_FrontBack*sin(0.01*lz), 
	    x,y,z,
	    0.0f,cos(0.01*ly),0.0f);
  lx -= fuyang; ly -= fuyang; lz -= fuyang;
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
  Vector3d from(x,y,z);
  vector = R_BW*from;
  return vector;
}

Vector3d offsetCenter(Vector3d offset, Quaterniond quat)
{
  Quaterniond Rtemp(0,0,0.707,0.707);
  quat = Rtemp * quat * Rtemp;
  Vector3d output;
  output = quat * Vector3d(-offset.x(), -offset.z(), -offset.y());
  output.y() = -output.y();
  return output;
}

void renderScene(void) {
  //Calculate time
  gettimeofday(&tvend,NULL);
  double avgtime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
  if(Robot_move==true)
    gametime+=avgtime;
  gettimeofday(&tvstart,NULL);

  if(fuyang)
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

  /*************Background***************/
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
  
  /******Vehicle Body********/
  switch(vehicle_type)
    {
    case VEHICLE_QUAD :
      {
	drawVehicleQuad(COG, R_BW);
	break;
      }
    case VEHICLE_BI_UP:
      {
	//Body horizontal bar
	glColor3f(GLCOLOR_BLUE);
	drawRectangle3D(COG, Vector3d(VEHICLE_SCALE(0.03), 2*vehicle_bi_arm_length, VEHICLE_SCALE(0.02)), R_BW);
	//Body block
	glColor3f(GLCOLOR_DEEPPINK1X);
	Vector3d vehicle_bi_body_offset = offsetCenter(Vector3d(VEHICLE_SCALE(0.06), VEHICLE_SCALE(0.0), VEHICLE_SCALE(0.03)), R_BW);
	drawRectangle3D(COG + vehicle_bi_body_offset, Vector3d(VEHICLE_SCALE(0.09), VEHICLE_SCALE(0.05), VEHICLE_SCALE(0.06)), R_BW);
	//Servo left
	glColor3f(GLCOLOR_SPRINGGREEN1);
	Vector3d vehicle_servo_left_offset = offsetCenter(Vector3d(VEHICLE_SCALE(0),-vehicle_bi_arm_length,VEHICLE_SCALE(0.02)), R_BW);
	drawRectangle3D(COG + vehicle_servo_left_offset, Vector3d(VEHICLE_SCALE(0.02), VEHICLE_SCALE(0.04), VEHICLE_SCALE(0.05)), R_BW);
	//Servo right
	Vector3d vehicle_servo_right_offset = offsetCenter(Vector3d(VEHICLE_SCALE(0),vehicle_bi_arm_length,VEHICLE_SCALE(0.02)), R_BW);
	drawRectangle3D(COG + vehicle_servo_right_offset, Vector3d(VEHICLE_SCALE(0.02), VEHICLE_SCALE(0.04), VEHICLE_SCALE(0.05)), R_BW);
	//Rotor Left
	glColor3f(GLCOLOR_RED3);
	Vector3d vehicle_rotor_left_bottom = COG + offsetCenter(Vector3d(VEHICLE_SCALE(0),-vehicle_bi_arm_length,VEHICLE_SCALE(0.05)), R_BW);
	//FIXME change R_BW here to have additional servo rotation
	Vector3d vehicle_rotor_left_top = COG + offsetCenter(R_LSB*Vector3d(VEHICLE_SCALE(0),-vehicle_bi_arm_length,VEHICLE_SCALE(0.08)), R_BW);
	drawCylinder(vehicle_rotor_left_bottom.x(), vehicle_rotor_left_bottom.y(), vehicle_rotor_left_bottom.z(),
		     vehicle_rotor_left_top.x(),    vehicle_rotor_left_top.y(),    vehicle_rotor_left_top.z(), vehicle_propeller_radius);
	//Rotor Right
	Vector3d vehicle_rotor_right_bottom = COG + offsetCenter(Vector3d(VEHICLE_SCALE(0),vehicle_bi_arm_length,VEHICLE_SCALE(0.05)), R_BW);
	//FIXME change R_BW here to have additional servo rotation
	Vector3d vehicle_rotor_right_top = COG + offsetCenter(R_RSB*Vector3d(VEHICLE_SCALE(0),vehicle_bi_arm_length,VEHICLE_SCALE(0.08)), R_BW);
	drawCylinder(vehicle_rotor_right_bottom.x(), vehicle_rotor_right_bottom.y(), vehicle_rotor_right_bottom.z(),
	vehicle_rotor_right_top.x(),    vehicle_rotor_right_top.y(),    vehicle_rotor_right_top.z(), vehicle_propeller_radius);
	//Shadow
	glColor3f(0.5f, 0.0f, 0.2f);
	glLineWidth(2);
	glBegin(GL_LINES);
	glVertex3f(vehicle_position_x,vehicle_position_y,vehicle_position_z);glVertex3f(vehicle_position_x,0.0f,vehicle_position_z);
	glEnd();
	//Direction
	break;
      }
    case VEHICLE_BI_DOWN:
      {
	break;
      }
    default:
      break;
    }

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
  switch(vehicle_goal_type){
  case VEHICLE_BALL:{
    glColor3f(1.0f, 0.2f, 0.2f);
    drawSphere(goal_position_x, goal_position_y, goal_position_z, 0.03, 20, 20);
    break;
  }
  case VEHICLE_QUAD:{
    drawVehicleQuad(COG_GOAL, R_GOW);
    break;
  }
  }
    

  FTGLPixmapFont font(FONT_FILE_PATH.c_str());
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
  if(!ros::ok())
    {
      ros::shutdown();
      exit(0);
    }
  ros::spinOnce();

}

void mouseMotion(int x, int y){
  static bool wrap = false;
  if(!wrap) {
    int ww = glutGet(GLUT_WINDOW_WIDTH);
    int wh = glutGet(GLUT_WINDOW_HEIGHT);

    int dx = x - ww / 2;
    int dy = y - wh / 2;

    // Do something with dx and dy here
    //cerr<<dx<<"  "<<dy<<endl;
    deltaAngle = deltaAngleFilter.update((dx>0?1:-1)*0.003*saturate(pow(abs(dx),2), 0, 15));
    fuyang = fuyangFilter.update((dy>0?1:-1)*0.14*saturate(pow(abs(dy),2), 0, 15));
    // move mouse pointer back to the center of the window
    wrap = true;
    glutWarpPointer(ww / 2, wh / 2);
  } else {
    wrap = false;
  }
}

void mouseRelease(int button, int state, int x, int y){
  if(state == 0){
    glutSetCursor(GLUT_CURSOR_NONE);
  }
  // Capture mouse button release
  if(state == 1){
    fuyang = 0;
    deltaAngle = 0;
    glutSetCursor(GLUT_CURSOR_INHERIT);
  }
}

void pressKey(unsigned char key, int x, int y) {

  switch (key) {
  case 'f' : fuyang=0.8;break;
  case 'r' : fuyang=-0.8;break;
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
  case  27 : ;// when pressing 'Esc': intended fall through
  case  3  : ros::shutdown();exit(0);break;// ctrl + c is pressed
  }
}

void releaseKey(unsigned char key, int x, int y) {

  switch (key) {
  case 'f' : fuyang=0;break;
  case 'r' : fuyang=0;break;
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

  //initial ROS
  ros::init(argc, argv, "glviewer");

  ros::NodeHandle x_handle;

  ros::Subscriber sub_position = x_handle.subscribe("/pose", 1000, x_position_Callback);
  ros::Subscriber sub_goal = x_handle.subscribe("/goal", 1000, x_goal_Callback);

  // for Bicopter only
  ros::Subscriber sub_servo_left  = x_handle.subscribe("/left_servo", 1000, left_angle_Callback);
  ros::Subscriber sub_servo_right = x_handle.subscribe("/right_servo", 1000, right_angle_Callback);

  char* username;
  username = (char *)malloc(4*sizeof(char));
  cuserid(username);
  string USERNAME(username);
  FONT_FILE_PATH.replace(6,8,USERNAME);

  //Initial vehicle
  vehicle_position_x=10.0;vehicle_position_y=0.0;vehicle_direction=rand()/(double)(RAND_MAX/6.282)-3.141;
  COG << vehicle_position_x, vehicle_position_y, vehicle_position_z;
  COG_GOAL << goal_position_x, goal_position_y, goal_position_z;
  R_BW = R_WGL * R_BW * R_WGL;
  R_GOW = R_WGL * R_GOW * R_WGL;
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100,100);
  glutInitWindowSize(640,360);
  glutCreateWindow("Rotorcraft Realtime Visualization and Playback");

  glEnable(GL_DEPTH_TEST);

  glutIgnoreKeyRepeat(1);
  glutKeyboardFunc(pressKey);
  glutKeyboardUpFunc(releaseKey);

  glutMotionFunc(mouseMotion);
  glutMouseFunc(mouseRelease);
  
  //glutDisplayFunc(renderScene);
  glutIdleFunc(renderScene);
	
  glutReshapeFunc(changeSize);
  //glutTimerFunc(20000,change_direction,0);
  //glutTimerFunc(5000,error_direction,0);
  glutMainLoop();

  return(0);
}
