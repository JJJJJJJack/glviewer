#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>


#define pi 3.1415926

using namespace std;

struct sep_part{
	double x;
	double y;
};
struct Point{
	double x;
	double y;
};
struct line_part{
	double start_x;
	double start_y;
	double stop_x;
	double stop_y;
	double k;
	double b;
	double length;
};
struct structure_window{
	Point left;
	Point right;
	Point center;
};
struct structure_door{
	Point left;
	Point right;
	Point center;
};
struct door_detected{
	bool new_detected;
	int num_of_door;
};

vector<sep_part> separate[300];
vector<sep_part> sep_corner[300];
line_part line[300],longline[500][300];
//structure_window window;
vector<structure_door> door,door1,door2,door3,door4,door5,door6,door7,door8,door9,door10;
structure_door temp_door,window;
sep_part part;
door_detected is_door_detected;
//bool is_window_detected;


int NUM_DOOR=0;
structure_door MSG_DOOR1,MSG_DOOR2,MSG_DOOR3,MSG_DOOR4,MSG_DOOR5,MSG_WINDOW;


double mean(double x1,double x2,double x3){
	return (x1+x2+x3)/(double)3.0;
}

