#ifndef GRAPH3DAXIS_H
#define GRAPH3DAXIS_H
#include "config.h"

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FL/glut.H>

#define AXIS_TOL 4.0

class Graph3DAxis : public Fl_Gl_Window {
	
public:
	Graph3DAxis(int x,int y,int w,int h,const char *l=0);
	void draw();
	void set3DAxis(double pose[6]);
	void set3DAxis(float pose[6]);
	
private:
	double x, y, z;
	double t_x, t_y, t_z;
	
	GLdouble camera_position[3], camera_direction[3], up_direction[3];
	
	int width, height;
	int font;
	float axis_pose[6];
	float pos_axis_color[3];
	float neg_axis_color[3];
	float zer_axis_color[3];
	float axis_size;
	float base_pose[6];
	float base_color[3];
	float base_size;
			
	void draw3DAxis();	
	void drawAxes(float pose[6], float color[3], float size, bool base_frame);
};

#endif
