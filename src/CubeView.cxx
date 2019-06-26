//
// "$Id: CubeView.cxx 5519 2006-10-11 03:12:15Z mike $"
//
// CubeView class implementation for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2005 by Bill Spitzak and others.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
//
// Please report all bugs and problems on the following page:
//
//     http://www.fltk.org/str.php
//

// Used to plot sensor force/torque

#include "CubeView.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

CubeView::CubeView(int x,int y,int w,int h,const char *l)
            : Fl_Gl_Window(x,y,w,h,l)
{
    vAng = 0.0;
    hAng=0.0;
    xshift=0.0;
    yshift=0.0;
    center_w = w/2.0;
    center_h = h/2.0;
    
    // Set the boxes to height zero
    height_x = 0.0; 
    height_y = 0.0; 
    height_z = 0.0;
    height_rx = 0.0;
    height_ry = 0.0;
    height_rz = 0.0;
    
    /* The cube definition. These are the vertices of a unit cube
     * centered on the origin.*/
    
    boxv0[0] = -0.5; boxv0[1] = -0.5; boxv0[2] = -0.5;
    boxv1[0] =  0.5; boxv1[1] = -0.5; boxv1[2] = -0.5;
    boxv2[0] =  0.5; boxv2[1] =  0.5; boxv2[2] = -0.5;
    boxv3[0] = -0.5; boxv3[1] =  0.5; boxv3[2] = -0.5;
    boxv4[0] = -0.5; boxv4[1] = -0.5; boxv4[2] =  0.5;
    boxv5[0] =  0.5; boxv5[1] = -0.5; boxv5[2] =  0.5;
    boxv6[0] =  0.5; boxv6[1] =  0.5; boxv6[2] =  0.5;
    boxv7[0] = -0.5; boxv7[1] =  0.5; boxv7[2] =  0.5;

}

void 
CubeView::drawCube() 
{
  /* Draw a colored cube */
  #define ALPHA 0.5
    glShadeModel(GL_FLAT);

    glBegin(GL_QUADS);
      glColor4f(0.0, 0.0, 1.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv1);
      glVertex3fv(boxv2);
      glVertex3fv(boxv3);

      glColor4f(1.0, 1.0, 0.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv4);
      glVertex3fv(boxv5);
      glVertex3fv(boxv1);

      glColor4f(0.0, 1.0, 1.0, ALPHA);
      glVertex3fv(boxv2);
      glVertex3fv(boxv6);
      glVertex3fv(boxv7);
      glVertex3fv(boxv3);

      glColor4f(1.0, 0.0, 0.0, ALPHA);
      glVertex3fv(boxv4);
      glVertex3fv(boxv5);
      glVertex3fv(boxv6);
      glVertex3fv(boxv7);

      glColor4f(1.0, 0.0, 1.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv3);
      glVertex3fv(boxv7);
      glVertex3fv(boxv4);

      glColor4f(0.0, 1.0, 0.0, ALPHA);
      glVertex3fv(boxv1);
      glVertex3fv(boxv5);
      glVertex3fv(boxv6);
      glVertex3fv(boxv2);
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
      glVertex3fv(boxv0);
      glVertex3fv(boxv1);

      glVertex3fv(boxv1);
      glVertex3fv(boxv2);

      glVertex3fv(boxv2);
      glVertex3fv(boxv3);

      glVertex3fv(boxv3);
      glVertex3fv(boxv0);

      glVertex3fv(boxv4);
      glVertex3fv(boxv5);

      glVertex3fv(boxv5);
      glVertex3fv(boxv6);

      glVertex3fv(boxv6);
      glVertex3fv(boxv7);

      glVertex3fv(boxv7);
      glVertex3fv(boxv4);

      glVertex3fv(boxv0);
      glVertex3fv(boxv4);

      glVertex3fv(boxv1);
      glVertex3fv(boxv5);

      glVertex3fv(boxv2);
      glVertex3fv(boxv6);

      glVertex3fv(boxv3);
      glVertex3fv(boxv7);
    glEnd();
}//drawCube

void CubeView::draw() 
{
        if (!valid()) {
                glLoadIdentity();
                glViewport(0,0,w(),h());
                glOrtho(-10,10,-10,10,-20050,10000);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        //Draw a line to show where zero is
        float zero0[] = {-8.0, 3.2, 0.0};
        float zero1[] = {8.0, 3.2, 0.0};
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_LINES);
                glVertex3fv(zero0);
                glVertex3fv(zero1);
        glEnd();

        //Title
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(-7.0, 8.9, 0.0);   
        char p_t[] = "Measured Forces/Torques";   
        gl_draw(p_t, strlen(p_t));
        
        // Draw the y-axis label
        draw_ylabels();
        
        //Draw the forces/torques in each direction
        draw_force_dir(F_SCALE*height_x, -5.0, "Fx");
        draw_force_dir(F_SCALE*height_y, -3.0, "Fy");
        draw_force_dir(F_SCALE*height_z, -1.0, "Fz");
        draw_force_dir(R_SCALE*height_rx, 1.0, "Tx");
        draw_force_dir(R_SCALE*height_ry, 3.0, "Ty");
        draw_force_dir(R_SCALE*height_rz, 5.0, "Tz");
}

// Draws the label for the force direction and the box showing its value
//      (height), at the specified x-position (x_offset)
void CubeView::draw_force_dir(double height, float x_offset, char* my_label)
{
        //Draw the box for this direction of force
        glPushMatrix();
                glTranslatef(x_offset, 0.0, 0.0);
                glTranslatef(0.0, float(height/2.0)+3.2, 0.0);
                // printf(" %5.2f)\n", height/2.0);
                glScalef(1.0,float(height), 1.0);
                drawCube();
        glPopMatrix();
        
        // Label the box for Torque z
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(x_offset-0.5, -8.9, 0.0);   
        char r_z[] = "  ";
        sprintf(r_z, "%s", my_label);
        gl_draw(my_label, strlen(my_label));
} // draw_force_dir

void 
CubeView::draw_ylabels()
{

	//Label the zero line
	gl_font(1, 12);
	glColor3f(1.0, 1.0, 1.0);
	
	// Declare the initial value for the string to hold the y-axis label
	char f[] = "1";
	char r[] = "0";
  
  // Draw all of the y-axis labels
  for (int i = -15; i <= 6; i+=1)
  {
    glRasterPos3f(F_AXIS, i+3, 0.0);
    if (i < 0 )
    {
      sprintf(f, "%d", i);
    }
    else
    {
      sprintf(f, " %d", i); 
    }

    // if (i < -9)
    // {   
    //   sprintf(f, "%d", i);
    // }
    // else if ((i < 0) || (i > 9))
    // {
    //   sprintf(f, "%d", i);
    // }
    // else
    // {   
    //   sprintf(f, "%d", i);
    // }
    gl_draw(f, strlen(f));
    // glRasterPos3f(R_AXIS, (float)i, 0.0);
    // sprintf(r, "%5.2f", (float)i*30.0/1000.0);
    // gl_draw(r, strlen(r));
  }
}

