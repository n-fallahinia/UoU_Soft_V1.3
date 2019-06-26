//
// "$Id: Plot2DView.cxx 5519 2006-10-11 03:12:15Z mike $"
//
// Modified version of CubeView (modified by Thomas R. Grieve 2011-11-18)
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

// Used to plot Maglev rotation

#include "Plot2DView.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

Plot2DView::Plot2DView(int x,int y,int w,int h,const char *l)
            : Fl_Gl_Window(x,y,w,h,l)
{
        // Define the 
        vAng = 0.0;
        hAng=0.0;
        xshift=0.0;
        yshift=0.0;
        center_w = w/2.0;
        center_h = h/2.0;
        
        // The cube definition.  These are the vertices of a unit cube centered
        //      on the origin.
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
Plot2DView::drawCube() {
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

void 
Plot2DView::draw() {
        if (!valid()) {
                glLoadIdentity();
                glViewport(0,0,w(),h());
                glOrtho(-10,10,-10,10,-20050,10000);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        //Draw line to show where zero, +/-4 are
        float zero0[] = {-8.0, 0.0, 0.0};
        float zero1[] = {8.0, 0.0, 0.0};
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_LINES);
        for (int i = -4; i <= 4; i += 4)
        {
                zero0[1] = (float)i;
                zero1[1] = (float)i;
                glVertex3fv(zero0);
                glVertex3fv(zero1);
        }
        glEnd();

        //Title
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(-4.0, 7.0, 0.0);   
        char p_t[] = "MLD Angles";   
        gl_draw(p_t, strlen(p_t));

        draw_ylabel();

        //Draw the angle x
        glPushMatrix();
                glTranslatef(-5.0, 0.0, 0.0);
                glTranslatef(0.0, float(ROT_SCALE*height_x/2.0), 0.0);
                //printf("(%5.2f,", ROT_SCALE*height_x/2.0);
                glScalef(2.0,float(ROT_SCALE*height_x), 1.0);
                drawCube();
        glPopMatrix();



        //Label the bar to represent the angle x
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(-5.0, -8.9, 0.0);   
        char p_x[] = "X";   
        gl_draw(p_x, strlen(p_x));

        //Draw the angle y
        glPushMatrix();
                glTranslatef(0.0, 0.0, 0.0);
                glTranslatef(0.0, float(ROT_SCALE*height_y/2.0), 0.0);
                //printf(" %5.2f,", ROT_SCALE*height_y/2.0);
                glScalef(2.0,float(ROT_SCALE*height_y), 1.0);
                drawCube();
        glPopMatrix();

        //Label the bar to represent the angle y
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(0.0, -8.9, 0.0);   
        char p_y[] = "Y";   
        gl_draw(p_y, strlen(p_y));

        //Draw the angle Z
        glPushMatrix();
                glTranslatef(5.0, 0.0, 0.0);
                glTranslatef(0.0, float(ROT_SCALE*height_z/2.0), 0.0);
                //printf(" %5.2f)\n", ROT_SCALE*height_z/2.0);
                glScalef(2.0,float(ROT_SCALE*height_z), 1.0);
                drawCube();
        glPopMatrix();

        //Label the bar to represent the angle z
        gl_font(1, 12);
        glColor3f(1.0, 1.0, 1.0);
        glRasterPos3f(5.0, -8.9, 0.0);   
        char p_z[] = "Z";   
        gl_draw(p_z, strlen(p_z));

}


void 
Plot2DView::draw_ylabel()
{

	//Label the zero line
	gl_font(1, 12);
	glColor3f(1.0, 1.0, 1.0);
	
	// Declare the initial value for the string to hold the y-axis label
	char p[] = "  0";
        
        // Draw all of the y-axis labels
        for (int i = -8; i <= 8; i+=4)
        {
	        glRasterPos3f(ROT_AXIS, (float)i, 0.0);
                if (i < 0)
                {
                        sprintf(p, "%d", i);
                }
                else
	        {   
	                sprintf(p, " %d", i);
                }
	        gl_draw(p, strlen(p));
        }

/*	glRasterPos3f(ROT_AXIS, 2, 0);   
	sprintf(p, " 2");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 3,0);   
	sprintf(p, " 3");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 4, 0);   
	sprintf(p, " 4");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 5, 0);   
	sprintf(p, " 5");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 6, 0);   
	sprintf(p, " 6");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 7, 0);   
	sprintf(p, " 7");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 8,0);   
	sprintf(p, " 8");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, 9, 0);   
	sprintf(p, " 9");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -1, 0);   
	sprintf(p, "-1");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -2, 0);   
	sprintf(p, "-2");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -3, 0);   
	sprintf(p, "-3");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -4, 0);   
	sprintf(p, "-4");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -5, 0);   
	sprintf(p, "-5");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -6, 0);   
	sprintf(p, "-6");   
	gl_draw(p, strlen(p));

	glRasterPos3f(ROT_AXIS, -7, 0);   
	sprintf(p, "-7");   
	gl_draw(p, strlen(p));
	glRasterPos3f(ROT_AXIS, -8, 0);   
	sprintf(p, "-8");   
	gl_draw(p, strlen(p));
	
	glRasterPos3f(ROT_AXIS, -9, 0);   
	sprintf(p, "-9");   
	gl_draw(p, strlen(p));*/


}

