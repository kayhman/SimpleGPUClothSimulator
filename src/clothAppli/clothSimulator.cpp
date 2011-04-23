#include <iostream>
#include "clothSimulation.h"

ClothSimulation clothSimulation(150, 150);


//
// This code was created by Jeff Molofee '99 (ported to Linux/GLUT by Richard Campbell '99)
//
// If you've found this code useful, please let me know.
//
// Visit me at www.demonews.com/hosted/nehe 
// (email Richard Campbell at ulmont@bellsouth.net)
//
#include <GL/glut.h>    // Header File For The GLUT Library 
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <unistd.h>     // needed to sleep

/* ASCII code for the escape key. */
#define ESCAPE 27

/* The number of our GLUT window */
int window; 

/* rotation angle for the triangle. */
float rtri = 0.0f;

/* rotation angle for the quadrilateral. */
float rquad = 0.0f;

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(int Width, int Height)	        // We call this right after our OpenGL window is created.
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
  glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
  glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				// Reset The Projection Matrix

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

  glMatrixMode(GL_MODELVIEW);
}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
    Height=1;

  glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
  glMatrixMode(GL_MODELVIEW);
}

/* The main drawing function. */
void DrawGLScene()
{
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	// Clear The Screen And The Depth Buffer
  glLoadIdentity();				// Reset The View

  glTranslatef(-1.5f,0.0f,-6.0f);		// Move Left 1.5 Units And Into The Screen 6.0
	
  glRotatef(rtri,0.0f,1.0f,0.0f);		// Rotate The Pyramid On The Y axis 

  // draw a pyramid (in smooth coloring mode)
  glBegin(GL_TRIANGLES);				// start drawing a pyramid


  int gridSizeX = clothSimulation.getSizeX();
  int gridSizeY = clothSimulation.getSizeY();
  std::vector<float>& X = clothSimulation.getNodeX();
  std::vector<float>& Y = clothSimulation.getNodeY();
  std::vector<float>& Z = clothSimulation.getNodeZ();
  for(int i = 0 ; i < gridSizeX-1 ; ++i)
	  for(int j = 0 ; j < gridSizeY-1 ; ++j)
		  {
		  	  float nodeX[4];
		  	  float nodeY[4];
		  	  float nodeZ[4];
		  	  
		  	  nodeX[0] = X[i * gridSizeY + j];
		  	  nodeY[0] = Y[i * gridSizeY + j];
		  	  nodeZ[0] = Z[i * gridSizeY + j];

		  	  nodeX[1] = X[i * gridSizeY + j + 1 ];
		  	  nodeY[1] = Y[i * gridSizeY + j + 1 ];
		  	  nodeZ[1] = Z[i * gridSizeY + j + 1 ];
		  	  
		  	  nodeX[2] = X[(i + 1) * gridSizeY + j];
		  	  nodeY[2] = Y[(i + 1) * gridSizeY + j];
		  	  nodeZ[2] = Z[(i + 1) * gridSizeY + j];
		  	  
			  nodeX[3] = X[(i + 1) * gridSizeY + j + 1 ];
		  	  nodeY[3] = Y[(i + 1) * gridSizeY + j + 1 ];
		  	  nodeZ[3] = Z[(i + 1) * gridSizeY + j + 1 ];


		  	  glColor3f(1.0f,0.0f,0.0f);			// Set The Color To Red
  			  glVertex3f(nodeX[0], nodeY[0], nodeZ[0]);		        // Top of triangle (front)
  			  glColor3f(0.0f,1.0f,0.0f);			// Set The Color To Green
			  glVertex3f(nodeX[1], nodeY[1], nodeZ[1]);		// left of triangle (front)
			  glColor3f(0.0f,0.0f,1.0f);			// Set The Color To Blue
			  glVertex3f(nodeX[2], nodeY[2], nodeZ[2]);		        // right of traingle (front)	
		  	
		  	
		  	  glColor3f(1.0f,0.0f,0.0f);			// Set The Color To Red
  			  glVertex3f(nodeX[2], nodeY[2], nodeZ[2]);		        // Top of triangle (front)
  			  glColor3f(0.0f,1.0f,0.0f);			// Set The Color To Green
			  glVertex3f(nodeX[1], nodeY[1], nodeZ[1]);		// left of triangle (front)
			  glColor3f(0.0f,0.0f,1.0f);			// Set The Color To Blue
			  glVertex3f(nodeX[3], nodeY[3], nodeZ[3]);		        // right of traingle (front)	
		  }


  // front face of pyramid

  glEnd();					// Done Drawing The Pyramid

  rtri+= 0.15f;					// Increase The Rotation Variable For The Pyramid
  rquad-= 0.15f;					// Decrease The Rotation Variable For The Cube

  // swap the buffers to display, since double buffering is used.
  glutSwapBuffers();
}

/* The function called whenever a key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{
    /* avoid thrashing this call */
    usleep(100);

    /* If escape is pressed, kill everything. */
    if (key == ESCAPE) 
    { 
      /* shut down our window */
      glutDestroyWindow(window); 
      
      /* exit the program...normal termination. */
      exit(0);                   
    }
}

int main(int argc, char **argv) 
{ 
	clothSimulation.init(0.5, 0., 0.,
							1.0, 1.0,
							1e3, 1e3);

 
  /* Initialize GLUT state - glut will take any command line arguments that pertain to it or 
     X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html */  
  glutInit(&argc, argv);  

  /* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffered for automatic clipping */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* get a 640 x 480 window */
  glutInitWindowSize(640, 480);  

  /* the window starts at the upper left corner of the screen */
  glutInitWindowPosition(0, 0);  

  /* Open a window */  
  window = glutCreateWindow("Jeff Molofee's GL Code Tutorial ... NeHe '99");  

  /* Register the function to do all our OpenGL drawing. */
  glutDisplayFunc(&DrawGLScene);  

  /* Go fullscreen.  This is as soon as possible. */
  glutFullScreen();

  /* Even if there are no events, redraw our gl scene. */
  glutIdleFunc(&DrawGLScene);

  /* Register the function called when our window is resized. */
  glutReshapeFunc(&ReSizeGLScene);

  /* Register the function called when the keyboard is pressed. */
  glutKeyboardFunc(&keyPressed);

  /* Initialize our window. */
  InitGL(640, 480);
  
  
  
  
  /* Start Event Processing Engine */  
  glutMainLoop();  
return 0;
	
	//clothSimulation.computeInternalForces();

	//clothSimulation.transfertFromGpu();

	for(int i = 0 ; i < 512  ; ++i)
	{
		std::cout << clothSimulation.getNodeY()[i] << std::endl;
	}

	std::cout << "welcome into our cloth simulator. Please select an item." << std::endl;


  return 1;
}