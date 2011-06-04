#include <iostream>
#include "clothSimulation.h"
#include <math.h>


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

   GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat mat_shininess[] = { 50.0 };
   GLfloat light0_position[] = { 0, 7., 0., 0.0 }; // LR / UD / FB
   GLfloat light1_position[] = { -5., 3., 3., 0.0 };
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_SMOOTH);

   glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
   glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
   glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
   glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);
  //glPolygonMode(GL_FRONT, GL_LINE);

  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
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
	static bool init = true;
	static ClothSimulation clothSimulation;

	if(init)
	{
		clothSimulation.init(-2.0, -1.0, 0.1,
					4.0, 2.0,
					4.0 / 250,
					50.51, 0.);
		//clothSimulation.init(-2.0, -2.0, 0.1,
		//			4.0, 4.0,
		//			4.0 / 250,
		//			50.51, 0.);
		init = false;
	}
	
	clothSimulation.computeInternalForces();
	clothSimulation.handleCollision(1);
	clothSimulation.integrate();
	clothSimulation.transfertFromGpu();

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	// Clear The Screen And The Depth Buffer
  glLoadIdentity();				// Reset The View

  glTranslatef(0.f,0.0f,-6.0f);		// Move Left 1.5 Units And Into The Screen 6.0
  glRotatef(110.0,1.0f,0.0f,0.0f);		// Rotate The Pyramid On The Y axis 
	
  glRotatef(rtri,0.0f,0.0f,1.0f);		// Rotate The Pyramid On The Y axis 

  glBegin(GL_TRIANGLES);				// start drawing a pyramid


  int gridSizeX = clothSimulation.getSizeX();
  int gridSizeY = clothSimulation.getSizeY();
  std::vector<float>& X = clothSimulation.getNodeX();
  std::vector<float>& Y = clothSimulation.getNodeY();
  std::vector<float>& Z = clothSimulation.getNodeZ();
 
  std::vector<float> nX(X.size());
  std::vector<float> nY(Y.size());
  std::vector<float> nZ(Z.size());
 
  std::fill(nX.begin(), nX.end(), 0.);
  std::fill(nY.begin(), nY.end(), 0.);
  std::fill(nZ.begin(), nZ.end(), 0.);
 
 ///////////////////////////
 //     smooth normals    //
 ///////////////////////////
  for(int i = 0 ; i < gridSizeX-1 ; ++i)
	  for(int j = 0 ; j < gridSizeY-1 ; ++j)
		  {
		  	  int n1Idx = i * gridSizeY + j;
		  	  int n2Idx = i * gridSizeY + j + 1;
		  	  int n3Idx = (i+1) * gridSizeY + j;
		  	  int n4Idx = (i+1) * gridSizeY + j + 1;
		  	  

			  float nx = (Y[n2Idx] - Y[n1Idx]) * (Z[n3Idx] - Z[n1Idx])-  (Y[n3Idx] - Y[n1Idx]) * (Z[n2Idx] - Z[n1Idx]);
			  float ny = (Z[n2Idx] - Z[n1Idx]) * (X[n3Idx] - X[n1Idx]) - (Z[n3Idx] - Z[n1Idx]) * (X[n2Idx] - X[n1Idx]);
			  float nz = (X[n2Idx] - X[n1Idx]) * (Y[n3Idx] - Y[n1Idx]) - (X[n3Idx] - X[n1Idx]) * (Y[n2Idx] - Y[n1Idx]);
				
			nX[n1Idx] += nx;
			nY[n1Idx] += ny;
			nZ[n1Idx] += nz;
			
			nX[n2Idx] += nx;
			nY[n2Idx] += ny;
			nZ[n2Idx] += nz;
			
			nX[n3Idx] += nx;
			nY[n3Idx] += ny;
			nZ[n3Idx] += nz;
			
			nX[n4Idx] += nx;
			nY[n4Idx] += ny;
			nZ[n4Idx] += nz;
}  
 
for(int i = 0 ; i < nX.size() ; i++)
{
	float norm = sqrt(nX[i] * nX[i] + nY[i] * nY[i] + nZ[i] * nZ[i]);
	if(norm)
	{
	nX[i] /= -norm;
	nY[i] /= -norm;
	nZ[i] /= -norm;
	}
}  

///////////////////////////
//    Display triangles  //
///////////////////////////
  for(int i = 0 ; i < gridSizeX-1 ; ++i)
	  for(int j = 0 ; j < gridSizeY-1 ; ++j)
		  {
			  int n1Idx = i * gridSizeY + j;
		  	  int n2Idx = i * gridSizeY + j + 1;
		  	  int n3Idx = (i+1) * gridSizeY + j;
		  	  int n4Idx = (i+1) * gridSizeY + j + 1;
		  	  
			  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Red
  			  glVertex3f(X[n1Idx], Y[n1Idx], -Z[n1Idx]);		        // Top of triangle (front)
  			  glNormal3f(-nX[n1Idx], -nY[n1Idx], -nZ[n1Idx]);		        // Top of triangle (front)
  			  
			  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Green
			  glVertex3f(X[n2Idx], Y[n2Idx], -Z[n2Idx]);		// left of triangle (front)
 			  glNormal3f(-nX[n2Idx], -nY[n2Idx], -nZ[n2Idx]);		        // Top of triangle (front)
 			  
			  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Blue
			  glVertex3f(X[n3Idx], Y[n3Idx], -Z[n3Idx]);		        // right of traingle (front)	
 			  glNormal3f(-nX[n3Idx], -nY[n3Idx], -nZ[n3Idx]);		        // Top of triangle (front)
 		  	
		  	
		  	  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Red
  			  glVertex3f(X[n3Idx], Y[n3Idx], -Z[n3Idx]);		        // Top of triangle (front)
  			  glNormal3f(-nX[n3Idx], -nY[n3Idx], -nZ[n3Idx]);		        // Top of triangle (front)
  			  
			  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Green
			  glVertex3f(X[n2Idx], Y[n2Idx], -Z[n2Idx]);		// left of triangle (front)
 			  glNormal3f(-nX[n2Idx], -nY[n2Idx], -nZ[n2Idx]);		        // Top of triangle (front)
 			  
			  glColor3f(0.0f,0.3f,0.7f);			// Set The Color To Blue
			  glVertex3f(X[n4Idx], Y[n4Idx], -Z[n4Idx]);		        // right of traingle (front)	
 			  glNormal3f(-nX[n4Idx], -nY[n4Idx], -nZ[n4Idx]);		        // Top of triangle (front)
 		  }

  // front face of pyramid

  glEnd();					// Done Drawing The Pyramid

  rtri+= 0.15f;					// Increase The Rotation Variable For The Pyramid
  rquad-= 0.015f*0.;					// Decrease The Rotation Variable For The Cube

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
	
}
