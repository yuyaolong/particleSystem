#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <cmath>
#include <vector>
#include"Vector.h"
#include "Camera.h"
#include "gauss.h"
#include "Particle.h"

#define automaticSpeed 10
int WIDTH = 800;
int HEIGHT = 800;

//int stopSign = 0;

double hStep = 0.1;

Camera *camera;
std::vector<Particle>particles;


Vector3d particleAcceleration(0, -1, 0);


void mouseEventHandler(int button, int state, int x, int y) {
    // let the camera handle some specific mouse events (similar to maya)
    camera->HandleMouseEvent(button, state, x, y);
    glutPostRedisplay();
}

void motionEventHandler(int x, int y) {
    // let the camera handle some mouse motions if the camera is to be moved
    camera->HandleMouseMotion(x, y);
    glutPostRedisplay();
}



void init() {
    //LoadParameters(ParamFilename);
    // set up camera
    // parameters are eye point, aim point, up vector
    camera = new Camera(Vector3d(0, 0, 1), Vector3d(0, 0, 0), Vector3d(0, 1, 0));
    
    // black background for window
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    //glDepthRange(0, 1);
    
    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    particles.reserve(1000);
    for (float i = -5; i<5; i+=0.1) {
        for (float j = -10; j<10; j+=0.1) {
            particles.push_back( Particle(Vector3d(i,j,j), Vector3d(0,0,0), Vector4d(0,1,1,1), 1, 10, 0.5, false) );//position,velocity,color,mass,lifespan,pointsize,stopSign
        }
    }
    
}



void myDisplay(void)
{
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    // draw the camera created in perspective
    camera->PerspectiveDisplay(WIDTH, HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
 
    //glTranslatef(0,0,0);
    glBegin(GL_POINTS);
    for (int i = 0; i < particles.size(); ++i) {
        
        glPointSize(particles[i].getPointSize());
        glColor4f(particles[i].getColor().x, particles[i].getColor().y, particles[i].getColor().z, particles[i].getColor().w);
        glVertex3f(particles[i].getPosition().x, particles[i].getPosition().y, particles[i].getPosition().z);
    }
    glEnd();

    
    //glFlush();
    glutSwapBuffers();
}

void calculatePosition()
{
    Vector3d particlePositionNew, particleVelocityNew;
    for (int i=0; i<particles.size(); ++i) {
        if (particles[i].getStopSign() == false) {
            particleVelocityNew = particles[i].getVelocity() + particleAcceleration*hStep;
            particlePositionNew = particles[i].getPosition() + particles[i].getVelocity()*hStep;
            
            //cout<<"Velocity: "<<particleVelocityNew<<endl;
            //cout<<"Postion:  "<<particlePositionNew<<endl;
            
            if (particlePositionNew.y >-10) {
                particles[i].setVelocity(particleVelocityNew);
                particles[i].setPosition(particlePositionNew);
            }
            else
            {
                particles[i].setStopSign(true);
            }
        }
    }
   
}

void timeProc(int id)
{
    if (id == 1) {
            calculatePosition();
            glutPostRedisplay();
        
            glutTimerFunc(automaticSpeed, timeProc, 1);

        
    }
    
}

void handleKey(unsigned char key, int x, int y){
    switch(key){
        case 'a':
        case 'A':
            glutTimerFunc(automaticSpeed, timeProc, 1);
            break;
            
        case 'm':
        case 'M':
            calculatePosition();
            glutPostRedisplay();
            break;
        case 'q':       // q - quit
        case 'Q':
        case 27:        // esc - quit
            exit(0);
            
        default:        // not a valid key -- just ignore it
            return;
    }
}


int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Particle");
    
    init();
    
    glutDisplayFunc(myDisplay);
    glutMouseFunc(mouseEventHandler);
    glutMotionFunc(motionEventHandler);
    glutKeyboardFunc(handleKey);
    
    glutMainLoop();
    
    delete camera;
    return 0;
}