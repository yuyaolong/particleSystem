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

#define automaticSpeed 20
#define cleanSpeed 3000
#define PRECISION  0.1

int WIDTH = 1000;
int HEIGHT = 800;


Vector3d rockPosition(0,0,0);
double Tolerance = 1;

double ParticleDense = 0.05;

double PlaneHeight = 0;

double hStep = 0.1;

Camera *camera;
std::vector<Particle>particles;
std::vector<Particle>::iterator it;


Vector3d particleAcceleration(0, -1, 0);

double RockRADIUS = 1;
Vector3d sphereCenter(0,0,-2);


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


void particlesGenerator()
{
        for (float i = -5; i<5; i+=ParticleDense) {
            particles.push_back( Particle(Vector3d(i,gauss(10, 1, 1),-10), Vector3d(0,0,gauss(1.5, 0.5, 1)), Vector4d(0,1,1,1), 1, 10, 0.5, false) );//position,velocity,color,mass,lifespan,pointsize,stopSign
    }

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
    particles.reserve(1000000);
    particlesGenerator();
        
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


bool detectSphereCollision(Vector3d& particlePosition, Vector3d& particleVelocity,  Vector3d& particlePositionNew  , Vector3d&  particleVelocityNew)
{
   
    Vector3d LVector = sphereCenter - particlePosition;
    double L = LVector.norm();
    Vector3d VDirection = (particlePositionNew - particlePosition).normalize();
    double tc = LVector * VDirection;
    if (tc < 0.0) {
        return  false;
    }
    double d = sqrt((L*L)-(tc*tc));
    if (d > RockRADIUS) {
        return false;
    }
    
    double t1c = sqrt(RockRADIUS*RockRADIUS-d*d);
    
    double t1 = tc - t1c;
    double t2 = tc + t1c;
    
    Vector3d collisionPoint = particlePosition + t1*VDirection;
    
    
    Vector3d normalVector = (collisionPoint - sphereCenter);
    double d1 = (particlePosition - collisionPoint)*normalVector;
    double d2 = (particlePositionNew - collisionPoint)*normalVector;
    if (d1*d2<0) {
        particlePositionNew = particlePositionNew - 1.7*d2*normalVector;
        Vector3d vn = (particleVelocity*normalVector)*normalVector;
        Vector3d vt = particleVelocity - vn;
        particleVelocityNew = -0.3*vn + 0.3*vt;
        it->setColor(Vector4d(1,1,1,1));
    }
    
    return true;
    
   
}

void detectPlaneCollision(Vector3d& particlePosition, Vector3d& particleVelocity,  Vector3d& particlePositionNew  , Vector3d&  particleVelocityNew)
{
    
        Vector3d p(0,0,0);
        Vector3d n(0,1,0);
        double d1 = (particlePosition - p) * n;
        double d2 = (particlePositionNew - p)*n;
        if (d1*d2<0) {
                 //std::cout<<"collsion"<<std::endl;
            particlePositionNew = particlePositionNew - 1.7*d2*n;
            Vector3d vn = (particleVelocity*n)*n;
            Vector3d vt = particleVelocity - vn;
            particleVelocityNew = -0.5*vn + 0.5*vt;
            it->setColor(Vector4d(1,1,1,1));
        }

         
    
}

void calculatePosition()
{
    Vector3d particlePositionNew, particleVelocityNew;
    for (it = particles.begin(); it!=particles.end(); ++it) {
        it->setLifeSpan(it->getLifeSpan()+1);
        if (it->getStopSign() == false) {
            particleVelocityNew = it->getVelocity() + particleAcceleration*hStep;
            particlePositionNew = it->getPosition() + it->getVelocity()*hStep;
            
            //cout<<"Velocity: "<<particleVelocityNew<<endl;
            //cout<<"Postion:  "<<particlePositionNew<<endl;
            
            if (particlePositionNew.y >-10) {
                Vector3d PTmp = it->getPosition();
                Vector3d VTmp = it->getVelocity();
                
                /*
                //planeDetect
                if (PTmp.y<3&&(PTmp.x<2&&PTmp.x>-2)&&(PTmp.z<0&&PTmp.z>-5))
                {
                    detectPlaneCollision(PTmp, VTmp, particlePositionNew, particleVelocityNew);
                }
                */
                
                //sphere
                if (PTmp.y>(sphereCenter.y-RockRADIUS)&&PTmp.y<(sphereCenter.y+RockRADIUS)
                    && PTmp.x>(sphereCenter.x-RockRADIUS)&&PTmp.x<(sphereCenter.x+RockRADIUS)
                    &&PTmp.z>(sphereCenter.z-RockRADIUS)&&PTmp.z<(sphereCenter.z+RockRADIUS)) {
                    
                    if(detectSphereCollision(PTmp, VTmp, particlePositionNew, particleVelocityNew))
                        cout<<"Collision: "<<endl;
                }
                
                
                it->setVelocity(particleVelocityNew);
                it->setPosition(particlePositionNew);
                
                
            }
            else
            {
                it->setStopSign(true);
            }
        }
        
        
        //Particles destroy  lifeSpan=300
        if (it->getLifeSpan()>300) {
            it = particles.erase(particles.begin(), particles.begin()+1000);
            //std::cout<<"delete"<<std::endl;
            --it;
        }
        
    }
    
    
    
    
   
}

void timeProc(int id)
{
    if (id == 1) {
            calculatePosition();
            glutPostRedisplay();
            particlesGenerator();
        //std::cout<<"size: "<<particles.size()<<std::endl;
        //std::cout<<"lifeSpan: "<<particles[0].getLifeSpan()<<std::endl;
            glutTimerFunc(automaticSpeed, timeProc, 1);

        
    }
    
}

void timeProc_clean(int id)
{
    if (id  == 2) {
        particles.clear();
        glutTimerFunc(cleanSpeed, timeProc_clean, 2);
    }
}

void handleKey(unsigned char key, int x, int y){
    switch(key){
        case 'a':
        case 'A':
            glutTimerFunc(automaticSpeed, timeProc, 1);
            //glutTimerFunc(cleanSpeed, timeProc_clean, 2);
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