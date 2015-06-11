#include <stdio.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
#include "raytracing.h"
#include <cmath>
#include <Vec3D.h>


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;
extern unsigned int WindowSize_X;
extern unsigned int WindowSize_Y;


//use this function for any preprocessing of the mesh.
void init()
{
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj",
	//otherwise the application will not load properly
    MyMesh.loadMesh("dodgeColorTest.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

void Trace(int level, const Vec3Df ray, Vec3Df &color) {
    Vec3Df hit;

    if( Intersect( level, ray, &hit) )
        //Shade( level, hit, &color);
    else
        color = glClearColor(0.0, 0.0, 0.0, 0.0);
}

Vec3Df Intersect(int level, const Vec3Df ray, float max, Vec3Df &hit) {
    Vec3Df v0;
    Vec3Df v1;
    Vec3Df v2;

	for(unsigned int i = 0; i < MyMesh.triangles.size(); i++){
		v0 = MyMesh.vertices[MyMesh.triangles[i].v[0]].p;
		v1 = MyMesh.vertices[MyMesh.triangles[i].v[1]].p;
		v2 = MyMesh.vertices[MyMesh.triangles[i].v[2]].p;
	}

    Vec3Df crossP1 = Vec3D::crossProduct( (v0-v2), (v1-v2));
    Vec3Df crossP2 = Vec3D::crossP1.getLength();

    // a normal to the plane
	Vec3Df n = crossP1 / crossP2;

    // d equals the projection of v0 onto n
    Vec3Df d = Vec3D::projectOn(v0 , n);

    return v0;
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
    Vec3Df color;
	Vec3Df ray = (dest - origin);

	ray[0] = origin[0] - dest[0];
	ray[1] = origin[1] - dest[1];
	ray[2] = origin[2] - dest[2];

	Trace( 0, ray, &color);

    	for(unsigned int y = 0; y < WindowSize_Y; ++y ){
		for(unsigned int x = 0; x < WindowSize_X; ++x){
			//Trace( 0, ray, &color );
			//PutPixel( x, y, color );
		}
	}

	return Vec3Df(dest[0],dest[1],dest[2]);
}

void yourDebugDraw()
{
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();

	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (int i=0;i<MyLightPositions.size();++i)
		glVertex3fv(MyLightPositions[i].pointer());
	glEnd();
	glPopAttrib();//restore all GL attributes
	//The Attrib commands maintain the state.
	//e.g., even though inside the two calls, we set
	//the color to white, it will be reset to the previous
	//state after the pop.


	//as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0,1,1);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(0,0,1);
	glVertex3f(testRayDestination[0], testRayDestination[1], testRayDestination[2]);
	glEnd();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3fv(MyLightPositions[0].pointer());
	glEnd();
	glPopAttrib();

	//draw whatever else you want...
	////glutSolidSphere(1,10,10);
	////allows you to draw a sphere at the origin.
	////using a glTranslate, it can be shifted to whereever you want
	////if you produce a sphere renderer, this
	////triangulated sphere is nice for the preview
}


//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved:
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering.
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray.
//    It then stores the result in an image "result.ppm".
//    Initially, this function is fast (performRaytracing simply returns
//    the target of the ray - see the code above), but once you replaced
//    this function and raytracing is in place, it might take a
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin=rayOrigin;
	testRayDestination=rayDestination;

	// do here, whatever you want with the keyboard input t.

	//...


	std::cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<std::endl;
}
