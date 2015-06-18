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


//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
Vec3Df testRayOrigin;
Vec3Df testRayDestination;


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
	// MyMesh.loadMesh("dodgeColorTest.obj", true);
    MyMesh.loadMesh("cube.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	float hit;
	Vec3Df color;
	int level = 0;
	int max = 3;
	int triangleIndex = 0;
	if( intersectRay(origin, dest, hit, level, max, triangleIndex) )
	{
		std::cout << "Intersection distance " << hit << " at index " << triangleIndex <<std::endl;
		shade( level, hit, color, triangleIndex );
	}
	else
	{
		color = Vec3Df(0,0,0);
	}
	return color;
}

bool intersectRay( const Vec3Df & origin, const Vec3Df & dest, float & hit, int & level, const int & max, int & triangleIndex)
{
    float currDistance = 9999.f;
    float distance = 0.f;
    
    for(unsigned int i = 0; i < MyMesh.triangles.size(); i++)
    {
        float distance = intersect(origin, dest, MyMesh.triangles[i]);
        if( distance != 0 && distance < currDistance )
        {
        	currDistance = distance;
            triangleIndex = i;
        }
    }

    if( currDistance < 9999.f ) {
    	hit = currDistance;
    	return true;
    }
    else {
    	hit = 9999.f;
    	return false;
    }

}

float intersect( const Vec3Df & origin, const Vec3Df & dest, Triangle & triangle )
{
    Vec3Df edge0 = MyMesh.vertices[triangle.v[1]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df edge1 = MyMesh.vertices[triangle.v[2]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df n = Vec3Df::crossProduct (dest, edge1);
    float det = Vec3Df::dotProduct(edge0, n);
    
    if(det == 0)
        return 0;
    
    Vec3Df distanceToOrigin = origin - MyMesh.vertices[triangle.v[0]].p;
    float u = Vec3Df::dotProduct(distanceToOrigin, n) / det;
    if(u < 0.f || u > 1.f)
        return 0;
    //std::cout << u << std::endl;
    
    Vec3Df Q = Vec3Df::crossProduct(distanceToOrigin, edge0);
    float v = Vec3Df::dotProduct(dest, Q) / det;
    if(v < 0.f || u + v > 1.f)
    {
        return 0;
    }
    
    float distance = Vec3Df::dotProduct(edge1, Q) / det;
    
    if(distance < 0)
        return 0;
    
    //std::cout << "intersection: " << distance << "\n" << std::endl;
    //std::cout << "dotproduct edge0 & cross: " << dot << "\n" << std::endl;
    //std::cout << "vector0: " << edge0 << " \nvector1: " << edge1 << " \nn: " << n << "\n\n" << std::endl;
    return distance;
}

void shade( int & level, float & hit, Vec3Df & color, int & triangleIndex) {
	level++;
	Material material = MyMesh.materials[MyMesh.triangleMaterials[triangleIndex]];
	color = material.Kd();
	std::cout << "Color: \n" << "ka: "<< material.Ka() << "\n kd: " << material.Kd() << "\nks" <<material.Ks() << std::endl;
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
