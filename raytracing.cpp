#include <stdio.h>
#include <math.h>
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

// Set to true when you want printed info
const bool verbose = false;

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
	//MyMesh.loadMesh("reflectionTest.obj", true);
	//MyMesh.loadMesh("dodgeColorTest.obj", true);
	//MyMesh.loadMesh("macbook pro.obj", true);
	//MyMesh.loadMesh("CoffeeTable.obj", true);
	//MyMesh.loadMesh("cube.obj", true);
	//MyMesh.loadMesh("capsule.obj", true);
	//MyMesh.loadMesh("Rock1.obj", true);
	MyMesh.loadMesh("sphereonplane.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

//return the color of your pixel.
void performRayTracing(const Vec3Df & origin, const Vec3Df & dest, int &level, Vec3Df & color)
{
	// Stop if the maxlevel is reached
	if(level <= 0)
		return;

	Vec3Df hit;
	int triangleIndex = 0;
	if( intersectRay(origin, dest, hit, --level, triangleIndex) )
	{
		if(verbose)
			std::cout << "\nIntersection at " << hit << " at triangleIndex " << triangleIndex << std::endl;
		// Calculate the color of the intersected triangle.
		shade( origin, dest, --level, hit, color, triangleIndex );
	}
	else
	{
		// Return the background color if there's no intersection.
		//if(color == Vec3Df(0.f,0.f,0.f))
			//color += Vec3Df(0.4f,0.4f,0.4f);
	}
	return;
}

bool intersectRay( const Vec3Df & origin, const Vec3Df & dest, Vec3Df & hit, int & level, int & triangleIndex)
{
    float currDistance = 9999.f;
    Vec3Df intersectionPoint;
    float distance = 0.f;
    
    // Intersection with triangles
    for(unsigned int i = 0; i < MyMesh.triangles.size(); i++)
    {
        if( intersect(origin, dest, MyMesh.triangles[i], intersectionPoint, distance) & (distance < currDistance) )
        {
        	currDistance = distance;
            triangleIndex = i;
            hit = intersectionPoint;
            return true;
        }
    }
    return intersected;
    // Intersection with planes

}

bool intersect2( const Vec3Df & origin, const Vec3Df & dest, const Triangle & triangle, Vec3Df & hit, float & t )
{
    // Compute the normal plane.
	Vec3Df v0v1 = MyMesh.vertices[triangle.v[1]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df v0v2 = MyMesh.vertices[triangle.v[2]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df normal = Vec3Df::crossProduct (v0v1, v0v2);
	//std::cout << "normal: " << normal << "\nnormal2: " << normal2 << std::endl;

    //float area2 = normal.getLength();

    // If determinant == 0 then ray and plane are parallel
    // And no intersection takes place.
    float NdotR = Vec3Df::dotProduct(normal, -dest);
    //std::cout << "NdotR: " << NdotR << std::endl;
    if (NdotR < 0.1f && NdotR > -0.1f)
    	return false;

    // Compute distance from the origin to the plane.
    float d = Vec3Df::dotProduct(normal, MyMesh.vertices[triangle.v[0]].p);
    //std::cout << "d: " << d << std::endl;
    // Compute the distance from the origin to the intersection point.
    t = (Vec3Df::dotProduct(normal, origin) + d) / (NdotR);
    //std::cout << "t: " << t << std::endl;
    if(t < 0)
    	return false;

    // Compute intersection point
    // intersectPoint = origin + distanceIntersectPoint * directionRay
    hit = origin + t * dest;

/*    // Test whether point is inside the triangle.
    Vec3Df edge0 = MyMesh.vertices[triangle.v[1]].p - MyMesh.vertices[triangle.v[0]].p;
    Vec3Df edge1 = MyMesh.vertices[triangle.v[2]].p - MyMesh.vertices[triangle.v[1]].p;
    Vec3Df edge2 = MyMesh.vertices[triangle.v[0]].p - MyMesh.vertices[triangle.v[2]].p;
    Vec3Df pv0 = hit - MyMesh.vertices[triangle.v[0]].p;
    Vec3Df pv1 = hit - MyMesh.vertices[triangle.v[1]].p;
    Vec3Df pv2 = hit - MyMesh.vertices[triangle.v[2]].p;
    float dot0 = Vec3Df::dotProduct(normal, Vec3Df::crossProduct(edge0, pv0));
    float dot1 = 1;//Vec3Df::dotProduct(normal, Vec3Df::crossProduct(edge1, pv1));
    float dot2 = Vec3Df::dotProduct(normal, Vec3Df::crossProduct(edge2, pv2));
   // std::cout << "dot0: " << dot0 << "\ndot1: " << dot1 << "\ndot2: " << dot2 << std::endl;
    if( dot0 > 0 || dot1 > 0 || dot2 > 0 )
    	return false;*/
    return true;
}

bool intersect( const Vec3Df & origin, const Vec3Df & dest, const Triangle & triangle, Vec3Df & hit, float & distance )
{
    // Compute the normal plane.
	Vec3Df v0v1 = MyMesh.vertices[triangle.v[1]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df v0v2 = MyMesh.vertices[triangle.v[2]].p -  MyMesh.vertices[triangle.v[0]].p;
    Vec3Df pvec = Vec3Df::crossProduct(dest, v0v2);

    // If determinant == 0 then no intersection takes place.
    float determinent = Vec3Df::dotProduct(v0v1, pvec);
    //std::cout << "NdotR: " << NdotR << std::endl;
    if (determinent == 0.f)
    	return false;

    float invDeterminent = 1 / determinent;

    Vec3Df tvec = origin - MyMesh.vertices[triangle.v[0]].p;
    float u = Vec3Df::dotProduct(tvec, pvec) * invDeterminent;
    if(u < 0.f || u > 1.f)
    	return false;

    Vec3Df qvec = Vec3Df::crossProduct(tvec, v0v1);
    float v = Vec3Df::dotProduct(dest, qvec) * invDeterminent;
    if(v < 0.f || u + v > 1.f)
    	return false;

    distance = Vec3Df::dotProduct(v0v2, qvec) * invDeterminent;
    if(distance < 0)
    	return false;

    // Compute intersection point
    // intersectPoint = origin + distanceIntersectPoint * directionRay
    hit = origin + distance * dest;

    return true;
}

void shade( const Vec3Df & origin, const Vec3Df & dest, int & level, Vec3Df & hit, Vec3Df & color, int & triangleIndex) {
	for (unsigned int i = 0; i < MyLightPositions.size(); ++i)
	{
		computeDirectLight(MyLightPositions[i], hit, triangleIndex, color);
		if(verbose)
			std::cout << "material illum: " << MyMesh.materials[MyMesh.triangleMaterials[triangleIndex]].illum() << std::endl;
//		if(MyMesh.materials[MyMesh.triangleMaterials[triangleIndex]].illum() == 2) {
//			computeReflectedLight(origin, dest, level, hit, color, triangleIndex);
//		}
	}

}

void computeReflectedLight( const Vec3Df & origin, const Vec3Df & dest, int & level, Vec3Df & hit, Vec3Df & color, int & triangleIndex  )
{
	//Calculate normal of triangle
	Triangle triangle3d = MyMesh.triangles[triangleIndex];
//	Vec3Df edge0 = MyMesh.vertices[triangle3d.v[1]].p -  MyMesh.vertices[triangle3d.v[0]].p;
//	Vec3Df edge1 = MyMesh.vertices[triangle3d.v[2]].p -  MyMesh.vertices[triangle3d.v[0]].p;
//	Vec3Df normal = Vec3Df::crossProduct(edge0, edge1);
	Vec3Df normal = MyMesh.vertices[triangle3d.v[0]].n;
	normal.normalize();

	float reflected = 2.0f * Vec3Df::dotProduct(MyCameraPosition, normal);
	Vec3Df newDest = dest - reflected * normal;
	Vec3Df newHit;
	int newTriangleIndex;
	std::cout << "reflected angle: " << reflected << std::endl;
	performRayTracing(hit, newDest, --level, color);

}

void computeDirectLight( Vec3Df lightPosition, Vec3Df hit, const int triangleIndex, Vec3Df & color )
{
	Vec3Df lightColor = Vec3Df(1.f, 1.f, 1.f);
	float lightIntensity = 20.f;
	Material material = MyMesh.materials[MyMesh.triangleMaterials[triangleIndex]];
	if(verbose)
		std::cout << "\nMaterial: \n" << "ka: "<< material.Ka() << "\nkd: " << material.Kd() << "\nks: " <<material.Ks() << "\nns: " << material.Ns() << "\nni: " << material.Ni() << std::endl;

	Triangle triangle3d = MyMesh.triangles[triangleIndex];

	Vec3Df lightDir = lightPosition - hit;
	float distance = lightDir.getLength();
	lightDir.normalize();
	distance = distance * distance;

	//Calculate normal of triangle
//	Vec3Df edge0 = MyMesh.vertices[triangle3d.v[1]].p -  MyMesh.vertices[triangle3d.v[0]].p;
//	Vec3Df edge1 = MyMesh.vertices[triangle3d.v[2]].p -  MyMesh.vertices[triangle3d.v[0]].p;
//	Vec3Df normal = Vec3Df::crossProduct(edge0, edge1);
	Vec3Df normal = MyMesh.vertices[triangle3d.v[0]].n;
	normal.normalize();

	// cosine of the angle between the normal and the lightDir.
	float NdotL = Vec3Df::dotProduct(normal, lightDir);
	// Can't be negative, if so set to 0.
	if(NdotL < 0) {
		NdotL = 0.f;
	}
	// D = Id*Kd*cos(a)
	Vec3Df diffuse = lightColor * lightIntensity * NdotL * material.Kd();
	// A = Ia*Ka
	Vec3Df ambient = lightColor * lightIntensity * material.Ka();

	// blinn phong
	Vec3Df halfDir = lightDir + MyCameraPosition;
	halfDir.normalize();
	float specAngle = Vec3Df::dotProduct(halfDir, normal);
	Vec3Df specular = Vec3Df(0, 0, 0);
	if(specAngle > 0 && material.Ks() != Vec3Df(0,0,0)) {
		double specTerm = std::pow(specAngle, 2.0f* material.Ns());
		specular = specTerm * material.Ks();
		if(verbose)
			std::cout << "\nSpecular angle: " << specAngle << "\nSpecTerm: " << specTerm << std::endl;
	}
	color += ambient + (diffuse + specular) / distance;
	if(verbose)
		std::cout << "\nLight angle: " << NdotL << "\nDiffuse: " << diffuse
					<< "\nAmbient: " << ambient << "\nSpecular: " << specular
					<< "\nColor: " << color << std::endl;
}

void yourDebugDraw()
{
	glColor3f(0,0,1);
	glBegin(GL_QUADS); // Start drawing a quad primitive
		glVertex3f(-1.0, -.4, 1.0);
			//br
			glVertex3f(1.0, -.4, 1.0);
			//tr
			glVertex3f(1.0, -.4, -1.0);
			//tl
			glVertex3f(-1.0, -.4, -1.0);
			glEnd();

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
	
	// WASDQE keys are used for moving the light source
	switch (t)
	{
		case '/':
		{
			// Change lightsource clockwise
			Vec3Df temp = MyLightPositions.front();
			MyLightPositions.push_back(temp);
			MyLightPositions.erase(MyLightPositions.begin());

		}
		break;
		case '*':
		{
			// Change lightsource backwise
			Vec3Df temp = MyLightPositions.back();
			MyLightPositions.pop_back();
			MyLightPositions.insert(MyLightPositions.begin(), temp);
		}
		break;

		case 'a':
		{
			// Move lightsource to the left
			Vec3Df temp = MyLightPositions.back();
			temp[0] += 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;
		}
		break;
		case 's':
		{
			// Move lightsource backwards
			Vec3Df temp = MyLightPositions.back();
			temp[2] -= 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;

		}
		break;
		case 'd':
		{
			// Move lightsource to the right
			Vec3Df temp = MyLightPositions.back();
			temp[0] -= 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;

		}
		break;
		case 'w':
		{
			// Move lightsource forwards
			Vec3Df temp = MyLightPositions.back();
			temp[2] += 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;

		}
		break;
		case 'q':
		{
			// Move lightsource downwards
			Vec3Df temp = MyLightPositions.back();
			temp[1] -= 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;

		}
		break;
		case 'e':
		{
			// Move lightsource upwards
			Vec3Df temp = MyLightPositions.back();
			temp[1] += 0.1;
			MyLightPositions[MyLightPositions.size() - 1] = temp;
		}
		break;
	}
}
