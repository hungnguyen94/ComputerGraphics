#ifndef RAYTRACING_Hjdslkjfadjfasljf
#define RAYTRACING_Hjdslkjfadjfasljf
#include <vector>
#include "mesh.h"

//Welcome to your MAIN PROJECT...
//THIS IS THE MOST relevant code for you!
//this is an important file, raytracing.cpp is what you need to fill out
//In principle, you can do the entire project ONLY by working in these two files

extern Mesh MyMesh; //Main mesh
extern std::vector<Vec3Df> MyLightPositions;
extern Vec3Df MyCameraPosition; //currCamera
extern unsigned int WindowSize_X;//window resolution width
extern unsigned int WindowSize_Y;//window resolution height
extern unsigned int RayTracingResolutionX;  // largeur fenetre
extern unsigned int RayTracingResolutionY;  // largeur fenetre

//use this function for any preprocessing of the mesh.
void init();

//you can use this function to transform a click to an origin and destination
//the last two values will be changed. There is no need to define this function.
//it is defined elsewhere
void produceRay(int x_I, int y_I, Vec3Df & origin, Vec3Df & dest);

bool intersectRay( const Vec3Df & origin, const Vec3Df & dest, Vec3Df & hit, int & level, int & triangleIndex, Vec3Df & hitnormal);

bool intersect( const Vec3Df & origin, const Vec3Df & dest, const Triangle & triangle, Vec3Df & hit, float & distance, Vec3Df & hitnormal);

bool intersectPlane(const Vec3Df & origin, const Vec3Df & dest, const std::vector<Vec3Df> & plane, Vec3Df & hit, float & distance);

void shade( const Vec3Df & origin, const Vec3Df & dest, int & level, Vec3Df & hit, Vec3Df & color, int & triangleIndex, Vec3Df & hitnormal);

void computeDirectLight( Vec3Df lightPosition, Vec3Df hit, const int triangleIndex, Vec3Df & color, Vec3Df & hitnormal);

void computeReflectedLight( const Vec3Df & origin, const Vec3Df & dest, int & level, Vec3Df & hit, Vec3Df & color, int & triangleIndex, Vec3Df & hitnormal);

//your main function to rewrite
void performRayTracing(const Vec3Df & origin, const Vec3Df & dest, int &level, Vec3Df & color);

//a function to debug --- you can draw in OpenGL here
void yourDebugDraw();

//want keyboard interaction? Here it is...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination);

#endif
