//TODO: Zmień fonty


//*****************************************************************************
// hello.c - Simple hello world example.
//
// Maciej Kucia July 2013
//
// This is part of revision 1.0 of the EK-LM4F232 Firmware Package.
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "drivers/ili9341_240x320x262K.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
const int MAXH = 240;
const int MAXW = 320;


//REGION STRUCTS
struct vec3d
{
	float x, y, z, w;
};
typedef struct vec3d vec3d;

struct triangle
{
	vec3d p[3];
};
typedef struct triangle triangle;

//TODO: this is hack, replace this with vector or dynamic array
struct mesh
{
	triangle tris[2000];
    int counter;
};
typedef struct mesh mesh;

struct mat4x4
{
	float m[4][4];
};
typedef struct mat4x4 mat4x4;
//ENDREGION

//REGION FUNCTIONS
vec3d Vector_Add(vec3d *v1, vec3d *v2) {
    return (vec3d){v1->x + v2->x, v1->y + v2->y, v1->z + v2->z};
}

vec3d Vector_Sub(vec3d *v1, vec3d *v2) {
    return (vec3d){v1->x - v2->x, v1->y - v2->y, v1->z - v2->z};
}

vec3d Vector_Mul(vec3d *v1, float k) {
    return (vec3d){v1->x * k, v1->y * k, v1->z * k};
}

vec3d Vector_Div(vec3d *v1, float k) {
    return (vec3d){v1->x / k, v1->y / k, v1->z / k};
}

float Vector_DotProduct(vec3d *v1, vec3d *v2) {
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

float Vector_Length(vec3d *v) {
    return sqrtf(Vector_DotProduct(v, v));
}

vec3d Vector_Normalise(vec3d *v) {
    float l = Vector_Length(v);
    return (vec3d){v->x / l, v->y / l, v->z / l};
}

vec3d Vector_CrossProduct(vec3d *v1, vec3d *v2) {
    vec3d v;
    v.x = v1->y * v2->z - v1->z * v2->y;
    v.y = v1->z * v2->x - v1->x * v2->z;
    v.z = v1->x * v2->y - v1->y * v2->x;
    return v;
}

vec3d Vector_IntersectPlane(vec3d *plane_p, vec3d *plane_n, vec3d *lineStart, vec3d *lineEnd) {
    *plane_n = Vector_Normalise(plane_n);
    float plane_d = -Vector_DotProduct(plane_n, plane_p);
    float ad = Vector_DotProduct(lineStart, plane_n);
    float bd = Vector_DotProduct(lineEnd, plane_n);
    float t = (-plane_d - ad) / (bd - ad);
    vec3d lineStartToEnd = Vector_Sub(lineEnd, lineStart);
    vec3d lineToIntersect = Vector_Mul(&lineStartToEnd, t);
    return Vector_Add(lineStart, &lineToIntersect);
}

vec3d Matrix_MultiplyVector(mat4x4 *m, vec3d *i) {
    vec3d v;
    v.x = i->x * m->m[0][0] + i->y * m->m[1][0] + i->z * m->m[2][0] + i->w * m->m[3][0];
    v.y = i->x * m->m[0][1] + i->y * m->m[1][1] + i->z * m->m[2][1] + i->w * m->m[3][1];
    v.z = i->x * m->m[0][2] + i->y * m->m[1][2] + i->z * m->m[2][2] + i->w * m->m[3][2];
    v.w = i->x * m->m[0][3] + i->y * m->m[1][3] + i->z * m->m[2][3] + i->w * m->m[3][3];
    return v;
}

mat4x4 Matrix_MakeIdentity() {
    mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4x4 Matrix_MakeRotationX(float fAngleRad) {
    mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = cosf(fAngleRad);
    matrix.m[1][2] = sinf(fAngleRad);
    matrix.m[2][1] = -sinf(fAngleRad);
    matrix.m[2][2] = cosf(fAngleRad);
    matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4x4 Matrix_MakeRotationY(float fAngleRad) {
    mat4x4 matrix;
    matrix.m[0][0] = cosf(fAngleRad);
    matrix.m[0][2] = sinf(fAngleRad);
    matrix.m[2][0] = -sinf(fAngleRad);
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = cosf(fAngleRad);
    matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4x4 Matrix_MakeRotationZ(float fAngleRad) {
    mat4x4 matrix;
    matrix.m[0][0] = cosf(fAngleRad);
    matrix.m[0][1] = sinf(fAngleRad);
    matrix.m[1][0] = -sinf(fAngleRad);
    matrix.m[1][1] = cosf(fAngleRad);
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4x4 Matrix_MakeTranslation(float x, float y, float z) {
    mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    matrix.m[3][0] = x;
    matrix.m[3][1] = y;
    matrix.m[3][2] = z;
    return matrix;
}

mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar) {
    float fFovRad = 1.0f / tanf(fFovDegrees * 0.5f / 180.0f * 3.14159f);
    mat4x4 matrix;
    matrix.m[0][0] = fAspectRatio * fFovRad;
    matrix.m[1][1] = fFovRad;
    matrix.m[2][2] = fFar / (fFar - fNear);
    matrix.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matrix.m[2][3] = 1.0f;
    matrix.m[3][3] = 0.0f;
    return matrix;
}

mat4x4 Matrix_MultiplyMatrix(mat4x4 *m1, mat4x4 *m2) {
    mat4x4 matrix;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            matrix.m[r][c] = m1->m[r][0] * m2->m[0][c] + m1->m[r][1] * m2->m[1][c] + m1->m[r][2] * m2->m[2][c] + m1->m[r][3] * m2->m[3][c];
    return matrix;
}

mat4x4 Matrix_PointAt(vec3d *pos, vec3d *target, vec3d *up) {
    // Calculate new forward direction
    vec3d newForward = {target->x - pos->x, target->y - pos->y, target->z - pos->z, 1.0f};
    newForward = Vector_Normalise(&newForward);

    // Calculate new Up direction
    vec3d a = Vector_Mul(&newForward, Vector_DotProduct(up, &newForward));
    vec3d newUp = Vector_Sub(up, &a);
    newUp = Vector_Normalise(&newUp);

    // New Right direction is easy, its just cross product
    vec3d newRight = Vector_CrossProduct(&newUp, &newForward);

    // Construct Dimensioning and Translation Matrix	
    mat4x4 matrix;
    matrix.m[0][0] = newRight.x;	matrix.m[0][1] = newRight.y;	matrix.m[0][2] = newRight.z;	matrix.m[0][3] = 0.0f;
    matrix.m[1][0] = newUp.x;		matrix.m[1][1] = newUp.y;		matrix.m[1][2] = newUp.z;		matrix.m[1][3] = 0.0f;
    matrix.m[2][0] = newForward.x;	matrix.m[2][1] = newForward.y;	matrix.m[2][2] = newForward.z;	matrix.m[2][3] = 0.0f;
    matrix.m[3][0] = pos->x;		matrix.m[3][1] = pos->y;		matrix.m[3][2] = pos->z;		matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4x4 Matrix_QuickInverse(mat4x4 *m) {
    mat4x4 matrix;
    matrix.m[0][0] = m->m[0][0]; matrix.m[0][1] = m->m[1][0]; matrix.m[0][2] = m->m[2][0]; matrix.m[0][3] = 0.0f;
    matrix.m[1][0] = m->m[0][1]; matrix.m[1][1] = m->m[1][1]; matrix.m[1][2] = m->m[2][1]; matrix.m[1][3] = 0.0f;
    matrix.m[2][0] = m->m[0][2]; matrix.m[2][1] = m->m[1][2]; matrix.m[2][2] = m->m[2][2]; matrix.m[2][3] = 0.0f;
    matrix.m[3][0] = -(m->m[3][0] * matrix.m[0][0] + m->m[3][1] * matrix.m[1][0] + m->m[3][2] * matrix.m[2][0]);
    matrix.m[3][1] = -(m->m[3][0] * matrix.m[0][1] + m->m[3][1] * matrix.m[1][1] + m->m[3][2] * matrix.m[2][1]);
    matrix.m[3][2] = -(m->m[3][0] * matrix.m[0][2] + m->m[3][1] * matrix.m[1][2] + m->m[3][2] * matrix.m[2][2]);
    matrix.m[3][3] = 1.0f;
    return matrix;
}
float dist(vec3d *p, vec3d *plane_n, vec3d *plane_p) {
    vec3d n = Vector_Normalise(plane_n);
    return (n.x * p->x + n.y * p->y + n.z * p->z - Vector_DotProduct(&n, plane_p));
}

int Triangle_ClipAgainstPlane(vec3d plane_p, vec3d plane_n, triangle *in_tri, triangle *out_tri1, triangle *out_tri2) {
    // Make sure plane normal is indeed normal
    plane_n = Vector_Normalise(&plane_n);


    // Create two temporary storage arrays to classify points either side of the plane
    // If distance sign is positive, the point lies on the "inside" of the plane
    vec3d *inside_points[3];
    int nInsidePointCount = 0;
    vec3d *outside_points[3];
    int nOutsidePointCount = 0;

    // Get signed distance of each point in the triangle to the plane
    float d0 = dist(&in_tri->p[0], &plane_n, &plane_p);
    float d1 = dist(&in_tri->p[1], &plane_n, &plane_p);
    float d2 = dist(&in_tri->p[2], &plane_n, &plane_p);

    if (d0 >= 0) {
        inside_points[nInsidePointCount++] = &in_tri->p[0];
    } else {
        outside_points[nOutsidePointCount++] = &in_tri->p[0];
    }
    if (d1 >= 0) {
        inside_points[nInsidePointCount++] = &in_tri->p[1];
    } else {
        outside_points[nOutsidePointCount++] = &in_tri->p[1];
    }
    if (d2 >= 0) {
        inside_points[nInsidePointCount++] = &in_tri->p[2];
    } else {
        outside_points[nOutsidePointCount++] = &in_tri->p[2];
    }

    // Now classify triangle points and break the input triangle into
    // smaller output triangles if required. There are four possible
    // outcomes...

    if (nInsidePointCount == 0) {
        // All points lie on the outside of the plane, so clip the whole triangle
        // It ceases to exist
        return 0; // No returned triangles are valid
    }

    if (nInsidePointCount == 3) {
        // All points lie on the inside of the plane, so do nothing
        // and allow the triangle to simply pass through
        *out_tri1 = *in_tri;
        return 1; // Just the one returned original triangle is valid
    }

    if (nInsidePointCount == 1 && nOutsidePointCount == 2) {
        // Triangle should be clipped. As two points lie outside
        // the plane, the triangle simply becomes a smaller triangle

        // Copy appearance info to the new triangle
        //out_tri1->col = in_tri->col;
        //out_tri1->sym = in_tri->sym;

        // The inside point is valid, so keep that...
        out_tri1->p[0] = *inside_points[0];

        // But the two new points are at the locations where the 
        // original sides of the triangle (lines) intersect with the plane
        out_tri1->p[1] = Vector_IntersectPlane(&plane_p, &plane_n, inside_points[0], outside_points[0]);
        out_tri1->p[2] = Vector_IntersectPlane(&plane_p, &plane_n, inside_points[0], outside_points[1]);

        return 1; // Return the newly formed single triangle
    }

    if (nInsidePointCount == 2 && nOutsidePointCount == 1) {
        // Triangle should be clipped. As two points lie inside the plane,
        // the clipped triangle becomes a "quad". Fortunately, we can
        // represent a quad with two new triangles

        // Copy appearance info to new triangles
        //out_tri1->col = in_tri->col;
        //out_tri1->sym = in_tri->sym;

        //out_tri2->col = in_tri->col;
        //out_tri2->sym = in_tri->sym;

        // The first triangle consists of the two inside points and a new
        // point determined by the location where one side of the triangle
        // intersects with the plane
        out_tri1->p[0] = *inside_points[0];
        out_tri1->p[1] = *inside_points[1];
        out_tri1->p[2] = Vector_IntersectPlane(&plane_p, &plane_n, inside_points[0], outside_points[0]);

        // The second triangle is composed of one of the inside points, a
        // new point determined by the intersection of the other side of the 
        // triangle and the plane, and the newly created point above
        out_tri2->p[0] = *inside_points[1];
        out_tri2->p[1] = out_tri1->p[2];
        out_tri2->p[2] = Vector_IntersectPlane(&plane_p, &plane_n, inside_points[1], outside_points[0]);

        return 2; // Return
    }
}
void DrawTriangle(tContext *sContext, int x1, int y1, int x2, int y2, int x3, int y3){
    GrContextForegroundSet(sContext, ClrWhite);
    GrLineDraw(sContext, x1, y1, x2, y2);
    GrLineDraw(sContext, x2, y2, x3, y3);
    GrLineDraw(sContext, x3, y3, x1, y1);
}


void _clear_screen(tContext *sContext){
    tRectangle sRect;
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = 320;
    sRect.i16YMax = 240;

    GrContextForegroundSet(sContext, ClrBlack);
    GrRectFill(sContext, &sRect);
}

uint32_t clacRGB(int R, int G, int B){
    uint32_t result = R*65536 + G*256 + B;
    return result;
}
//ENDREGION



int main(void)
{
tContext sContext;


//
// Enable lazy stacking for interrupt handlers. This allows floating-point
// instructions to be used within interrupt handlers, but at the expense of
// extra stack usage.
//
ROM_FPULazyStackingEnable();
//
// Set the clocking to run directly from the crystal.
//
ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
//
// Initialize the display driver.
//
ILI9341_240x320x262K_Init();
//
// Initialize the graphics context.
//
GrContextInit(&sContext, &g_sILI9341_240x320x262K);


uint32_t red = clacRGB(255,52,25);
uint32_t semiDarkRed = clacRGB(240,29,0);
uint32_t darkRed = clacRGB(180,21,0);

mesh meshCube;
meshCube.counter = 0;
mat4x4 matProj;
float fTheta;

meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f,1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 0.0f,1.0f, 1.0f, 1.0f, 0.0f,1.0f, 1.0f, 0.0f, 0.0f, 1.0f}};

// EAST
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f}};

// NORTH
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f}};

// WEST
meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

// TOP
meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f}};

// BOTTOM
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f}};



matProj = Matrix_MakeProjection(90.0f, (float)MAXH / (float)MAXW, 0.1f, 1000.0f);
float fElapsedTime = 0.1;
vec3d vCamera = {0, 0 ,0};

int fps = 10000000;
int fpsI = 0;
while(true){
    if(fpsI == fps){
    _clear_screen(&sContext);
    mat4x4 matRotZ, matRotX, matTrans, matWorld;
	fTheta += 1.0f * fElapsedTime;

    matRotZ = Matrix_MakeRotationZ(fTheta);
    matRotX = Matrix_MakeRotationX(fTheta);
    matTrans = Matrix_MakeTranslation(0.0f, 0.0f, 16.0f);

    matWorld = Matrix_MakeIdentity();
    matWorld = Matrix_MultiplyMatrix(&matRotZ, &matRotX);
    matWorld = Matrix_MultiplyMatrix(&matWorld, &matTrans);

    for (int i = 0; i<= meshCube.counter; i++)
		{
            triangle tri = meshCube.tris[i];

			triangle triProjected, triTransformed;

			triTransformed.p[0] = Matrix_MultiplyVector(&matWorld, &tri.p[0]);
            triTransformed.p[1] = Matrix_MultiplyVector(&matWorld, &tri.p[1]);
            triTransformed.p[2] = Matrix_MultiplyVector(&matWorld, &tri.p[2]);

            // Calculate triangle Normal
			vec3d normal, line1, line2;

			// Get lines either side of triangle
			line1 = Vector_Sub(&triTransformed.p[1], &triTransformed.p[0]);
			line2 = Vector_Sub(&triTransformed.p[2], &triTransformed.p[0]);

			// Take cross product of lines to get normal to triangle surface
			normal = Vector_CrossProduct(&line1, &line2);

			// You normally need to normalise a normal!
			normal = Vector_Normalise(&normal);
			normal.z = line1.x * line2.y - line1.y * line2.x;

			// It's normally normal to normalise the normal
			vec3d vCameraRay = Vector_Sub(&triTransformed.p[0], &vCamera);

			// If ray is aligned with normal, then triangle is visible
			if (Vector_DotProduct(&normal, &vCameraRay) < 0.0f)
			{
                // Project triangles from 3D --> 2D
                triProjected.p[0] = Matrix_MultiplyVector(&matProj, &triTransformed.p[0]);
                triProjected.p[1] = Matrix_MultiplyVector(&matProj, &triTransformed.p[1]);
                triProjected.p[2] = Matrix_MultiplyVector(&matProj, &triTransformed.p[2]);

                triProjected.p[0] = Vector_Div(&triProjected.p[0], triTransformed.p[0].w);
                triProjected.p[1] = Vector_Div(&triProjected.p[1], triTransformed.p[1].w);
                triProjected.p[2] = Vector_Div(&triProjected.p[2], triTransformed.p[2].w);

                // Scale into view
                vec3d vOffsetView = { 1,1,0 };
                triProjected.p[0] = Vector_Add(&triProjected.p[0], &vOffsetView);
                triProjected.p[1] = Vector_Add(&triProjected.p[1], &vOffsetView);
                triProjected.p[2] = Vector_Add(&triProjected.p[2], &vOffsetView);
                triProjected.p[0].x *= 0.5f * (float)MAXW;
                triProjected.p[0].y *= 0.5f * (float)MAXH;
                triProjected.p[1].x *= 0.5f * (float)MAXW;
                triProjected.p[1].y *= 0.5f * (float)MAXH;
                triProjected.p[2].x *= 0.5f * (float)MAXW;
                triProjected.p[2].y *= 0.5f * (float)MAXH;

                // Rasterize triangle
                DrawTriangle(&sContext, triProjected.p[0].x, triProjected.p[0].y,
                    triProjected.p[1].x, triProjected.p[1].y,
                    triProjected.p[2].x, triProjected.p[2].y);
            }

		}
        fpsI = 0;
    }
    fpsI++;

}

GrFlush(&sContext);


}