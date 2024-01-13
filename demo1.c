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
	float x, y, z;
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
void MultiplyMatrixVector(vec3d* i, vec3d* o, mat4x4* m)
{
    o->x = i->x * m->m[0][0] + i->y * m->m[1][0] + i->z * m->m[2][0] + m->m[3][0];
    o->y = i->x * m->m[0][1] + i->y * m->m[1][1] + i->z * m->m[2][1] + m->m[3][1];
    o->z = i->x * m->m[0][2] + i->y * m->m[1][2] + i->z * m->m[2][2] + m->m[3][2];
    float w = i->x * m->m[0][3] + i->y * m->m[1][3] + i->z * m->m[2][3] + m->m[3][3];

    if (w != 0.0f)
    {
        o->x /= w; o->y /= w; o->z /= w;
    }
}

void DrawTriangle(tContext *sContext, int x1, int y1, int x2, int y2, int x3, int y3){
    GrContextForegroundSet(sContext, ClrWhite);
    GrLineDraw(sContext, x1, y1, x2, y2);
    GrLineDraw(sContext, x2, y2, x3, y3);
    GrLineDraw(sContext, x3, y3, x1, y1);
}


void FillTriangle(tContext *sContext, int x1, int y1, int x2, int y2, int x3, int y3) {
    float d1 = (float) sqrt((pow((y2-y1),2))+(pow((x2-x1),2)));
    float d2 = (float) sqrt((pow((y3-y2),2))+(pow((x3-x2),2)));
    float d3 = (float) sqrt((pow((y1-y3),2))+(pow((x1-x3),2)));
    GrContextForegroundSet(sContext, ClrWhite);

    float x_start = 0;
    float y_start = 0;
    float x_end = 0;
    float yJump = 0;
    float x_top = 0;
    float y_top = 0;

    if(((d1<d2)||(d1=d2))&&((d1<d2)||(d1=d2))){
        x_start = (float)x2;
        y_start = (float)y2;
        x_end = (float)x1;
        yJump = (float)(y2-y1)/(x2-x1)/1;
        x_top = (float)x3;
        y_top = (float)y3;
    } //the first side is the shortest
    else if((d2<d3)||(d2=d3)){
        x_start = (float)x3;
        y_start = (float)y3;
        x_end = (float)x2;
        yJump = (float)(y3-y2)/(x3-x2)/1;
        x_top = (float)x1;
        y_top = (float)y1;
    } //the second side is the shortest
    else{
        x_start = (float)x1;
        y_start = (float)y1;
        x_end = (float)x3;
        yJump = (float)(y1-y3)/(x1-x3)/1;
        x_top = (float)x2;
        y_top = (float)y2;
    } // the third side is shortest
    while(x_start>=x_end){
        GrLineDraw(sContext, x_top, y_top, x_start, y_start);
        x_start-=1;
        y_start-=yJump;
    }
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

//RENDERER
mesh meshCube;
meshCube.counter = 0;
mat4x4 matProj;
float fTheta;
// SOUTH
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f}};

    // EAST
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f}};

    // NORTH
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f}};

    // WEST
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}};

    // TOP
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f}};

    // BOTTOM
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}};
    meshCube.tris[meshCube.counter++] = (struct triangle){{1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}};


float fNear = 0.1f;
float fFar = 1000.0f;
float fFov = 90.0f;
float fAspectRatio = (float)MAXH / (float)MAXW;
float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);
matProj.m[0][0] = fAspectRatio * fFovRad;
matProj.m[1][1] = fFovRad;
matProj.m[2][2] = fFar / (fFar - fNear);
matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
matProj.m[2][3] = 1.0f;
matProj.m[3][3] = 0.0f;

vec3d vCamera = {0, 0 ,0};

float fElapsedTime = 0.1;

int fps = 10000000;
int fpsI = 0;
while(true){
    if(fpsI == fps){
        _clear_screen(&sContext);
    mat4x4 matRotZ, matRotX;
	fTheta += 1.0f * fElapsedTime;

    matRotZ.m[0][0] = cosf(fTheta);
    matRotZ.m[0][1] = sinf(fTheta);
    matRotZ.m[1][0] = -sinf(fTheta);
    matRotZ.m[1][1] = cosf(fTheta);
    matRotZ.m[2][2] = 1;
    matRotZ.m[3][3] = 1;

    matRotX.m[0][0] = 1;
    matRotX.m[1][1] = cosf(fTheta * 0.5f);
    matRotX.m[1][2] = sinf(fTheta * 0.5f);
    matRotX.m[2][1] = -sinf(fTheta * 0.5f);
    matRotX.m[2][2] = cosf(fTheta * 0.5f);
    matRotX.m[3][3] = 1;

    for (int i = 0; i<= meshCube.counter; i++)
		{
            triangle tri = meshCube.tris[i];

			triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

			// Rotate in Z-Axis
			MultiplyMatrixVector(&tri.p[0], &triRotatedZ.p[0], &matRotZ);
			MultiplyMatrixVector(&tri.p[1], &triRotatedZ.p[1],& matRotZ);
			MultiplyMatrixVector(&tri.p[2], &triRotatedZ.p[2], &matRotZ);

			// Rotate in X-Axis
			MultiplyMatrixVector(&triRotatedZ.p[0], &triRotatedZX.p[0], &matRotX);
			MultiplyMatrixVector(&triRotatedZ.p[1], &triRotatedZX.p[1], &matRotX);
			MultiplyMatrixVector(&triRotatedZ.p[2], &triRotatedZX.p[2], &matRotX);

			// Offset into the screen
			triTranslated = triRotatedZX;
			triTranslated.p[0].z = triRotatedZX.p[0].z + 3.0f;
			triTranslated.p[1].z = triRotatedZX.p[1].z + 3.0f;
			triTranslated.p[2].z = triRotatedZX.p[2].z + 3.0f;


            // Use Cross-Product to get surface normal
			vec3d normal, line1, line2;
			line1.x = triTranslated.p[1].x - triTranslated.p[0].x;
			line1.y = triTranslated.p[1].y - triTranslated.p[0].y;
			line1.z = triTranslated.p[1].z - triTranslated.p[0].z;

			line2.x = triTranslated.p[2].x - triTranslated.p[0].x;
			line2.y = triTranslated.p[2].y - triTranslated.p[0].y;
			line2.z = triTranslated.p[2].z - triTranslated.p[0].z;

			normal.x = line1.y * line2.z - line1.z * line2.y;
			normal.y = line1.z * line2.x - line1.x * line2.z;
			normal.z = line1.x * line2.y - line1.y * line2.x;

			// It's normally normal to normalise the normal
			float l = sqrtf(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
			normal.x /= l; normal.y /= l; normal.z /= l;

            if(normal.x * (triTranslated.p[0].x - vCamera.x) + 
			   normal.y * (triTranslated.p[0].y - vCamera.y) +
			   normal.z * (triTranslated.p[0].z - vCamera.z) < 0.0f){
                // Project triangles from 3D --> 2D
                MultiplyMatrixVector(&triTranslated.p[0], &triProjected.p[0], &matProj);
                MultiplyMatrixVector(&triTranslated.p[1], &triProjected.p[1], &matProj);
                MultiplyMatrixVector(&triTranslated.p[2], &triProjected.p[2], &matProj);

                // Scale into view
                triProjected.p[0].x += 1.0f; triProjected.p[0].y += 1.0f;
                triProjected.p[1].x += 1.0f; triProjected.p[1].y += 1.0f;
                triProjected.p[2].x += 1.0f; triProjected.p[2].y += 1.0f;
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