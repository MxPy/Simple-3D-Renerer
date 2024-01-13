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
#define GPIO_PINS_ALL GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7


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

void createCube(mesh* meshCube, float x, float y, float z, float size) {
    // SOUTH
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y, z, 1, x, y + size, z, 1, x + size, y + size, z, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y, z, 1, x + size, y + size, z, 1, x + size, y, z, 1}};

    // EAST
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z, 1, x + size, y + size, z, 1, x + size, y + size, z + size, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z, 1, x + size, y + size, z + size, 1, x + size, y, z + size, 1}};

    // NORTH
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z + size, 1, x + size, y + size, z, 1 + size, x, y + size, z + size, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z + size, 1, x, y + size, z + size, 1, x, y, z + size, 1}};

    // WEST
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y, z + size, 1, x, y + size, z + size, 1, x, y + size, z, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y, z + size, 1, x, y + size, z, 1, x, y, z, 1}};

    // TOP
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y + size, z, 1, x, y + size, z + size, 1, x + size, y + size, z + size, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x, y + size, z, 1, x + size, y + size, z + size, 1, x + size, y + size, z, 1}};

    // BOTTOM
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z + size, 1, x, y, z + size, 1, x, y, z, 1}};
    meshCube->tris[meshCube->counter++] = (struct triangle){{x + size, y, z + size, 1, x, y, z, 1, x + size, y, z, 1}};
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

SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOJ);
GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PINS_ALL);



uint32_t red = clacRGB(255,52,25);
uint32_t semiDarkRed = clacRGB(240,29,0);
uint32_t darkRed = clacRGB(180,21,0);

//RENDERER
mesh meshCube;
meshCube.counter = 0;
mat4x4 matProj;
float fThetaZ;
float fThetaX;
float fThetaY;


createCube(&meshCube, 0.0f, 0.0f, 1.0f, 1.0f);

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
vec3d vLookDir = {0, 0 ,1};

float fElapsedTime = 0.1;

int fps = 10000000;
int fpsI = 0;
float x = 0.0f;
float y = 0.0f;
float z = 0.0f;
float yaw = 0;
while(true){
    
    if(fpsI == fps){
        vec3d vForward = Vector_Mul(&vLookDir, 4.0f * fElapsedTime);
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_7) == GPIO_PIN_7){ yaw -= 2.0f * fElapsedTime;}
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_6) == GPIO_PIN_6){ yaw += 2.0f * fElapsedTime;}
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_5) == GPIO_PIN_5){ vCamera = Vector_Add(&vCamera, &vForward);}
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_4) == GPIO_PIN_4){ vCamera = Vector_Sub(&vCamera, &vForward);}
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_3) == GPIO_PIN_3){ vCamera.y += 1.0f * fElapsedTime;}
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_2) == GPIO_PIN_2){ vCamera.y -= 1.0f * fElapsedTime;}
        _clear_screen(&sContext);

        vec3d vUp = {0,1,0};
        vec3d vTarget = {0,0,1};
        mat4x4 matCameraRot;
        matCameraRot.m[0][0] = cosf(yaw);
        matCameraRot.m[0][2] = sinf(yaw);
        matCameraRot.m[2][0] = -sinf(yaw);
        matCameraRot.m[1][1] = 1.0f;
        matCameraRot.m[2][2] = cosf(yaw);
        matCameraRot.m[3][3] = 1.0f;
        MultiplyMatrixVector(&vTarget, &vLookDir, &matCameraRot);
        vTarget = Vector_Add(&vCamera, &vLookDir);
        //printf("%d \n",vLookDir.z); 

        mat4x4 matCamera = Matrix_PointAt(&vCamera, &vTarget, &vUp);
        mat4x4 matView = Matrix_QuickInverse(&matCamera);

        mat4x4 matRotZ, matRotX, matRotY, matTrans;
        fThetaZ = 0.0f;
        fThetaX = 0.0f;
        fThetaY = 0.0f;

        matRotZ.m[0][0] = cosf(fThetaZ);
        matRotZ.m[0][1] = sinf(fThetaZ);
        matRotZ.m[1][0] = -sinf(fThetaZ);
        matRotZ.m[1][1] = cosf(fThetaZ);
        matRotZ.m[2][2] = 1;
        matRotZ.m[3][3] = 1;

        matRotX.m[0][0] = 1;
        matRotX.m[1][1] = cosf(fThetaX * 0.5f);
        matRotX.m[1][2] = sinf(fThetaX * 0.5f);
        matRotX.m[2][1] = -sinf(fThetaX * 0.5f);
        matRotX.m[2][2] = cosf(fThetaX * 0.5f);
        matRotX.m[3][3] = 1;

        matRotY.m[0][0] = cosf(fThetaY);
        matRotY.m[0][2] = sinf(fThetaY);
        matRotY.m[2][0] = -sinf(fThetaY);
        matRotY.m[1][1] = 1.0f;
        matRotY.m[2][2] = cosf(fThetaY);
        matRotY.m[3][3] = 1.0f;

        matTrans.m[0][0] = 1.0f;
        matTrans.m[1][1] = 1.0f;
        matTrans.m[2][2] = 1.0f;
        matTrans.m[3][3] = 1.0f;
        matTrans.m[3][0] = x;
        matTrans.m[3][1] = y;
        matTrans.m[3][2] = z;

        for (int i = 0; i<= meshCube.counter; i++)
            {
                triangle tri = meshCube.tris[i];

                triangle triProjected, triTranslated, triRotatedZ, triRotatedZX, triRotatedZXY, triViewd;

                // Rotate in Z-Axis
                MultiplyMatrixVector(&tri.p[0], &triRotatedZ.p[0], &matRotZ);
                MultiplyMatrixVector(&tri.p[1], &triRotatedZ.p[1],& matRotZ);
                MultiplyMatrixVector(&tri.p[2], &triRotatedZ.p[2], &matRotZ);

                // Rotate in X-Axis
                MultiplyMatrixVector(&triRotatedZ.p[0], &triRotatedZX.p[0], &matRotX);
                MultiplyMatrixVector(&triRotatedZ.p[1], &triRotatedZX.p[1], &matRotX);
                MultiplyMatrixVector(&triRotatedZ.p[2], &triRotatedZX.p[2], &matRotX);

                // Rotate in YAxis
                MultiplyMatrixVector(&triRotatedZX.p[0], &triRotatedZXY.p[0], &matRotY);
                MultiplyMatrixVector(&triRotatedZX.p[1], &triRotatedZXY.p[1], &matRotY);
                MultiplyMatrixVector(&triRotatedZX.p[2], &triRotatedZXY.p[2], &matRotY);

                
                // Offset into the screen
                MultiplyMatrixVector(&triRotatedZXY.p[0], &triTranslated.p[0], &matTrans);
                MultiplyMatrixVector(&triRotatedZXY.p[1], &triTranslated.p[1], &matTrans);
                MultiplyMatrixVector(&triRotatedZXY.p[2], &triTranslated.p[2], &matTrans);


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
                    MultiplyMatrixVector(&triTranslated.p[0], &triViewd.p[0], &matView);
                    MultiplyMatrixVector(&triTranslated.p[1], &triViewd.p[1], &matView);
                    MultiplyMatrixVector(&triTranslated.p[2], &triViewd.p[2], &matView);

                    MultiplyMatrixVector(&triViewd.p[0], &triProjected.p[0], &matProj);
                    MultiplyMatrixVector(&triViewd.p[1], &triProjected.p[1], &matProj);
                    MultiplyMatrixVector(&triViewd.p[2], &triProjected.p[2], &matProj);

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