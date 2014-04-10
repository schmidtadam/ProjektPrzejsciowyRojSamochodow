/* ODE tutorial  by Kosei Demura  http://demura.net   */
/* sample4.cpp :Lesson 4 3D Graphics 2008-2011*/
// A program for drawing a sphere, a cylinder, a capsule, a box and a ray
// Only drawing, no collision detection and no dynamics

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h> 
#include <conio.h>
#include <cmath>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#endif

#define DENSITY (5.0)	


struct MyObject
{
    dBodyID body;		
};
MyObject box[100];
MyObject platforma[2];

const dReal   m   = 1.0;
const dReal *pos1,*R1,*pos2,*R2;
dReal angle_deg[100];
dReal sides[3] = {5,0.25,0.05};
dReal platforma_sides[3] = {5,3,2};
dReal platforma2_sides[3] = {5,3,0};
// sides
dReal angle[100];
dMatrix3 R;
static dWorldID world;  // a dynamic world
double z;
dReal x=1;
dReal wys[100];
dReal odl[100];

// start simulation
static void start()
{
    static float xyz[3] = {5,3,0.5};    // eye point[m]
    static float hpr[3] = {-180, 0, 0}; // gaze direction[��]
    dsSetViewpoint (xyz,hpr);           // set viewpoint
}


// simulation loop
static void simLoop (int pause)
{
    const dReal *pos1,*R1,*pos2,*R2;
	int j=0;
	wys[j]=2;
	dWorldStep(world,0.3);
	
	for (int k=1; k<3 ; k++)
		{
		dsSetColorAlpha (0.5,0.5,0.5,1);
		pos2 = dBodyGetPosition(platforma[k-1].body);
		R2   = dBodyGetRotation(platforma[k-1].body);
		if (k==1)
		{
			pos2 = dBodyGetPosition(platforma[k-1].body);
			R2   = dBodyGetRotation(platforma[k-1].body);
			dBodySetPosition (platforma[k-1].body,0,-1.6,0.98);
			dBodySetRotation (platforma[k-1].body,R2);
			dsDrawBox(pos2,R2,platforma_sides);
		}
		else
		{
			pos2 = dBodyGetPosition(platforma[k-1].body);
			R2   = dBodyGetRotation(platforma[k-1].body);
			dBodySetPosition (platforma[k-1].body,0,(odl[99]+1.6),(0.5*wys[99]));
			dBodySetRotation (platforma[k-1].body,R2);
			platforma2_sides[2]=wys[99];
			dsDrawBox(pos2,R2,platforma2_sides);
		}
	}

  	for (int i=1; i<101 ; i++)
	{
		angle_deg[i-1] = angle[i-1] * M_PI / 180.0;
		dRFromAxisAndAngle(R, 1, 0, 0, angle_deg[i-1]);
		if (wys[i-1]<1 && angle_deg<0) angle_deg[i]=((-1)*angle_deg[i]) ;
		dsSetColorAlpha (0.9,0.6,0,2);
		dBodySetPosition (box[i-1].body,0,odl[i-1],wys[i-1]);
		dBodySetRotation(box[i-1].body, R);
		pos1 = dBodyGetPosition(box[i-1].body);
		R1   = dBodyGetRotation(box[i-1].body);
		dsDrawBox(pos1,R1,sides);
		wys[i]=wys[i-1]+(sides[1]*0.5*sin(angle_deg[i-1]))+(sides[1]*0.5*sin(angle_deg[i]));
		odl[i]=odl[i-1]+(sides[1]*0.5*cos(angle_deg[i-1]))+(sides[1]*0.5*cos(angle_deg[i]));

	}
}



void MakeBox()
{
	
	for (int i=0; i<100 ; i++)
	{
	dMass m; // a parameter for mass
    angle[i]=rand()%120-60;
	angle[0]=0;
	z=i*0.5;
	dMassSetZero (&m);       // initialize the parameter
    box[i].body = dBodyCreate (world);
    dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
    dBodySetMass (box[i].body,&m);
    dBodySetPosition (box[i].body,0,z,1);
	}
}

void MakePlatforma()
{
		for (int k=1; k<3 ; k++)
{

	dMass m; // a parameter for mass
    dMassSetZero (&m);       // initialize the parameter
    platforma[k-1].body = dBodyCreate (world);
    dMassSetBox (&m,DENSITY,platforma_sides[0],platforma_sides[1],platforma_sides[2]);
    dBodySetMass (platforma[k-1].body,&m);
    dBodySetPosition (platforma[k-1].body,0,-1.6,0.98);
}
}


int main (int argc, char **argv)
{
	time_t t;
    srand(time(&t));

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = NULL;
    fn.stop    = NULL;
    fn.path_to_textures = "C:/ODE/drawstuff/textures";


    dInitODE();              // init ODE
    world = dWorldCreate();  // create a dynamic world
	dWorldSetGravity(world,0,0,-9);

	MakeBox();
	MakePlatforma();
		
    // do the simulation
    dsSimulationLoop (argc,argv,960,480,&fn);

    dWorldDestroy (world); // destroy the world
    dCloseODE();           // close ODE

    return 0;
}