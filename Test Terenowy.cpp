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
#define dsDrawSphere dsDrawSphereD
#endif

#define DENSITY (5.0)	

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static int flag = 0;


struct MyObject
{
    dBodyID body;
	dGeomID geom;
};


const int DL_Most = 100;
MyObject box[DL_Most];
MyObject platforma[2];
MyObject ball;

const dReal r = 0.2;
const dReal m = 1.0;
const dReal m1 = 1.0;
const dReal *pos1,*R1,*pos2,*R2;
dReal angle_deg[100];
dReal sides[3] = {5,0.25,0.05};
dReal platforma_sides[3] = {5,3,2};
dReal platforma2_sides[3] = {5,3,0};
// boki platform
dReal angle[100]; // katy belek
dMatrix3 R; // macierz obrotu

double z;
dReal x=1;
dReal wys[DL_Most];
dReal odl[DL_Most]; // odchyłki belek

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if (isGround)  
  {
	  if (n >= 1) flag = 1;
	  else flag = 0;
	  for (int i=0; i < n; i++) 
	  {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu   = dInfinity;
		contact[i].surface.bounce     = 0.0; // (0.0~1.0) restitution parameter
		contact[i].surface.bounce_vel = 0.0; // minimum incoming velocity for bounce
		dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
		dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
	}
  }
 
}

// start simulation
static void start()
{
    static float xyz[3] = {12,3,2};    // eye point[m]
    static float hpr[3] = {-180, 0, 0}; // gaze direction[��]
    dsSetViewpoint (xyz,hpr);           // set viewpoint
}


// simulation loop
static void simLoop (int pause)
{
    const dReal *pos1,*R1,*pos2,*R2,*posG,*RG;
	
	int j=0;
	wys[j]=2;

	//dSpaceCollide(space,0,&nearCallback); KOLIZJE WYŁĄCZONE -> CRASH PRZY GENEROWANIU BELEK. PRZY GENEROWANIU SAMYCH PLATFORM PROGRAM DZIAŁA.
	dWorldStep(world,0.01);
	dJointGroupEmpty(contactgroup);
	
    const dReal *pos3,*R3;

    dWorldStep(world,0.01);

	if (flag == 0) dsSetColor(1.0, 0.0, 0.0);
    dsSetColor(0.0,0.5,0.0);
	pos3 = dBodyGetPosition(ball.body);
    R3   = dBodyGetRotation(ball.body);
    dsDrawSphere(pos3,R3,r);
	
	
	for (int k=1; k<3 ; k++)
		{
		dsSetColorAlpha (0.5,0.5,0.5,1);
		pos2 = dBodyGetPosition(platforma[k-1].body);
		R2   = dBodyGetRotation(platforma[k-1].body);
		if (k==1)
		{

			dBodySetPosition (platforma[k-1].body,0,-1.6,1);
			dBodySetRotation (platforma[k-1].body,R2);
			pos2 = dBodyGetPosition(platforma[k-1].body);
			R2   = dBodyGetRotation(platforma[k-1].body);
			dsDrawBox(pos2,R2,platforma_sides);
		}
		else
		{
		
			dBodySetPosition (platforma[k-1].body,0,(odl[DL_Most-1]+1.6),(0.5*wys[DL_Most-1]));
			dBodySetRotation (platforma[k-1].body,R2);
			pos2 = dBodyGetPosition(platforma[k-1].body);
			R2   = dBodyGetRotation(platforma[k-1].body);
			platforma2_sides[2]=wys[DL_Most-1];
			dsDrawBox(pos2,R2,platforma2_sides);
		}
	}
	
	
  	for (int i=1; i<(DL_Most+1) ; i++)
	{
		angle_deg[i-1] = angle[i-1] * M_PI / 180.0;
		dRFromAxisAndAngle(R, 1, 0, 0, angle_deg[i-1]);
		if (wys[i-1]<1 && angle_deg<0) angle_deg[i]=((-1)*angle_deg[i]) ;
		dsSetColorAlpha (0.9,0.6,0,2);
		dBodySetPosition (box[i-1].body,0,odl[i-1],wys[i-1]);
		dBodySetRotation(box[i-1].body, R);
		dGeomSetRotation(box[i-1].geom, R);
		dGeomSetPosition(box[i-1].geom,0,odl[i-1],wys[i-1]);
		pos1 = dBodyGetPosition(box[i-1].body);
		R1   = dBodyGetRotation(box[i-1].body);
		posG = dGeomGetPosition(box[i-1].geom);
		RG  = dGeomGetRotation(box[i-1].geom);
		wys[i]=wys[i-1]+(sides[1]*0.5*sin(angle_deg[i-1]))+(sides[1]*0.5*sin(angle_deg[i]));
		odl[i]=odl[i-1]+(sides[1]*0.5*cos(angle_deg[i-1]))+(sides[1]*0.5*cos(angle_deg[i]));
		dsDrawBox(pos1,R1,sides);
	
	}
	
		
}

void MakeBox()
{
	
	for (int i=0; i<DL_Most ; i++)
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

	box[i].geom = dCreateBox(space,sides[0],sides[1],sides[2]);
    dGeomSetBody(box[i].geom,box[i].body);

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
    dBodySetPosition (platforma[k-1].body,0,-1.6,1);

	platforma[k-1].geom = dCreateBox(space,platforma_sides[0],platforma_sides[1],platforma_sides[2]);
    dGeomSetBody(platforma[k-1].geom,platforma[k-1].body);

}
}

void MakeBall()
{
dMass m;
ball.body=dBodyCreate(world);
dMassSetZero(&m);
dMassSetSphereTotal(&m,m1,r);
dBodySetMass(ball.body,&m);
dBodySetPosition(ball.body, 4.0, 2.0, 6.0);

ball.geom = dCreateSphere(space,r);
dGeomSetBody(ball.geom,ball.body);

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
	space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

	dWorldSetGravity(world,0,0,-0.5);

	ground = dCreatePlane(space,0,0,1,0);
	MakeBox();
	MakePlatforma();
	MakeBall();

    // do the simulation
    dsSimulationLoop (argc,argv,960,480,&fn);

    dWorldDestroy (world); // destroy the world
    dCloseODE();           // close ODE

    return 0;
}