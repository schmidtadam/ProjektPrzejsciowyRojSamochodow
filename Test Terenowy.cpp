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



struct MyObject
{
    dBodyID body;
	dGeomID geom;
};


const int DL_Most = 40;
const int Test_Pilek = 5;

dGeomID box[DL_Most];
MyObject platforma[2];
MyObject ball[Test_Pilek];

const dReal r = 0.35;
const dReal m = 1.0;
const dReal mp = 40.0;
const dReal m1 = 1.0;
const dReal *pos1,*R1,*pos2,*R2;

dReal sides[3] = {5,0.4,0.05};
dReal platforma_sides[3] = {5,3,2};
dReal platforma_sides2[3] = {5,3,0};
// boki platform

dMatrix3 R; // macierz obrotu

dReal angle[DL_Most]; // katy belek
dReal angle_deg[DL_Most];
dReal wys[DL_Most];
dReal odl[DL_Most]; // odchyłki belek
dReal odlp[2]; // pozycje platform
dReal wysp[2];


static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 8;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));  
  
	  for (int i=0; i < n; i++) 
	  {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu   = dInfinity;
		contact[i].surface.bounce     = 0.2; // (0.0~1.0) restitution parameter
		contact[i].surface.bounce_vel = 0.0; // minimum incoming velocity for bounce
		dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
		dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
	}
  
 
}


static void start()
{
    static float xyz[3] = {12,3,2};    
    static float hpr[3] = {-180, 0, 0}; 
    dsSetViewpoint (xyz,hpr);           
}


static void simLoop (int pause)
{
    const dReal *pos1,*R1,*pos2,*R2, *pos3,*R3;
	
	
	dSpaceCollide(space,0,&nearCallback); //KOLIZJE WYŁĄCZONE -> CRASH PRZY GENEROWANIU BELEK. PRZY GENEROWANIU SAMYCH PLATFORM PROGRAM DZIAŁA.
	dWorldStep(world,0.01);
	dJointGroupEmpty(contactgroup);
	
    dWorldStep(world,0.01);
	
	for (int j=1; j<(DL_Most+1); j++) // belki
	{
		dRFromAxisAndAngle(R, 1, 0, 0, angle_deg[j-1]);
		dsSetColorAlpha (0.9,0.6,0,2);
		dGeomSetPosition(box[j-1],0,odl[j-1],wys[j-1]);
		dGeomSetRotation(box[j-1],R);
		pos1 = dGeomGetPosition(box[j-1]);
		R1 = dGeomGetRotation(box[j-1]);
		dGeomBoxGetLengths (box[j-1],sides);
		dsDrawBox(pos1,R1,sides);
	}

	for (int k=1; k<3; k++) // platformy
	{
		dsSetColorAlpha (0.5,0.5,0.5,1);
		pos2 = dBodyGetPosition(platforma[k-1].body);
		R2   = dBodyGetRotation(platforma[k-1].body);
		if (k==1) {	dsDrawBox(pos2,R2,platforma_sides);}
		else dsDrawBox(pos2,R2,platforma_sides2);
	}

	for (int p=0; p<Test_Pilek; p++) // piłki
	{
	    dsSetColor(0.0,0.5,0.0);
		dsSetSphereQuality(3);
		pos3 = dBodyGetPosition(ball[p].body);
		R3   = dBodyGetRotation(ball[p].body);
		dsDrawSphere(pos3,R3,r);
	}


	/* ***STARA WERSJA SIMLOOP***

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
	*/
	
	
		
}

void MakeBox()
{

	for (int i=0; i<DL_Most ; i++)
	{
		box[i] = dCreateBox(space,sides[0],sides[1],sides[2]);

		/* ***STARA WERSJA MAKEBOX***
		dMass m; 
		angle[i]=rand()%120-60;
		angle[0]=0;
		z=i*0.5;
		dMassSetZero (&m);       
		box[i].body = dBodyCreate (world);
		dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
		dBodySetMass (box[i].body,&m);
		dBodySetPosition (box[i].body,0,z,1);
		dGeomSetBody(box[i].geom,box[i].body);
		*/
	}
	
}

void MakePlatforma()
{
	odlp[0]=-3.1;
	odlp[1]=odl[DL_Most-1]+1.75;
	wysp[0]=1;
	wysp[1]=0.5*wys[DL_Most-1]+0.012;
	platforma_sides2[2]=wys[DL_Most-1]+0.025;


	for (int k=1; k<3 ; k++)
	{
		if(k==1)
		{
			dMass mp; 
			platforma[k-1].body = dBodyCreate (world);
			dMassSetZero (&mp); 
			dMassSetBox (&mp,DENSITY,platforma_sides[0],platforma_sides[1],platforma_sides[2]);
			dBodySetMass (platforma[k-1].body,&mp);
			dBodySetPosition (platforma[k-1].body,0,odlp[k-1],wysp[k-1]);
			platforma[k-1].geom = dCreateBox(space,platforma_sides[0],platforma_sides[1],platforma_sides[2]);
			dGeomSetBody(platforma[k-1].geom,platforma[k-1].body);
		}
	else
		{
			dMass mp; 
			platforma[k-1].body = dBodyCreate (world);
			dMassSetZero (&mp); 
			dMassSetBox (&mp,DENSITY,platforma_sides[0],platforma_sides[1],platforma_sides2[2]);
			dBodySetMass (platforma[k-1].body,&mp);
			dBodySetPosition (platforma[k-1].body,0,odlp[k-1],wysp[k-1]);
			platforma[k-1].geom = dCreateBox(space,platforma_sides[0],platforma_sides[1],platforma_sides2[2]);
			dGeomSetBody(platforma[k-1].geom,platforma[k-1].body);
		}
	}
}

void MakeBall()
{

	for (int i=0; i<Test_Pilek ; i++)
	{
		dMass m;
		ball[i].body=dBodyCreate(world);
		dMassSetZero(&m);
		dMassSetSphereTotal(&m,m1,r);
		dBodySetMass(ball[i].body,&m);
		dBodySetPosition(ball[i].body, 0.0, (i*3), 8.0);
		ball[i].geom = dCreateSphere(space,r);
		dGeomSetBody(ball[i].geom,ball[i].body);
	}
}

void WartosciPoczatkowe() // Wylosowanie kątów, ustawienie belek i platform
{
	angle[0]=0;
	angle[DL_Most-1]=0;

	for (int i=1;i<(DL_Most-1);i++)
	{
		angle[i]=rand()%120-60;
		angle_deg[i] = angle[i] * M_PI / 180.0;
	}

	for (int o=0;o<1;o++)
	{
		wys[o]=1.975;
		odl[o]=-1.375;
	}
			
	for (int j=1;j<DL_Most;j++)
	{
		wys[j]=wys[j-1]+(sides[1]*0.5*sin(angle_deg[j-1]))+(sides[1]*0.5*sin(angle_deg[j]));
		odl[j]=0.02+odl[j-1]+(sides[1]*0.5*cos(angle_deg[j-1]))+(sides[1]*0.5*cos(angle_deg[j]));
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
		
    dInitODE();             
    world = dWorldCreate(); 
	space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

	dWorldSetGravity(world,0,0,-0.5);

	ground = dCreatePlane(space,0,0,1,0);
	
	WartosciPoczatkowe();
	MakeBox();
	MakePlatforma();
	MakeBall();

    dsSimulationLoop (argc,argv,1200,600,&fn);

    dWorldDestroy (world); 
    dCloseODE();           

    return 0;
}