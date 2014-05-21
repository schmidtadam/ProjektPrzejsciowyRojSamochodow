#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h> 
#include <conio.h>
#include <cmath.h>
#include<Windows.h>
#include <iostream>

using namespace std;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
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

class sam

{
public:

MyObject coss[2];
 MyObject coss2[4];
MyObject cylinder[4];
void MakeKabina(int licznik);
void MakeCar(int licznik);

dJointID joint[5];
dJointID joint2[4];
dJointID joint3[2];

dMatrix3 R4,R5;
dReal radius;
dReal length;
dReal wysokosc;
dReal os;
dReal platforma_sides3[3];
dReal platforma_sides33[3];
sam();
};

sam::sam()
{const dReal m = 0.1; //masa koła
const dReal z = 0.1; //masa czesci kabiny
const dReal z2 = 0.1; //masa czesci kabiny
const dReal z3 = 0.1; //masa czesci kabiny
const dReal z4 = 0.1; //masa czesci kabiny
const dReal z5 = 0.1; //masa czesci kabiny
const dReal z6 = 0.1; //masa czesci kabiny
const dReal z7 = 0.1; //masa czesci kabiny
const dReal z8 = 0.1; //masa czesci kabiny
}
sam samochod[3];

dReal przesuniecie=1;
const int DL_Most = 40;
const int Test_Pilek = 5;

dGeomID box[DL_Most];
MyObject platforma[2];
MyObject ball[Test_Pilek];
/*MyObject coss[2];
MyObject coss2[4];
MyObject cylinder[4];*/

const dReal r = 0.35;
//const dReal m = 5;
const dReal mp = 400.0;
const dReal m1 = 1.0;
const dReal p = 0.05;
const dReal *pos1,*R1,*pos2,*R2;



dReal sides[3] = {5,0.4,0.05};
dReal platforma_sides[3] = {5,3,2};
dReal platforma_sides2[3] = {5,3,0};

// boki platform
//dJointID joint[5];
dMatrix3 R; // macierz obrotu

//dMatrix3 R4,R5;
/*const dReal m = 0.1; //masa koła
const dReal z = 0.1; //masa czesci kabiny
const dReal z2 = 0.1; //masa czesci kabiny
const dReal z3 = 0.1; //masa czesci kabiny
const dReal z4 = 0.1; //masa czesci kabiny*/
dReal pi=3.14;
/*
dReal radius = 0.35; // radius
dReal length = 0.1;  // length
dReal wysokosc=0.15;
dReal os=0.8; //odleglosc miedzy przednim, a tylnym kołem
dReal platforma_sides3[3] = {0.5,os,0.02};
dReal platforma_sides33[3] = {0.5,sqrt((0.25*os*os)+(wysokosc*wysokosc)),0.02};
*/
dReal angle[DL_Most]; // katy belek
dReal angle_deg[DL_Most];
dReal wys[DL_Most];
dReal odl[DL_Most]; // odchyłki belek
dReal odlp[2]; // pozycje platform
dReal wysp[2];
const dReal*predkosc[2];

//dJointID joint2[4];
//dJointID joint3[2];
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 30;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));  
  
	  for (int i=0; i < n; i++) 
	  {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu   = 0.3;
		contact[i].surface.bounce     = 0.1; // (0.0~1.0) restitution parameter
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
    const dReal *pos1,*R1,*pos2,*R2, *pos3,*R3,*pos4,*R4,*pos6,*R6,*pos7,*R7;


	dSpaceCollide(space,0,&nearCallback); //KOLIZJE WY��CZONE -> CRASH PRZY GENEROWANIU BELEK. PRZY 


	dWorldStep(world,0.01);
	dJointGroupEmpty(contactgroup);

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

/*for (int p=0; p<Test_Pilek; p++) // pi�ki
	{
	    dsSetColor(0.0,0.5,0.0);
		dsSetSphereQuality(3);
		pos3 = dBodyGetPosition(ball[p].body);
		R3   = dBodyGetRotation(ball[p].body);
		dsDrawSphere(pos3,R3,r);
	}*/
	

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


for(int licznik=0;licznik<2;licznik++)
{
		 for (int i=0; i<4 ; i++) //ko�a
    {
    
    dsSetColorAlpha (1,1,0,1);
    pos6 = dBodyGetPosition(samochod[licznik].cylinder[i].body);
    R6 = dBodyGetRotation(samochod[licznik].cylinder[i].body);
    dsDrawCylinder(pos6,R6,samochod[licznik].length,samochod[licznik].radius);
	}



	 for (int i=0; i<1 ; i++) //samoch�d
	 {
	    dsSetColorAlpha (0.5,0.5,0.5,1);
		pos4 = dBodyGetPosition(samochod[licznik].coss[i].body);
		R4   = dBodyGetRotation(samochod[licznik].coss[i].body);
		dsDrawBox(pos4,R4,samochod[licznik].platforma_sides3);
    }
	  for (int i=0; i<4 ; i++) //samoch�d
	 {
	    dsSetColorAlpha (0.5,0.5,0.5,1);
		pos7 = dBodyGetPosition(samochod[licznik].coss2[i].body);
		R7   = dBodyGetRotation(samochod[licznik].coss2[i].body);
		dsDrawBox(pos7,R7,samochod[licznik].platforma_sides33);
    }
	  predkosc[licznik]=dBodyGetLinearVel(samochod[licznik].coss[0].body);
}



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
			platforma[k-1].geom = dCreateBox(space,platforma_sides[0],platforma_sides
[1],platforma_sides[2]);
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
			platforma[k-1].geom = dCreateBox(space,platforma_sides[0],platforma_sides

[1],platforma_sides2[2]);
			dGeomSetBody(platforma[k-1].geom,platforma[k-1].body);
		}
	}
}
 
void MakeBall()
{

	for (int i=0; i<Test_Pilek ; i++)
	{
		dMass p;
		ball[i].body=dBodyCreate(world);
		dMassSetZero(&p);
		dMassSetSphereTotal(&p,m1,r);
		dBodySetMass(ball[i].body,&p);
		dBodySetPosition(ball[i].body, 0.0, (i*3), 8.0);
		ball[i].geom = dCreateSphere(space,r);
		dGeomSetBody(ball[i].geom,ball[i].body);
	}
}

void WartosciPoczatkowe() // Wylosowanie k�t�w, ustawienie belek i platform
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
		odl[j]=0.05+odl[j-1]+(sides[1]*0.5*cos(angle_deg[j-1]))+(sides[1]*0.5*cos(angle_deg[j]));
	}
}

void losuj_parametry(int licznik)

{int zmienna;
	
samochod[licznik].length=0.1;
samochod[licznik].radius=((rand()%20)+35)*0.01;
samochod[licznik].wysokosc=0.15;

samochod[licznik].os=2*samochod[licznik].radius+0.1;
samochod[licznik].platforma_sides3[0]=0.5;
samochod[licznik].platforma_sides3[1]=samochod[licznik].os;
samochod[licznik].platforma_sides3[2]=0.02;

samochod[licznik].platforma_sides33[0]=0.5;
samochod[licznik].platforma_sides33[1]=sqrt((0.25*samochod[licznik].os*samochod[licznik].os)+(samochod[licznik].wysokosc*samochod[licznik].wysokosc));
samochod[licznik].platforma_sides33[2]=0.02;
cout<<samochod[licznik].radius ;
cout<<samochod[licznik].wysokosc;
cout<<samochod[licznik].os;




}

void sam:: MakeKabina(int licznik) //kabina samochodu
{		const dReal *pozycja;
			pozycja=dBodyGetPosition(platforma[0].body);
			dReal x,y,q,y2;

			dReal kat;
			kat=wysokosc/(0.5*os);
			kat=atan(kat);

			q=pozycja[0]-0.5-przesuniecie*licznik;
	x=pozycja[1]+0.25*os;
	y=pozycja[2]+platforma_sides[2]/2+(radius-wysokosc)-0.02;
		y2=pozycja[2]+platforma_sides[2]/2;

	dMass z2; // a parameter for mass
			dMassSetZero (&z2);       // initialize the parameter
			coss2[0].body = dBodyCreate (world);
			dMassSetBox (&z2,DENSITY,platforma_sides33[0],platforma_sides33[1],platforma_sides33[2]);
			dBodySetMass (coss2[0].body,&z2);
			dRFromAxisAndAngle(R5,1,0,0,-kat);
			dBodySetRotation(coss2[0].body,R5);
			dBodySetPosition (coss2[0].body,q,x,y+0.5*wysokosc);
			coss2[0].geom = dCreateBox(space,platforma_sides33[0],platforma_sides33[1],platforma_sides33[2]);
			dGeomSetBody(coss2[0].geom,coss2[0].body);

			
	dMass z6; // a parameter for mass
			dMassSetZero (&z6);       // initialize the parameter
			coss2[1].body = dBodyCreate (world);
			dMassSetBox (&z6,DENSITY,platforma_sides3[0],platforma_sides33[1],platforma_sides3[2]);
			dBodySetMass (coss2[1].body,&z6);
			dRFromAxisAndAngle(R4,1,0,0,kat);
			dBodySetRotation(coss2[1].body,R4);
			dBodySetPosition (coss2[1].body,q,x+platforma_sides33[1]*sin(90-kat)-platforma_sides33[2]*sin(kat),y+0.5*wysokosc);
			coss2[1].geom = dCreateBox(space,platforma_sides33[0],platforma_sides33[1],platforma_sides33[2]);
			dGeomSetBody(coss2[1].geom,coss2[1].body);

		joint[2] = dJointCreateFixed(world, 0);
        dJointAttach(joint[2], coss2[1].body, coss2[0].body);
		dJointSetFixed(joint[2]);

		joint[3] = dJointCreateFixed(world, 0);
        dJointAttach(joint[3], coss2[1].body, coss[0].body);
		dJointSetFixed(joint[3]);

		

		
	dMass z7; // a parameter for mass
			dMassSetZero (&z7);       // initialize the parameter
			coss2[2].body = dBodyCreate (world);
			dMassSetBox (&z7,DENSITY,platforma_sides3[0],platforma_sides33[1],platforma_sides3[2]);
			dBodySetMass (coss2[2].body,&z7);
			dRFromAxisAndAngle(R4,1,0,0,kat);
			dBodySetRotation(coss2[2].body,R4);
			dBodySetPosition (coss2[2].body,q,x,y2+radius+0.5*wysokosc+platforma_sides3[2]);
			coss2[2].geom = dCreateBox(space,platforma_sides3[0],platforma_sides33[1],platforma_sides3[2]);
			dGeomSetBody(coss2[2].geom,coss2[2].body);

			
	dMass z8; // a parameter for mass
			dMassSetZero (&z8);       // initialize the parameter
			coss2[3].body = dBodyCreate (world);
			dMassSetBox (&z8,DENSITY,platforma_sides3[0],platforma_sides33[1],platforma_sides3[2]);
			dBodySetMass (coss2[3].body,&z8);
			dRFromAxisAndAngle(R5,1,0,0,-kat);
			dBodySetRotation(coss2[3].body,R5);
			dBodySetPosition (coss2[3].body,q,x+platforma_sides33[1]*sin(90-kat)-platforma_sides33[2]*sin(kat),y2+radius+0.5*wysokosc+platforma_sides3[2]);
			coss2[3].geom = dCreateBox(space,platforma_sides3[0],platforma_sides33[1],platforma_sides3[2]);
			dGeomSetBody(coss2[3].geom,coss2[3].body);

	joint[4] = dJointCreateFixed(world, 0);
        dJointAttach(joint[4], coss2[2].body, coss2[3].body);
		dJointSetFixed(joint[4]);

		joint[5] = dJointCreateFixed(world, 0);
        dJointAttach(joint[5], coss2[3].body, coss[0].body);
		dJointSetFixed(joint[5]);
		
}

void sam::MakeCar(int licznik)

{
	const dReal *pozycja;
	pozycja=dBodyGetPosition(platforma[0].body);
	dReal x,y,q;
	q=pozycja[0]-przesuniecie*licznik;
	x=pozycja[1];
	y=pozycja[2]+platforma_sides[2]/2;

	dMass m; // a parameter for mass
    dMassSetZero (&m);       // initialize the parameter
    cylinder[0].body = dBodyCreate (world);
    dMassSetCylinder(&m,DENSITY,3,radius,length);
    dBodySetMass (cylinder[0].body,&m);
    dRFromAxisAndAngle(R,0,1,0, pi/2);
	dBodySetPosition(cylinder[0].body,q-0.15,x,y+radius);
    dBodySetRotation(cylinder[0].body,R);
    cylinder[0].geom=dCreateCylinder(space,radius,length);
    dGeomSetBody(cylinder[0].geom,cylinder[0].body);
	dBodySetTorque(cylinder[0].body,-105,0,0);

	dMass z; // a parameter for mass
    dMassSetZero (&z);       // initialize the parameter
    cylinder[1].body = dBodyCreate (world);
    dMassSetCylinder(&z,DENSITY,3,radius,length);
    dBodySetMass (cylinder[1].body,&z);
    dRFromAxisAndAngle(R,0,1,0, pi/2);
	dBodySetPosition(cylinder[1].body,q-0.85,x,y+radius);
    dBodySetRotation(cylinder[1].body,R);
    cylinder[1].geom=dCreateCylinder(space,radius,length);
    dGeomSetBody(cylinder[1].geom,cylinder[1].body);
	dBodySetTorque(cylinder[1].body,-105,0,0);
	
		
	joint[0] = dJointCreateFixed(world, 0);
    dJointAttach(joint[0], cylinder[0].body,cylinder[1].body);
	dJointSetFixed(joint[0]);

	joint[1] = dJointCreateFixed(world, 0);
    dJointAttach(joint[1], cylinder[2].body,cylinder[3].body);
	dJointSetFixed(joint[1]);

	dMass z5; // a parameter for mass
    dMassSetZero (&z5);       // initialize the parameter
	coss[0].body = dBodyCreate (world);
	dMassSetBox (&z5,DENSITY,platforma_sides3[0],platforma_sides3[1],platforma_sides3[2]);
	dBodySetMass (coss[0].body,&z);
	//dRFromAxisAndAngle(R5, 1,0,0,-90+kat);
	dBodySetPosition (coss[0].body,q-0.5,x+(os/2),y+radius);
	//dBodySetRotation(coss[0].body,R5);
	coss[0].geom = dCreateBox(space,platforma_sides3[0],platforma_sides3[1],platforma_sides3[2]);
	dGeomSetBody(coss[0].geom,coss[0].body);
		
	MakeKabina(licznik);


	dMass z3; // a parameter for mass
    dMassSetZero (&z3);       // initialize the parameter
    cylinder[2].body = dBodyCreate (world);
    dMassSetCylinder(&z3,DENSITY,3,radius,length);
    dBodySetMass (cylinder[2].body,&z3);
    dRFromAxisAndAngle(R,0,1,0, pi/2);
	dBodySetPosition(cylinder[2].body,q-0.15,x+os,y+radius);
    dBodySetRotation(cylinder[2].body,R);
    cylinder[2].geom=dCreateCylinder(space,radius,length);
    dGeomSetBody(cylinder[2].geom,cylinder[2].body);
	//dBodySetTorque(cylinder[2].body,-50,0,0);

	dMass z4; // a parameter for mass
    dMassSetZero (&z4);       // initialize the parameter
    cylinder[3].body = dBodyCreate (world);
    dMassSetCylinder(&z4,DENSITY,3,radius,length);
    dBodySetMass (cylinder[3].body,&z4);
    dRFromAxisAndAngle(R,0,1,0, pi/2);
	dBodySetPosition(cylinder[3].body,q-0.85,x+os,y+radius);
    dBodySetRotation(cylinder[3].body,R);
    cylinder[3].geom=dCreateCylinder(space,radius,length);
    dGeomSetBody(cylinder[3].geom,cylinder[3].body);
	//dBodySetTorque(cylinder[3].body,-50,0,0);

	joint2[1] = dJointCreateHinge2(world, 0);
    dJointAttach(joint2[1], cylinder[0].body, coss[0].body);
	dJointSetHinge2Anchor(joint2[1],q-0.15,x,y+radius);
    dJointSetHinge2Axis1(joint2[1], 0,1,0);
	dJointSetHinge2Axis2(joint2[1],1, 0,0);

	joint2[2] = dJointCreateHinge2(world, 0);
    dJointAttach(joint2[2], cylinder[1].body, coss[0].body);
	dJointSetHinge2Anchor(joint2[2],q-0.85,x,y+radius);
	dJointSetHinge2Axis1(joint2[2], 0,1,0);
	dJointSetHinge2Axis2(joint2[2], 1,0,0);

	joint2[0] = dJointCreateHinge(world, 0);
	dJointAttach(joint2[0], cylinder[2].body, coss[0].body);
	dJointSetHingeAnchor(joint2[0],q-0.15,x+os,y+radius);
   // dJointSetHinge2Axis1(joint2[0], 0,1,0);
	dJointSetHingeAxis(joint2[0], 1,0,0);


	joint2[3] = dJointCreateHinge(world, 0);
	dJointAttach(joint2[3], cylinder[3].body, coss[0].body);
	dJointSetHingeAnchor(joint2[3],q-0.85,x+os,y+radius);
   //  dJointSetHinge2Axis1(joint2[3], 0,1,0);
	dJointSetHingeAxis(joint2[3], 1,0,0);

		
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

	dWorldSetGravity(world,0,0,-2);
	dWorldSetCFM(world,10E-7);
	dWorldSetERP(world,0.2);
	ground = dCreatePlane(space,0,0,1,0);

	WartosciPoczatkowe();
	MakeBox();
	MakePlatforma();
	//MakeBall();
	
	for (int licznik=0;licznik<2;licznik++)
	{losuj_parametry(licznik);
	samochod[licznik].MakeCar(licznik);
	
	}
    dsSimulationLoop (argc,argv,1200,600,&fn);

    dWorldDestroy (world); 
    dCloseODE();           

    return 0;
}