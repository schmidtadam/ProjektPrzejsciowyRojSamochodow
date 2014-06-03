#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h> 
#include <conio.h>
#include <cmath>
#include <Windows.h>
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
void ZniszczAuta();
void losuj_parametry(int licznik);
void nadaj_parametry(int zwyciezca);

struct MyObject
{
    dBodyID body;
	dGeomID geom;
};

class Car

{
	public:

	MyObject RamaP[2];
	MyObject Rama[4];
	MyObject Opona[4];
	void MakeRama(int licznik);
	void MakeCar(int licznik);

	dJointID joint[5];
	dJointID joint2[4];
	dJointID joint3[2];

	dMatrix3 R4,R5;
	dReal radius;
	dReal length;
	dReal wysokosc;
	dReal os;
	dReal WymiaryPoprz[3]; 
	dReal WymiaryRama[3];
	dReal czerwony;
	dReal zielony;
	dReal niebieski;
	Car();
};

Car::Car()
{
	const dReal m = 0.1; //masa ko�a
	const dReal z = 0.1; //masa czesci kabiny
	const dReal z2 = 0.1; //masa czesci kabiny
	const dReal z3 = 0.1; //masa czesci kabiny
	const dReal z4 = 0.1; //masa czesci kabiny
	const dReal z5 = 0.1; //masa czesci kabiny
	const dReal z6 = 0.1; //masa czesci kabiny
	const dReal z7 = 0.1; //masa czesci kabiny
	const dReal z8 = 0.1; //masa czesci kabiny
}

dReal przesuniecie=2;

const int Ilosc_Aut = 4;
const int DL_Most = 50;
const int Test_Pilek = 5;

//Obiekty
Car samochod[Ilosc_Aut];
dGeomID platforma[2];
dGeomID box[DL_Most];
MyObject ball[Test_Pilek];

//Kule
const dReal r = 0.35;
const dReal m1 = 1.0;
const dReal p = 0.05;

//Macierz obrotu
dMatrix3 R; 
dReal pi=3.14;
dReal a,meta;
int zwyciezca=Ilosc_Aut;
int czas=0;



//Platformy i belki
dReal sides[3] = {Ilosc_Aut*2.2,0.34,0.05};
dReal platforma_sides[3] = {Ilosc_Aut*2.2,3,2};
dReal platforma_sides2[3] = {Ilosc_Aut*2.2,3,0};

dReal angle[DL_Most];
dReal angle_deg[DL_Most];
dReal wys[DL_Most];
dReal odl[DL_Most];
dReal odlp[2]; 
dReal wysp[2];

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 30;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));  
  
	  for (int i=0; i < n; i++) 
	  {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu   = 0.6;
		contact[i].surface.bounce     = 0.0; // (0.0~1.0) restitution parameter
		contact[i].surface.bounce_vel = 0.0; // minimum incoming velocity for bounce
		dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
		dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
	}
   
}

static void start()
{
    static float xyz[3] = {12,1.8,5.5};    
    static float hpr[3] = {-190, -22,0}; 
    dsSetViewpoint (xyz,hpr);           
}


void ZniszczAuta()
{
	for(int l=0;l<Ilosc_Aut;l++)//Auta
	{
		for (int i=0; i<5 ; i++) //Opona
		{	
			if (i<1) 
			{
				dGeomDestroy(samochod[l].RamaP[i].geom);
			}

			if (i<4)
			{
				dGeomDestroy(samochod[l].Opona[i].geom);
				dGeomDestroy(samochod[l].Rama[i].geom);
			}
		}	
	}
}




static void simLoop (int pause)
{
   const dReal *pos1,*R1,*pos2,*R2, *pos3,*R3,*pos4,*R4,*pos6,*R6,*pos7,*R7;

	if(czas==0)
	{
		a=0;
		meta=1;
	}
   
	czas++;

	if (a>meta)
	{	
		ZniszczAuta();
		czas=0;

		for (int l=0;l<Ilosc_Aut;l++)
		{
			nadaj_parametry(zwyciezca);
			//losuj_parametry(l);
			samochod[l].MakeCar(l);
		}
	}

dSpaceCollide(space,0,&nearCallback);

dWorldStep(world,0.01);
dJointGroupEmpty(contactgroup);

for (int j=1; j<(DL_Most+1); j++) //Belki
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

for (int k=1; k<3; k++) //Platformy
{
	dsSetColorAlpha (0.5,0.5,0.5,1);
	dGeomSetPosition(platforma[k-1],0,odlp[k-1],wysp[k-1]);
	pos2 = dGeomGetPosition(platforma[k-1]);
	R2   = dGeomGetRotation(platforma[k-1]);
	if (k==1) dsDrawBox(pos2,R2,platforma_sides);
	else 
	{
		dsDrawBox(pos2,R2,platforma_sides2);
		meta=pos2[1];
	}

}

for(int l=0;l<Ilosc_Aut;l++)//Auta
{
	for (int i=0; i<4 ; i++) //Opona
	{
   		 dsSetColorAlpha (1,1,1,1);
		pos6 = dBodyGetPosition(samochod[l].Opona[i].body);
		R6 = dBodyGetRotation(samochod[l].Opona[i].body);
		dsDrawCylinder(pos6,R6,samochod[l].length,samochod[l].radius);
	}

	for (int i=0; i<1 ; i++) //Auto
	{
		dsSetColorAlpha (samochod[l].czerwony,samochod[l].zielony,samochod[l].niebieski,1);
		pos4 = dBodyGetPosition(samochod[l].RamaP[i].body);
		R4   = dBodyGetRotation(samochod[l].RamaP[i].body);
		dsDrawBox(pos4,R4,samochod[l].WymiaryPoprz);
		a=max(a,pos4[1]);
		if(a==pos4[1]) zwyciezca=l;
	}

	for (int i=0; i<4 ; i++) //Auto
	{
		dsSetColorAlpha (samochod[l].czerwony,samochod[l].zielony,samochod[l].niebieski,1);
		pos7 = dBodyGetPosition(samochod[l].Rama[i].body);
		R7   = dBodyGetRotation(samochod[l].Rama[i].body);
		dsDrawBox(pos7,R7,samochod[l].WymiaryRama);
	}

	dBodySetAngularVel(samochod[l].Opona[0].body, -5, 0, 0);
	dBodySetAngularVel(samochod[l].Opona[1].body, -5, 0, 0); 
	dBodySetAngularVel(samochod[l].Opona[2].body, -5, 0, 0);
	dBodySetAngularVel(samochod[l].Opona[3].body, -5, 0, 0);
}

/* KULE DO TESTU FIZYKI - NIEAKTYWNE
for (int p=0; p<Test_Pilek; p++) // Kule
{
   dsSetColor(0.0,0.5,0.0);
dsSetSphereQuality(3);
pos3 = dBodyGetPosition(ball[p].body);
R3   = dBodyGetRotation(ball[p].body);
dsDrawSphere(pos3,R3,r);
}
*/

}



void MakeBox()
{
	for (int i=0; i<DL_Most ; i++)
	{
		box[i] = dCreateBox(space,sides[0],sides[1],sides[2]);
	}
}

void MakePlatforma()
{
	odlp[0]=-3.08;
	odlp[1]=odl[DL_Most-1]+1.75;
	wysp[0]=1;
	wysp[1]=0.5*wys[DL_Most-1]+0.012;
	platforma_sides2[2]=wys[DL_Most-1]+0.025;
	
	for (int k=1; k<3 ; k++)
	{
		if(k==1)
		{
			platforma[k-1] = dCreateBox(space,platforma_sides[0],platforma_sides[1],platforma_sides[2]);
		}
		else
		{
			platforma[k-1] = dCreateBox(space,platforma_sides2[0],platforma_sides2[1],platforma_sides2[2]);
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

void WartosciPoczatkowe() // Wylosowanie katow, ustawienie belek i platform
{
	angle[0]=0;
	angle[DL_Most-1]=0;

	

	for (int i=1;i<(DL_Most-1);i++)
	{
		angle[i]=rand()%130-65;
		angle_deg[i] = angle[i] * M_PI / 180.0;
	}

	for (int o=0;o<1;o++)
	{
		wys[o]=1.975;
		odl[o]=-1.375;
	}

	for (int j=1;j<DL_Most;j++)
	{
		if (wys[j-1]<1)
		{
			if (angle[j]<0) 
			angle_deg[j]=0-(angle[j] * M_PI / 180.0);
			wys[j]=wys[j-1]+(sides[1]*0.5*sin(angle_deg[j-1]))+(sides[1]*0.5*sin(angle_deg[j]));
			odl[j]=odl[j-1]+(sides[1]*0.5*cos(angle_deg[j-1]))+(sides[1]*0.5*cos(angle_deg[j]));
		}
		else
		{
			wys[j]=wys[j-1]+(sides[1]*0.5*sin(angle_deg[j-1]))+(sides[1]*0.5*sin(angle_deg[j]));
			odl[j]=odl[j-1]+(sides[1]*0.5*cos(angle_deg[j-1]))+(sides[1]*0.5*cos(angle_deg[j]));
		}
	}
}

void losuj_parametry(int licznik) //Losowanie parametrow samochodow

{
	int zmienna;
	
	samochod[licznik].length=0.2;
	//samochod[licznik].radius=((rand()%30)+15)*0.01;
	
	samochod[licznik].wysokosc=((rand()%15)+15)*0.01;
	zmienna=samochod[licznik].wysokosc*100+7;
	samochod[licznik].radius=((rand()%30)+zmienna)*0.01;
	//samochod[licznik].radius=0.4;
	zmienna=(2*samochod[licznik].radius+0.1)*100;
	samochod[licznik].os=((rand()%10)+zmienna)*0.01;
		
	samochod[licznik].WymiaryPoprz[0]=0.5;
	samochod[licznik].WymiaryPoprz[1]=samochod[licznik].os;
	samochod[licznik].WymiaryPoprz[2]=0.02;

	samochod[licznik].WymiaryRama[0]=0.5;
	samochod[licznik].WymiaryRama[1]=sqrt((0.25*samochod[licznik].os*samochod[licznik].os)+(samochod[licznik].wysokosc*samochod[licznik].wysokosc));
	samochod[licznik].WymiaryRama[2]=0.02;
	cout<<samochod[licznik].radius ;
	cout<<samochod[licznik].wysokosc;
	cout<<samochod[licznik].os;

	samochod[licznik].czerwony=((rand()%100)+0)*0.01;
	samochod[licznik].zielony=((rand()%100)+0)*0.01;
	samochod[licznik].niebieski=((rand()%100)+0)*0.01;

}

void nadaj_parametry(int zwyciezca) //Losowanie parametrow samochodow

{
	for(int licznik=0;licznik<Ilosc_Aut;licznik++)
	{
	
	samochod[licznik].length=samochod[zwyciezca].length;
	//samochod[licznik].radius=((rand()%30)+15)*0.01;
	
	samochod[licznik].wysokosc=samochod[zwyciezca].wysokosc;
	
	samochod[licznik].radius=samochod[zwyciezca].radius;
	//samochod[licznik].radius=0.4;
	
	samochod[licznik].os=samochod[zwyciezca].os;
		
	samochod[licznik].WymiaryPoprz[0]=0.5;
	samochod[licznik].WymiaryPoprz[1]=samochod[zwyciezca].WymiaryPoprz[1];
	samochod[licznik].WymiaryPoprz[2]=0.02;

	samochod[licznik].WymiaryRama[0]=0.5;
	samochod[licznik].WymiaryRama[1]=samochod[zwyciezca].WymiaryRama[1];
	samochod[licznik].WymiaryRama[2]=0.02;

	samochod[licznik].czerwony=samochod[zwyciezca].czerwony;
	samochod[licznik].zielony=samochod[zwyciezca].zielony;
	samochod[licznik].niebieski=samochod[zwyciezca].niebieski;
	}
}

void Car:: MakeRama(int licznik) //kabina samochodu
{			
			
	dReal x,y,q,y2;
	dReal kat;

	kat=wysokosc/(0.5*os);
	kat=atan(kat);

	q=0-0.5-przesuniecie*licznik+((platforma_sides[0]/2)-1);
	x=odlp[0]+0.25*os;
	y=wysp[0]+platforma_sides[2]/2+(radius-wysokosc)-0.02;
	y2=wysp[0]+platforma_sides[2]/2;

	dMass z2; 
		dMassSetZero (&z2);       
		Rama[0].body = dBodyCreate (world);
		dMassSetBox (&z2,DENSITY,WymiaryRama[0],WymiaryRama[1],WymiaryRama[2]);
		dBodySetMass (Rama[0].body,&z2);
		dRFromAxisAndAngle(R5,1,0,0,-kat);
		dBodySetRotation(Rama[0].body,R5);
		dBodySetPosition (Rama[0].body,q,x,y+0.5*wysokosc);
		Rama[0].geom = dCreateBox(space,WymiaryRama[0],WymiaryRama[1],WymiaryRama[2]);
		dGeomSetBody(Rama[0].geom,Rama[0].body);
			
	dMass z6; 
		dMassSetZero (&z6);       
		Rama[1].body = dBodyCreate (world);
		dMassSetBox (&z6,DENSITY,WymiaryPoprz[0],WymiaryRama[1],WymiaryPoprz[2]);
		dBodySetMass (Rama[1].body,&z6);
		dRFromAxisAndAngle(R4,1,0,0,kat);
		dBodySetRotation(Rama[1].body,R4);
		dBodySetPosition (Rama[1].body,q,x+WymiaryRama[1]*cos(kat)+WymiaryRama[2],y+0.5*wysokosc);
		Rama[1].geom = dCreateBox(space,WymiaryRama[0],WymiaryRama[1],WymiaryRama[2]);
		dGeomSetBody(Rama[1].geom,Rama[1].body);

	joint[2] = dJointCreateFixed(world, 0);
		dJointAttach(joint[2], Rama[1].body, Rama[0].body);
		dJointSetFixed(joint[2]);

	joint[3] = dJointCreateFixed(world, 0);
		dJointAttach(joint[3], Rama[1].body, RamaP[0].body);
		dJointSetFixed(joint[3]);

	dMass z7; 
		dMassSetZero (&z7);       
		Rama[2].body = dBodyCreate (world);
		dMassSetBox (&z7,DENSITY,WymiaryPoprz[0],WymiaryRama[1],WymiaryPoprz[2]);
		dBodySetMass (Rama[2].body,&z7);
		dRFromAxisAndAngle(R4,1,0,0,kat);
		dBodySetRotation(Rama[2].body,R4);
		dBodySetPosition (Rama[2].body,q,x,y2+radius+0.5*wysokosc+WymiaryPoprz[2]);
		Rama[2].geom = dCreateBox(space,WymiaryPoprz[0],WymiaryRama[1],WymiaryPoprz[2]);
		dGeomSetBody(Rama[2].geom,Rama[2].body);

			
	dMass z8;
		dMassSetZero (&z8);      
		Rama[3].body = dBodyCreate (world);
		dMassSetBox (&z8,DENSITY,WymiaryPoprz[0],WymiaryRama[1],WymiaryPoprz[2]);
		dBodySetMass (Rama[3].body,&z8);
		dRFromAxisAndAngle(R5,1,0,0,-kat);
		dBodySetRotation(Rama[3].body,R5);
		dBodySetPosition (Rama[3].body,q,x+WymiaryRama[1]*cos(kat)+WymiaryRama[2],y2+radius+0.5*wysokosc+WymiaryPoprz[2]);
		Rama[3].geom = dCreateBox(space,WymiaryPoprz[0],WymiaryRama[1],WymiaryPoprz[2]);
		dGeomSetBody(Rama[3].geom,Rama[3].body);

	joint[4] = dJointCreateFixed(world, 0);
        dJointAttach(joint[4], Rama[2].body, Rama[3].body);
		dJointSetFixed(joint[4]);

	joint[5] = dJointCreateFixed(world, 0);
        dJointAttach(joint[5], Rama[3].body, RamaP[0].body);
		dJointSetFixed(joint[5]);
		
}

void Car::MakeCar(int licznik)

{

	dReal x,y,q;
	q=0-przesuniecie*licznik+((platforma_sides[0]/2)-1);
	x=odlp[0];
	y=wysp[0]+platforma_sides[2]/2;

	dMass m; 
		dMassSetZero (&m);       
		Opona[0].body = dBodyCreate (world);
		dMassSetCylinder(&m,DENSITY,3,radius,length);
		dBodySetMass (Opona[0].body,&m);
		dRFromAxisAndAngle(R,0,1,0, pi/2);
		dBodySetPosition(Opona[0].body,q-0.1,x,y+radius);
		dBodySetRotation(Opona[0].body,R);
		Opona[0].geom=dCreateCylinder(space,radius,length);
		dGeomSetBody(Opona[0].geom,Opona[0].body);
		//dBodySetTorque(Opona[0].body,-150,0,0);

	dMass z; 
		dMassSetZero (&z);       
		Opona[1].body = dBodyCreate (world);
		dMassSetCylinder(&z,DENSITY,3,radius,length);
		dBodySetMass (Opona[1].body,&z);
		dRFromAxisAndAngle(R,0,1,0, pi/2);
		dBodySetPosition(Opona[1].body,q-0.9,x,y+radius);
		dBodySetRotation(Opona[1].body,R);
		Opona[1].geom=dCreateCylinder(space,radius,length);
		dGeomSetBody(Opona[1].geom,Opona[1].body);
		//dBodySetTorque(Opona[1].body,-150,0,0);
	
	joint[0] = dJointCreateFixed(world, 0);
		dJointAttach(joint[0], Opona[0].body,Opona[1].body);
		dJointSetFixed(joint[0]);

	joint[1] = dJointCreateFixed(world, 0);
		dJointAttach(joint[1], Opona[2].body,Opona[3].body);
		dJointSetFixed(joint[1]);

	dMass z5;
		dMassSetZero (&z5);      
		RamaP[0].body = dBodyCreate (world);
		dMassSetBox (&z5,DENSITY,WymiaryPoprz[0],WymiaryPoprz[1],WymiaryPoprz[2]);
		dBodySetMass (RamaP[0].body,&z);
		dBodySetPosition (RamaP[0].body,q-0.5,x+(os/2),y+radius);
		RamaP[0].geom = dCreateBox(space,WymiaryPoprz[0],WymiaryPoprz[1],WymiaryPoprz[2]);
		dGeomSetBody(RamaP[0].geom,RamaP[0].body);
		
	MakeRama(licznik);

	dMass z3; 
		dMassSetZero (&z3);       
		Opona[2].body = dBodyCreate (world);
		dMassSetCylinder(&z3,DENSITY,3,radius,length);
		dBodySetMass (Opona[2].body,&z3);
		dRFromAxisAndAngle(R,0,1,0, pi/2);
		dBodySetPosition(Opona[2].body,q-0.10,x+os,y+radius);
		dBodySetRotation(Opona[2].body,R);
		Opona[2].geom=dCreateCylinder(space,radius,length);
		dGeomSetBody(Opona[2].geom,Opona[2].body);
		//dBodySetTorque(Opona[2].body,-150,0,0);


	dMass z4; 
		dMassSetZero (&z4);       
		Opona[3].body = dBodyCreate (world);
		dMassSetCylinder(&z4,DENSITY,3,radius,length);
		dBodySetMass (Opona[3].body,&z4);
		dRFromAxisAndAngle(R,0,1,0, pi/2);
		dBodySetPosition(Opona[3].body,q-0.9,x+os,y+radius);
		dBodySetRotation(Opona[3].body,R);
		Opona[3].geom=dCreateCylinder(space,radius,length);
		dGeomSetBody(Opona[3].geom,Opona[3].body);
		//dBodySetTorque(Opona[3].body,-150,0,0);

	joint2[1] = dJointCreateHinge2(world, 0);
		dJointAttach(joint2[1], Opona[0].body, RamaP[0].body);
		dJointSetHinge2Anchor(joint2[1],q-0.1,x,y+radius);
		dJointSetHinge2Axis1(joint2[1], 0,1,0);
		dJointSetHinge2Axis2(joint2[1],1, 0,0);

	joint2[2] = dJointCreateHinge2(world, 0);
		dJointAttach(joint2[2], Opona[1].body, RamaP[0].body);
		dJointSetHinge2Anchor(joint2[2],q-0.9,x,y+radius);
		dJointSetHinge2Axis1(joint2[2], 0,1,0);
		dJointSetHinge2Axis2(joint2[2], 1,0,0);

	joint2[0] = dJointCreateHinge(world, 0);
		dJointAttach(joint2[0], Opona[2].body, RamaP[0].body);
		dJointSetHingeAnchor(joint2[0],q-0.1,x+os,y+radius);
		dJointSetHingeAxis(joint2[0], 1,0,0);

	joint2[3] = dJointCreateHinge(world, 0);
		dJointAttach(joint2[3], Opona[3].body, RamaP[0].body);
		dJointSetHingeAnchor(joint2[3],q-0.9,x+os,y+radius);
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
   fn.path_to_textures = "C:/ODE/ode-0.9/drawstuff/textures";

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

	for (int l=0;l<Ilosc_Aut;l++)
	{
		losuj_parametry(l);
		samochod[l].MakeCar(l);
	}

	//MakeBall();

	dsSimulationLoop (argc,argv,1200,600,&fn);
	dWorldDestroy (world); 

	dCloseODE();           

   return 0;
}           