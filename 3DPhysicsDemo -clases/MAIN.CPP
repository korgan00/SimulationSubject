/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <windows.h>

#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <mmsystem.h>

#include "geo3d.h"
#include "physics3D.h"
#include "logout.h"

#define FIXEDTIMESTEP 1
const float FPS = 25;
const int substep = 5;

extern int unsigned WindowWidth;
extern int unsigned WindowHeight;
extern int GravityActive;
extern int FrictionActive;
extern dynamic_world *pWorld;

FILE* logstr=0;

float GetTime( void )
{
#if 0   
    static clock_t StartTime = clock();
    assert(StartTime != (clock_t)(-1));
    return ((float)(clock()-StartTime))/((float)CLOCKS_PER_SEC);
#else
    static DWORD StartMilliseconds;
    if(!StartMilliseconds)
    {
        StartMilliseconds = timeGetTime();
    }

    DWORD CurrentMilliseconds = timeGetTime();
    return float(CurrentMilliseconds - StartMilliseconds) / 1000.0f;
#endif  
}


void Initialize( void )
{
	LOGOUT_START( logstr);
	LOGOUT( logstr, "--- fichero de depuracion logout ------------\n ");

    srand((unsigned int)time(0));

	//init world
    pWorld = new dynamic_world(9,9,9);
    assert(pWorld);
	//init render
	pWorld->getRender()->Init(WindowWidth, WindowHeight);
}

void selectObjectAtPoint( int xpos, int ypos)
{
	if( !pWorld)return;

	double nearx,neary, nearz;
	double farx,fary, farz;

	pWorld->getRender()->getRayThrough(xpos,ypos, 
						   nearx, neary, nearz,
						   farx, fary, farz);

	vector3D pos0(nearx,neary, nearz);
	vector3D  dir(farx,fary, farz);
	dir -=pos0;
	dir.normalize();

	vector3D hitpoint(0,0,0);
	rigid_body* hitbody = pWorld->getFirstHitBody(pos0, dir, hitpoint);

	if( hitbody)
	{
		//push the body
		hitbody->applyImpulse(dir*25.0f, hitpoint, 0);
	}

}

void Execute(float zoom, float height, float angle)
{
	static int counter=0;
	counter++;


	assert(pWorld);
	pWorld->getRender()->clearView(zoom, height, angle);

	static real LastTime = GetTime();

	real const dt = 1.0f / (substep*FPS);

//#define FIXEDTIMESTEP 1
#ifdef FIXEDTIMESTEP
	real Time = LastTime;
	
	pWorld->Step(dt);

	LastTime = Time + dt;

#else
	real Time = GetTime();
	real elapsed_dt = Time - LastTime;	
	int ite = (int)(elapsed_dt / dt);
	real last_dt = elapsed_dt - ite*dt;
	for (int i = 0; i < ite; i++) pWorld->Step(dt);
	if(last_dt > Epsilon)pWorld->Step(last_dt);

	LOGOUT( logstr, " counter %d elapsed_dt %.3f last_dt %.3f ite %d \n", counter, elapsed_dt, last_dt, ite);
//	pWorld->printDT( elapsed_dt, ite );

	LastTime = GetTime();

#endif


	pWorld->Render();
}

void Reset( void )
{
	pWorld->ClearCollisions();
	pWorld->IntitilizeBodiesInWorld();		
}


void Destroy( void )
{
    delete pWorld;
}

void ToggleGravity( void )
{
    GravityActive = GravityActive ? 0 : 1;
}

void ToggleFriction( void )
{
    FrictionActive = FrictionActive ? 0 : 1;
}

