/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <vector>

#if !defined(PHYSICS3D_H)
#define PHYSICS3D_H

#include "geo3d.h"	
#include "thinkGL.h"

enum BVolume
{
   BOX_SHAPE,
   SPHERE_SHAPE
};

//TODO: point-normal representation
struct plane
{
    vector3D Normal;        
    vector3D pos;        
//    real Offset;                 
};

const int  MaxNumberOfBoundingVertices = 50;
struct rigid_body
{
	//geometric properties
	BVolume shape;
	vector3D size;
	real radius;
	float color[3];
	float backupcolor[3];
	//permanent dynamic properties
    real mass;
    matrix3D InverseLocalInertiaTensor;
    real restitution;
    real friction;	

    struct DynamicState
    {
		//variable dynamic properties
        vector3D CMPosition;
        matrix3D Orientation;

        vector3D CMVelocity;
        vector3D AngularMomentum;

        vector3D CMForce;
        vector3D Torque;

        matrix3D InverseWorldInertiaTensor;
        vector3D AngularVelocity;
        
    } dynamicState[2];

	void setMass(real value){ mass=value;}
	real getMass(){ return mass;}
	real getInvMass(){ return (1.0f/mass);}

	void applyImpulse(const vector3D impulse, const vector3D point, int state){
	    dynamicState[state].CMVelocity += getInvMass() * (impulse);

	    vector3D r = (point - dynamicState[state].CMPosition);
    	dynamicState[state].AngularMomentum += CrossProduct(r,impulse);
	    dynamicState[state].AngularVelocity =  dynamicState[state].InverseWorldInertiaTensor * dynamicState[state].AngularMomentum;
    }


};

struct collision_pair
{
	int body1Index;
	int body2Index;
	int wall_Index;
	int nPoints;
	vector3D point[16];
	vector3D normal[16];	
};

const     int CurrentState=0;
const     int TargetState=1;
const     int NumberOfBodies =  16;
	
class dynamic_world
{
public:

    void BuildWalls();
    void BuildBodies( void );
	void IntitilizeBodiesInWorld();
    void InitializeRendering( void );
	void ResetBodiesPosition();
	void ClearCollisions( void );

	void Step( real DeltaTime );
    void Render( void );
	void printDT( real DeltaTime, int ite );
	rigid_body* getFirstHitBody(const vector3D pos, const vector3D dir, vector3D& hitpoint);

	thinkGL* getRender(){ return currentRender;}

    dynamic_world( real sX, real sY, real sZ );    
    ~dynamic_world( void );

private:

	real sizeX, sizeY, sizeZ;

    enum collision_state
    {
        Clear,
        Colliding
    } CollisionState;	


    void ApplyForces( int state );
    void Integrate( real DeltaTime );
    void UpdateCurrentState();
    collision_state CheckForInterCollisions( int state );
    collision_state CheckForWallCollisions( int state );
    void ResolveCollisions( int ConfigurationIndex );

    rigid_body bodies[NumberOfBodies];
	plane walls[6];

    std::vector<collision_pair*> activePairs;

	thinkGL* currentRender;

};

#endif
	