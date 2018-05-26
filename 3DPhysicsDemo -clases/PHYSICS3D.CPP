/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/


#include <windows.h>
#include <string>

#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include <time.h>

#include "physics3D.h"

#define ASSERT(a) {   bool cond = static_cast<bool>( a ); \
                         if( !(cond) )\
                        __debugbreak(); } 

using namespace std;

dynamic_world *pWorld = 0;

extern int unsigned WindowWidth;

extern vector3D edge_p0,edge_p1;
extern vector3D edge_q0,edge_q1;
extern vector3D last_col_point[4];
extern vector3D last_col_normal[4];

float GetTime( void );

int GravityActive = 1;
int FrictionActive = 1;
const float restitution=0.7f;
const float friction=0.3f;

//NOTE: this is actually the accelaration of gavity in m/s2, 
// gravity force is G=g*M
vector3D gravity(real(0),real(0),real(-20));
        
dynamic_world::~dynamic_world( void )
{
	ClearCollisions();
	delete currentRender;currentRender=0;
}

dynamic_world::dynamic_world(real sX, real sY, real sZ )
{
	sizeX=sX;
	sizeY=sY;
	sizeZ=sZ;
	currentRender = new thinkGL();
	if( !currentRender)return;
	currentRender->setWordlSize(VALUE_MAX(sZ, VALUE_MAX( sX, sY)));

	BuildWalls();
    BuildBodies();
    IntitilizeBodiesInWorld();	
}
void dynamic_world::BuildWalls()
{
    walls[0].Normal = vector3D(real(0),real(-1),real(0));
	walls[0].pos = -sizeX/2*walls[0].Normal;
    walls[1].Normal = vector3D(real(0),real(1),real(0));
	walls[1].pos = -sizeX/2*walls[1].Normal;

    walls[2].Normal = vector3D(real(-1),real(0),real(0));
	walls[2].pos = -sizeX/2*walls[2].Normal;
    walls[3].Normal = vector3D(real(1),real(0),real(0));
	walls[3].pos = -sizeX/2*walls[3].Normal;

    walls[4].Normal = vector3D(real(0),real(0),real(-1));
	walls[4].pos = -sizeX/2*walls[4].Normal;
    walls[5].Normal = vector3D(real(0),real(0),real(1));
	walls[5].pos = -sizeX/2*walls[5].Normal;
}

void dynamic_world::BuildBodies( void )
{
	string String;

    for(int i = 0; i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];

		Body.shape = SPHERE_SHAPE;
//		Body.shape = BOX_SHAPE;
	
		if( Body.shape==SPHERE_SHAPE)
		{
			real radius = GenerateUnitRandomReal()*1.0f;
//			if( radius < 0.95f)radius=0.3f;
			radius = VALUE_MAX(radius,0.6f);
//			radius = VALUE_MIN(radius,0.6f);
			Body.size=vector3D(radius,radius,radius);
			Body.radius=radius;

			real Density = real(1.0f);
			real Mass = real(4.0f/3.0f)*Density*PI*radius*radius*radius;
//			real Mass = real(4.0f/3.0f)*Density*PI*1.0f/(radius*radius*radius);

			Body.setMass(Mass);
			Body.InverseLocalInertiaTensor(0,0) = real(5.0f)/(real(2.0f)*(Mass*(radius*radius)));
			Body.InverseLocalInertiaTensor(1,1) = real(5.0f)/(real(2.0f)*(Mass*(radius*radius)));
			Body.InverseLocalInertiaTensor(2,2) = real(5.0f)/(real(2.0f)*(Mass*(radius*radius)));

			Body.restitution =restitution;
			Body.friction = friction;
			Body.color[0]=GenerateUnitRandomReal();
			Body.color[1]=GenerateUnitRandomReal();
			Body.color[2]=GenerateUnitRandomReal();
			Body.backupcolor[0]=Body.color[0];
			Body.backupcolor[1]=Body.color[1];
			Body.backupcolor[2]=Body.color[2];
		}
		else
		{
			ASSERT( strlen(" ERROR: BOX_SHAPE not supported in this version!\n")==0 );
		}
    }

}
void  dynamic_world::IntitilizeBodiesInWorld()
{
	float maxSpeed=25.0f;
//	CurrentState=0;
//	TargetState=1;
    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body::DynamicState &Source = bodies[i].dynamicState[CurrentState];
        rigid_body::DynamicState &Target = bodies[i].dynamicState[TargetState];

		Source.CMForce = vector3D();
		Source.Torque = vector3D();

		//assign random position
		real px = (GenerateUnitRandomReal()-0.5f)*sizeX;
		real py = (GenerateUnitRandomReal()-0.5f)*sizeY;
		real pz = (GenerateUnitRandomReal()-0.5f)*sizeZ;

		if( (px + bodies[i].radius) > sizeX)px-= bodies[i].radius;
		if( (py + bodies[i].radius) > sizeY)py-= bodies[i].radius;
		if( (pz + bodies[i].radius) > sizeZ)pz-= bodies[i].radius;
		if( (px - bodies[i].radius) < -sizeX)px+= bodies[i].radius;
		if( (py - bodies[i].radius) < -sizeY)py+= bodies[i].radius;
		if( (pz - bodies[i].radius) < -sizeZ)pz+= bodies[i].radius;


		vector3D position(px,py,pz);
//		vector3D position(0,0,-sizeZ*.2*i);
		Source.CMPosition=position;

		//assign random velocities
		real vx =(GenerateUnitRandomReal()-0.5f)*maxSpeed;
		real vy =(GenerateUnitRandomReal()-0.5f)*maxSpeed;
		real vz =(GenerateUnitRandomReal()-0.5f)*maxSpeed;

		vector3D velocity(vx,vy,vz);
//		vector3D velocity(0,0,0);
		Source.CMVelocity=velocity;

		vector3D w(0,0,0);
		Source.AngularMomentum = w;
		Source.AngularVelocity = w;

		Source.Orientation.setIdentity();

        Source.InverseWorldInertiaTensor = Source.Orientation *
                bodies[i].InverseLocalInertiaTensor *
                Transpose(Source.Orientation);

		Target = Source;
	}
}


void dynamic_world::Render( void )
{
	currentRender->clearBoxTransformList();
	currentRender->clearSphereTransformList();
    for(int i=0; i < NumberOfBodies;i++)
    {
        matrix3D& Orientation = bodies[i].dynamicState[CurrentState].Orientation;
        vector3D& CMPosition = bodies[i].dynamicState[CurrentState].CMPosition;

		thkTransform myGlTransform;
        BuildOpenGLTransformMatrix(Orientation,CMPosition,myGlTransform.aM);
		//if( bodies[i].shape==BOX_SHAPE)
		//{
		//	currentRender->addBoxTransform( myGlTransform);
		//}
		//else
		if( bodies[i].shape==SPHERE_SHAPE)
		{
			myGlTransform.radius=bodies[i].radius;
			myGlTransform.color[0]=bodies[i].color[0];
			myGlTransform.color[1]=bodies[i].color[1];
			myGlTransform.color[2]=bodies[i].color[2];
			currentRender->addSphereTransform( myGlTransform);
//			currentRender->addBoxTransform( myGlTransform);
		}
	}
	if( currentRender->bDrawCollisionInfo)
	{
		float color[3];
		color[0]=1.0f;color[1]=0.0f;color[2]=0.0f;
		currentRender->clearLinesList();
		//col edges
		//currentRender->addLine(edge_p0.getArray(), edge_p1.getArray(), color);
		//currentRender->addLine(edge_q0.getArray(), edge_q1.getArray(), color);
		//col normal
		color[0]=0.0f;color[1]=1.0f;color[2]=0.0f;
		vector3D end_point = last_col_normal[0] + last_col_point[0];
		currentRender->addLine(last_col_point[0].getArray(), end_point.getArray(), color);
		currentRender->clearPointsList();
		currentRender->addPoint(last_col_point[0].getArray(), color);
	}

//    currentRender->glColor3f(0.5f,0.5f,0.5f);
    char string[64];
	if( GravityActive)
	{
		sprintf_s(string,64, "Gravity ON");
	}
	else
	{
		sprintf_s(string,64, "Gravity OFF");
	}
    currentRender->printGLtext(WindowWidth -100,10,string,strlen(string));
	if( FrictionActive )
	{
		sprintf_s(string,64, "Friction ON");
	}
	else
	{
		sprintf_s(string,64, "Friction OFF");
	}
    currentRender->printGLtext(WindowWidth -100, 50,string,strlen(string));


	currentRender->Render();
}

void dynamic_world::ClearCollisions( void )
{
    for(size_t i = 0; i < activePairs.size(); ++i)
    {
        collision_pair* pair = activePairs[i];
		delete pair;
	}
	activePairs.clear();	
	CollisionState = Clear;
	currentRender->bDrawCollisionInfo=true;
	currentRender->clearDrawInfo();

}

void dynamic_world::ApplyForces( int state )
{
    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];
        rigid_body::DynamicState &dynState = Body.dynamicState[state];

        // clear forces
		//TODO: add default code for torques
        dynState.Torque = vector3D();
        dynState.CMForce = vector3D();

        if(GravityActive)
        {
			// because Gravity force is g*M
			dynState.CMForce += gravity * Body.getMass();
        }

		//wind
//		dynState.CMForce += vector3D(50,0,0);

    }
}   

void dynamic_world::UpdateCurrentState()
{
    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];
        rigid_body::DynamicState &dynamicState0 = Body.dynamicState[CurrentState];
        rigid_body::DynamicState &dynamicState1 = Body.dynamicState[TargetState];

		dynamicState0.AngularMomentum= dynamicState1.AngularMomentum;
		dynamicState0.AngularVelocity= dynamicState1.AngularVelocity;
		dynamicState0.CMPosition = dynamicState1.CMPosition;
		dynamicState0.CMVelocity = dynamicState1.CMVelocity;
		dynamicState0.InverseWorldInertiaTensor = dynamicState1.InverseWorldInertiaTensor;
		dynamicState0.Orientation = dynamicState1.Orientation;
		dynamicState0.Torque = dynamicState1.Torque;
	}
}
void dynamic_world::Integrate( real DeltaTime )
{
	const real airFriction=0.0005f;

    for(int i= 0;i < NumberOfBodies;i++)
    {
        rigid_body::DynamicState &Source =bodies[i].dynamicState[CurrentState];
        rigid_body::DynamicState &Target =bodies[i].dynamicState[TargetState];

		//TODO: try forward-backward Euler scheme
		//Euler integration uses current velocity, more accurrate
        Target.CMPosition = Source.CMPosition +
                DeltaTime * Source.CMVelocity;

        Target.CMVelocity = Source.CMVelocity +
                (DeltaTime * bodies[i].getInvMass()) * Source.CMForce;
		Target.CMVelocity *=(1.0f - airFriction);

		//direct intgation uses next velocity
//		Target.CMPosition = Source.CMPosition +
//                DeltaTime * Target.CMVelocity;

        Target.Orientation = Source.Orientation +
                DeltaTime *
                matrix3D(Source.AngularVelocity,matrix3D::SkewSymmetric) *
                Source.Orientation;

		OrthonormalizeOrientation(Target.Orientation);

        Target.InverseWorldInertiaTensor = Target.Orientation *
                bodies[i].InverseLocalInertiaTensor *
                Transpose(Target.Orientation);

        Target.AngularMomentum = Source.AngularMomentum + DeltaTime * Source.Torque;
		Target.AngularMomentum *=(1.0f - airFriction);

        Target.AngularVelocity = Target.InverseWorldInertiaTensor *
                Target.AngularMomentum;

		if( airFriction > Epsilon)
		{
			while( Target.AngularVelocity.squaredLength() > 1000.0f)
			{
				//damp it until acceptable
				Target.AngularMomentum *=(1.0f - airFriction);
				Target.AngularVelocity = Target.InverseWorldInertiaTensor *
					Target.AngularMomentum;
			}
		}
    }
}

dynamic_world::collision_state dynamic_world::CheckForWallCollisions( int state )
{
    float const DepthEpsilon = Epsilon*100.0f;//0.001f;

//TODO: allow multiple collision points with walls
	for(int WallIndex = 0;(WallIndex < 6);WallIndex++)
	{
		plane &Wall = walls[WallIndex];

		for(int BodyIndex = 0;(BodyIndex < NumberOfBodies) ;BodyIndex++)
		{
			rigid_body &Body = bodies[BodyIndex];
			rigid_body::DynamicState &dynamicState =
					Body.dynamicState[state];

			if( Body.shape==SPHERE_SHAPE)		
			{
				vector3D col_point = dynamicState.CMPosition -
					walls[WallIndex].Normal*Body.radius;
				vector3D R = col_point - dynamicState.CMPosition;

				real penetration = Dot(col_point - Wall.pos,Wall.Normal);
//				real penetration = Dot(col_point,Wall.Normal) + Wall.Offset;
				if( penetration < DepthEpsilon)
				{
					CollisionState = Colliding;						
					//prevent interpenetration with ground
					if( walls[WallIndex].Normal.getZ() > 0.99f)
						dynamicState.CMPosition-=penetration*walls[WallIndex].Normal; 

					collision_pair* pair = new collision_pair;
					if( pair)
					{
						pair->body1Index=BodyIndex;
						pair->body2Index=-1;
						pair->normal[0] = walls[WallIndex].Normal;
						pair->point[0] = col_point;
						pair->nPoints=1;
						pair->wall_Index = WallIndex;
					}
					activePairs.push_back(pair);
				}
			}
		}
	}

    return CollisionState;
}

dynamic_world::collision_state dynamic_world::CheckForInterCollisions( int state )
{
    for(int BodyIndex1 = 0;(BodyIndex1 < NumberOfBodies) ;BodyIndex1++)
    {
        rigid_body &Body1 = bodies[BodyIndex1];
        rigid_body::DynamicState &dynamicState1 =
                Body1.dynamicState[state];
		Bbox bbox1(dynamicState1.Orientation,dynamicState1.CMPosition,Body1.size);

		for (int BodyIndex2 = BodyIndex1 + 1; (BodyIndex2 < NumberOfBodies); BodyIndex2++)
		{
			rigid_body &Body2 = bodies[BodyIndex2];
			rigid_body::DynamicState &dynamicState2 =
				Body2.dynamicState[state];
			Bbox bbox2(dynamicState2.Orientation, dynamicState2.CMPosition, Body2.size);

			if (Body1.shape == SPHERE_SHAPE && Body2.shape == SPHERE_SHAPE)
			{
				//Spheres detection
				vector3D dist = dynamicState1.CMPosition - dynamicState2.CMPosition;
				real min_dist = (Body1.radius + Body2.radius);
				if (dist.squaredLength() < (min_dist*min_dist))
				{
					CollisionState = Colliding;
					collision_pair* pair = new collision_pair;
					if (pair)
					{
						pair->body1Index = BodyIndex1;
						pair->body2Index = BodyIndex2;
						pair->normal[0] = dist.normalize();
						pair->point[0] = dynamicState1.CMPosition - (pair->normal[0] * Body1.radius);
						pair->nPoints = 1;
						pair->wall_Index = -1;
					}
					activePairs.push_back(pair);
				}
			}
		}
	}
	return CollisionState;
}

void dynamic_world::ResolveCollisions( int dynamicStateIndex )
{
	//NOTE: there can be interpenetrions in the following cases
	// very fast moving/rotating objects
	// large bodies fall over smaller bodies
	// multiple bodies involved in a collsion
	// multiple bodies and walls involved in a collision
	// iterations arent enough to clear collisions

	unsigned int numCollisions= activePairs.size();
    for(size_t i = 0; i < numCollisions; ++i)
    {
        collision_pair* pair = activePairs[i];
		if( pair->wall_Index >= 0)
		{
			//TODO: iterate over several points like body-body collision
			//resolve wall collision
			rigid_body &Body = bodies[pair->body1Index];
			vector3D col_normal = pair->normal[0];//walls[pair->wall_Index].Normal;
			rigid_body::DynamicState &dynamicState = Body.dynamicState[dynamicStateIndex];

			vector3D col_point = pair->point[0];//dynamicState.aBoundingVertices[pair->vertex1Index];
			vector3D R = col_point - dynamicState.CMPosition;
			vector3D Velocity = dynamicState.CMVelocity +
				CrossProduct(dynamicState.AngularVelocity,R);
			real ImpulseNumerator = -(real(1) + restitution) *
				Dot(Velocity, col_normal);
			real ImpulseDenominator = Body.getInvMass() +
				Dot(CrossProduct(dynamicState.InverseWorldInertiaTensor *
									CrossProduct(R,col_normal),R),col_normal);

			real impn =  (ImpulseNumerator/ImpulseDenominator);
			if( impn < Epsilon)
			{
				//sticky impulse
				continue;
			}

			vector3D Impulse = impn * col_normal;

			// apply impulse to primary quantities
			dynamicState.CMVelocity += Body.getInvMass() * Impulse;
			dynamicState.AngularMomentum += CrossProduct(R,Impulse);
    
			// compute affected auxiliary quantities
			dynamicState.AngularVelocity = dynamicState.InverseWorldInertiaTensor *
				dynamicState.AngularMomentum;


			if( FrictionActive)
			{
				// apply friction			
				Velocity = dynamicState.CMVelocity +
					CrossProduct(dynamicState.AngularVelocity,R);
				vector3D tand = Velocity - Dot(Velocity,col_normal)*col_normal;
				tand.normalize();
				{
					Impulse = -Body.friction *impn * tand;
					// apply impulse to primary quantities
					dynamicState.CMVelocity += Body.getInvMass() * Impulse;
					dynamicState.AngularMomentum += CrossProduct(R,Impulse);
    
					// compute affected auxiliary quantities
					dynamicState.AngularVelocity = dynamicState.InverseWorldInertiaTensor *
						dynamicState.AngularMomentum;
				}
			}

		}
		else
		{
			//resolve bodies collision
			rigid_body &body1 = bodies[pair->body1Index];
			rigid_body &body2 = bodies[pair->body2Index];
			rigid_body::DynamicState &dynamicState1 = body1.dynamicState[dynamicStateIndex];
			rigid_body::DynamicState &dynamicState2 = body2.dynamicState[dynamicStateIndex];

			real restitutionCoeff =  (body1.restitution + body2.restitution)*.5f;
			int ite =0;
			while( ite < 100)//max number of iteration allowed
			{
				for( int j=0; j < pair->nPoints; j++)
				{
					vector3D col_normal = pair->normal[j];//walls[pair->wall_Index].Normal;
					vector3D col_point = pair->point[j];//dynamicState.aBoundingVertices[pair->vertex1Index];
					vector3D R1 = col_point - dynamicState1.CMPosition;
					vector3D R2 = col_point - dynamicState2.CMPosition;

					vector3D Velocity1 = dynamicState1.CMVelocity +
						CrossProduct(dynamicState1.AngularVelocity,R1);
					vector3D Velocity2 = dynamicState2.CMVelocity +
						CrossProduct(dynamicState2.AngularVelocity,R2);

					vector3D relative_velocity = Velocity1 - Velocity2;
		//		    vector3D relative_velocity = Velocity1;
					real vn = Dot( relative_velocity, col_normal);
					if( vn > Epsilon)
					{
						//false collision
						continue;
					}

					real rIrn1 = body1.getInvMass() +
						Dot( col_normal,CrossProduct( dynamicState1.InverseWorldInertiaTensor*
						CrossProduct(R1, col_normal), R1)); 
					real rIrn2 = body2.getInvMass() +
						Dot( col_normal,CrossProduct( dynamicState2.InverseWorldInertiaTensor*
						CrossProduct(R2, col_normal), R2)); 
					real impn = -((real(1)+restitutionCoeff)*vn)/(rIrn1+rIrn2);
		//			real impn = -((r(1)+restitutionCoeff)*vn)/rIrn1;
					if( impn < Epsilon)
					{
						//sticky impulse
						continue;
					}

					vector3D Impulse = impn*col_normal;

					dynamicState1.CMVelocity += body1.getInvMass() * Impulse;
					dynamicState1.AngularMomentum += CrossProduct(R1,Impulse);
    
					// compute affected auxiliary quantities
					dynamicState1.AngularVelocity = dynamicState1.InverseWorldInertiaTensor *
						dynamicState1.AngularMomentum;

					dynamicState2.CMVelocity -= body2.getInvMass() * Impulse;
					dynamicState2.AngularMomentum -= CrossProduct(R2,Impulse);
    
					 //compute affected auxiliary quantities
					dynamicState2.AngularVelocity = dynamicState2.InverseWorldInertiaTensor *
						dynamicState2.AngularMomentum;

					if( FrictionActive)//warnning: increase interpenetrations becouse of no contact step
					{
						// apply friction			
						vector3D Velocity1 = dynamicState1.CMVelocity +
							CrossProduct(dynamicState1.AngularVelocity,R1);
						vector3D Velocity2 = dynamicState2.CMVelocity +
							CrossProduct(dynamicState2.AngularVelocity,R2);

						vector3D velocity = Velocity1 - Velocity2;

						vector3D tand = velocity - Dot(velocity,col_normal)*col_normal;
						tand.normalize();
						{
							real friction =  (body1.friction+  body2.friction)*.5f;
							Impulse = friction *(-impn) * tand;

							// apply impulse to primary quantities
							dynamicState1.CMVelocity += body1.getInvMass() * Impulse;
							dynamicState1.AngularMomentum += CrossProduct(R1,Impulse);
    
							// compute affected auxiliary quantities
							dynamicState1.AngularVelocity = dynamicState1.InverseWorldInertiaTensor *
								dynamicState1.AngularMomentum;
							dynamicState2.CMVelocity -= body2.getInvMass() * Impulse;
							dynamicState2.AngularMomentum -= CrossProduct(R2,Impulse);
    
							// compute affected auxiliary quantities
							dynamicState2.AngularVelocity = dynamicState2.InverseWorldInertiaTensor *
								dynamicState2.AngularMomentum;
						}
					}

				}
				int solvedCounter=0;
				for( int j=0; j < pair->nPoints; j++)
				{
					vector3D normal = pair->normal[j];//walls[pair->wall_Index].Normal;
					vector3D Position = pair->point[j];//dynamicState.aBoundingVertices[pair->vertex1Index];
					vector3D R1 = Position - dynamicState1.CMPosition;
					vector3D R2 = Position - dynamicState2.CMPosition;

					vector3D Velocity1 = dynamicState1.CMVelocity +
						CrossProduct(dynamicState1.AngularVelocity,R1);
					vector3D Velocity2 = dynamicState2.CMVelocity +
						CrossProduct(dynamicState2.AngularVelocity,R2);

					vector3D relative_velocity = Velocity1 - Velocity2;
		//		    vector3D relative_velocity = Velocity1;
					real vn = Dot( relative_velocity, normal);
					if( vn > Epsilon)
					{
						solvedCounter++;
					}
				}
				if( solvedCounter >=pair->nPoints)
				{
					break;
				}
				else
				{
					ite++;
//					assert( ite < 2);
				}
			}
		}
	}
}
void dynamic_world::printDT( real DeltaTime, int ite )
{
    char string[128];

	sprintf_s(string,128, "%.3f %d",DeltaTime, ite);

    currentRender->printGLtext(WindowWidth -100,200,string,strlen(string));
}

rigid_body* dynamic_world::getFirstHitBody(const vector3D pos, const vector3D dir, vector3D& hitpoint)
{

	rigid_body* hitBody = 0;
	int numHits=0;
	real hitDist, nearDist=REAL_MAX;
	vector3D nearHitPoint;
	vector3D hits[2];

    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];
        rigid_body::DynamicState &dynamicState =Body.dynamicState[0];

		if(  Body.shape==SPHERE_SHAPE)
		{
			if( rayfindSphere( pos, dir, dynamicState.CMPosition,Body.radius, numHits,hits))
			{
				hitDist = (hits[0]-pos).squaredLength();
				if( hitDist  <  nearDist)
				{
					hitBody = &Body;
					nearDist = hitDist;
				}
			}
		}
	}
	if( hitBody)
	{
		hitpoint = hits[0];
	}

	return hitBody;
}

void dynamic_world::Step( real DeltaTime )
{
    real CurrentTime = real(0);
    real TargetTime = DeltaTime;

		//move bodies
    ApplyForces(CurrentState);

    Integrate(TargetTime-CurrentTime);

	//manage collisions
	ClearCollisions();

    CheckForWallCollisions(TargetState);
    CheckForInterCollisions(TargetState);
    // either colliding or clear
    if(CollisionState == Colliding )
    {
        ResolveCollisions(TargetState);
    }
            
	UpdateCurrentState();

}


