/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include "geo3d.h"

//*** allow double to float conversion
#pragma warning ( disable : 4136 )
#pragma warning ( disable : 4244 )
//*** Turn on float conversion warning
#pragma warning ( default : 4136 )
#pragma warning ( disable : 4514 ) // unused inline function

#include <assert.h>
#include <math.h>
#include <stdlib.h>


bool bDrawCollisionInfo=false;
vector3D last_col_point[4];
vector3D last_col_normal[4];
vector3D edge_p0,edge_p1;
vector3D edge_q0,edge_q1;

matrix3D operator*( matrix3D const &Operand,
        matrix3D const &Multiplier )
{
    matrix3D Result;

    for(int i = 0;i < 3;i++)
    {
        for(int j = 0;j < 3;j++)
        {
            real Value = 0;
          
            for(int k = 0;k < 3;k++)
            {
                Value += Operand.GetElement(i,k) *
                        Multiplier.GetElement(k,j);
            }

            Result.SetElement(i,j,Value);
        }
    }

    return Result;
}

vector3D operator*( matrix3D const &Operand,
        vector3D const &Multiplier )
{
    vector3D Result;

    for(int i = 0;i < 3;i++)
    {
        real Value = 0;
      
        for(int k = 0;k < 3;k++)
        {
            Value += Operand.GetElement(i,k) *
                    Multiplier(k);
        }

        Result.set(i,Value);
    }

    return Result;
}


matrix3D::matrix3D( void )
{
    for(int Counter = 0;Counter < 9;Counter++)
    {
        element[0][Counter] = 0;
    }

    element[0][0] = element[1][1] = element[2][2] = 1;
}

void matrix3D::setIdentity()
{
    for(int Counter = 0;Counter < 9;Counter++)
    {
        element[0][Counter] = 0;
    }

    element[0][0] = element[1][1] = element[2][2] = 1;
}


real vector3D::length( void ) const
{
    return sqrt(getX()*getX() + getY()*getY() + getZ()*getZ());
}

real vector3D::squaredLength( void ) const
{
    return (getX()*getX() + getY()*getY() + getZ()*getZ());
}

vector3D &vector3D::normalize( void )
{
    real Length = length();

    if(Length != 0)
    {
        setX(getX() / Length);
        setY(getY() / Length);
        setZ(getZ() / Length);
    }

    return *this;
}
  
void OrthonormalizeOrientation( matrix3D &Orientation )
{
    vector3D X(Orientation(0,0),Orientation(1,0),Orientation(2,0));
    vector3D Y(Orientation(0,1),Orientation(1,1),Orientation(2,1));
    vector3D Z;

    X.normalize();
    Z = CrossProduct(X,Y).normalize();
    Y = CrossProduct(Z,X).normalize();

    Orientation(0,0) = X(0); Orientation(0,1) = Y(0); Orientation(0,2) = Z(0);
    Orientation(1,0) = X(1); Orientation(1,1) = Y(1); Orientation(1,2) = Z(1);
    Orientation(2,0) = X(2); Orientation(2,1) = Y(2); Orientation(2,2) = Z(2);
}



real GenerateUnitRandomReal( void )
{
    real Random = ((real)rand())/((real)RAND_MAX);
    assert((Random >= real(0)) && (Random <= real(1)));
    return Random;
}


real sqrDistanceSegs(const vector3D& p0, const vector3D& p1,
                    const vector3D& q0, const vector3D& q1,
                    real& pfSegP0, real& pfSegP1)
{
    vector3D dirP = p1 - p0;
    vector3D dirQ = q1 - q0; 
    vector3D kDiff = p0 - q0;
    real fA00 = dirP.squaredLength();
    real fA01 = -Dot( dirP, dirQ);
    real fA11 = dirQ.squaredLength();
    real fB0 = Dot( kDiff, dirP);
    real fC = kDiff.squaredLength();
    real fDet = fabs(fA00*fA11-fA01*fA01);
    real fB1, fS, fT, fSqrDist, fTmp;

    if ( fDet > Epsilon )
    {
        // line segments are not parallel
        fB1 = -Dot(kDiff, dirQ);
        fS = fA01*fB1-fA11*fB0;
        fT = fA01*fB0-fA00*fB1;
        
        if ( fS > (-Epsilon) )
        {
            if ( fS < fDet )
            {
                if ( fT >= (-Epsilon) )
                {
                    if ( fT < fDet )  // region 0 (interior)
                    {
                        // minimum at two interior points of 3D lines
                        real fInvDet = 1.0f/fDet;
                        fS *= fInvDet;
                        fT *= fInvDet;
                        fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
                            fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
                    }
                    else  // region 3 (side)
                    {
                        fT = 1.0f;
                        fTmp = fA01+fB0;
                        if ( fTmp > (-Epsilon) )
                        {
                            fS = 0.0f;
                            fSqrDist = fA11+2.0f*fB1+fC;
                        }
                        else if ( (-fTmp) > fA00 )
                        {
                            fS = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
                        }
                        else
                        {
                            fS = -fTmp/fA00;
                            fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                        }
                    }
                }
                else  // region 7 (side)
                {
                    fT = 0.0f;
                    if ( fB0 > (-Epsilon) )
                    {
                        fS = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB0) >= fA00 )
                    {
                        fS = 1.0f;
                        fSqrDist = fA00+2.0f*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDist = fB0*fS+fC;
                    }
                }
            }
            else
            {
                if ( fT > (-Epsilon) )
                {
                    if ( fT < fDet )  // region 1 (side)
                    {
                        fS = 1.0f;
                        fTmp = fA01+fB1;
                        if ( fTmp > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fA00+2.0f*fB0+fC;
                        }
                        else if ( (-fTmp) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                        }
                        else
                        {
                            fT = -fTmp/fA11;
                            fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                        }
                    }
                    else  // region 2 (corner)
                    {
                        fTmp = fA01+fB0;
                        if ( (-fTmp) < fA00 )
                        {
                            fT = 1.0f;
                            if ( fTmp > (-Epsilon) )
                            {
                                fS = 0.0f;
                                fSqrDist = fA11+2.0f*fB1+fC;
                            }
                            else
                            {
                                 fS = -fTmp/fA00;
                                 fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                            }
                        }
                        else
                        {
                            fS = 1.0f;
                            fTmp = fA01+fB1;
                            if ( fTmp > (-Epsilon) )
                            {
                                fT = 0.0f;
                                fSqrDist = fA00+2.0f*fB0+fC;
                            }
                            else if ( (-fTmp) > fA11 )
                            {
                                fT = 1.0f;
                                fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                            }
                            else
                            {
                                fT = -fTmp/fA11;
                                fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                            }
                        }
                    }
                }
                else  // region 8 (corner)
                {
                    if ( (-fB0) < fA00 )
                    {
                        fT = 0.0f;
                        if ( fB0 > (-Epsilon) )
                        {
                            fS = 0.0f;
                            fSqrDist = fC;
                        }
                        else
                        {
                            fS = -fB0/fA00;
                            fSqrDist = fB0*fS+fC;
                        }
                    }
                    else
                    {
                        fS = 1.0f;
                        fTmp = fA01+fB1;
                        if ( fTmp > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fA00+2.0f*fB0+fC;
                        }
                        else if ( (-fTmp) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                        }
                        else
                        {
                            fT = -fTmp/fA11;
                            fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                        }
                    }
                }
            }
        }
        else 
        {
            if ( fT > (-Epsilon) )
            {
                if ( fT < fDet )  // region 5 (side)
                {
                    fS = 0.0f;
                    if ( fB1 > 0.0f )
                    {
                        fT = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB1) > fA11 )
                    {
                        fT = 1.0f;
                        fSqrDist = fA11+2.0f*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDist = fB1*fT+fC;
                    }
                }
                else  // region 4 (corner)
                {
                    fTmp = fA01+fB0;
                    if ( fTmp < 0.0f )
                    {
                        fT = 1.0f;
                        if ( (-fTmp) > fA00 )
                        {
                            fS = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
                        }
                        else
                        {
                            fS = -fTmp/fA00;
                            fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                        }
                    }
                    else
                    {
                        fS = 0.0f;
                        if ( fB1 > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fC;
                        }
                        else if ( (-fB1) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA11+2.0f*fB1+fC;
                        }
                        else
                        {
                            fT = -fB1/fA11;
                            fSqrDist = fB1*fT+fC;
                        }
                    }
                }
            }
            else   // region 6 (corner)
            {
                if ( fB0 < (Epsilon) )
                {
                    fT = 0.0f;
                    if ( (-fB0) > fA00 )
                    {
                        fS = 1.0f;
                        fSqrDist = fA00+2.0f*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDist = fB0*fS+fC;
                    }
                }
                else
                {
                    fS = 0.0f;
                    if ( fB1 > (-Epsilon) )
                    {
                        fT = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB1) > fA11 )
                    {
                        fT = 1.0f;
                        fSqrDist = fA11+2.0f*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDist = fB1*fT+fC;
                    }
                }
            }
        }
    }
    else
    {
        // line segments are parallel
        if ( fA01 > (-Epsilon) )
        {
            // direction vectors form an obtuse angle
            if ( fB0 > (-Epsilon) )
            {
                fS = 0.0f;
                fT = 0.0f;
                fSqrDist = fC;
            }
            else if ( (-fB0) < fA00 )
            {
                fS = -fB0/fA00;
                fT = 0.0f;
                fSqrDist = fB0*fS+fC;
            }
            else
            {
                fB1 = -Dot( kDiff, dirQ);
                fS = 1.0f;
                fTmp = fA00+fB0;
                if ( (-fTmp) > fA01 )
                {
                    fT = 1.0f;
                    fSqrDist = fA00+fA11+fC+2.0f*(fA01+fB0+fB1);
                }
                else
                {
                    fT = -fTmp/fA01;
                    fSqrDist = fA00+2.0f*fB0+fC+fT*(fA11*fT+2.0f*(fA01+fB1));
                }
            }
        }
        else
        {
            // direction vectors form an acute angle
            if ( (-fB0) > fA00 )
            {
                fS = 1.0f;
                fT = 0.0f;
                fSqrDist = fA00+2.0f*fB0+fC;
            }
            else if ( fB0 < (-Epsilon) )
            {
                fS = -fB0/fA00;
                fT = 0.0f;
                fSqrDist = fB0*fS+fC;
            }
            else
            {
                fB1 = -Dot(kDiff, dirQ);
                fS = 0.0f;
                if ( fB0 > -fA01 )
                {
                    fT = 1.0f;
                    fSqrDist = fA11+2.0f*fB1+fC;
                }
                else
                {
                    fT = -fB0/fA01;
                    fSqrDist = fC+fT*(2.0f*fB1+fA11*fT);
                }
            }
        }
    }

     pfSegP0 = fS;
     pfSegP1 = fT;

    return fabs(fSqrDist);
}

bool SphereTestBox(const vector3D& center, const real radius, const Bbox& q)
{

	real dotX = Dot( center - q.center, q.getXaxis());
	real dotY = Dot( center - q.center, q.getYaxis());
	real dotZ = Dot( center - q.center, q.getZaxis());

	real myColMargin=Epsilon;

	real distX = q.size.getX() + myColMargin + radius - fabs(dotX);
	real distY = q.size.getY() + myColMargin + radius - fabs(dotY);
	real distZ = q.size.getZ() + myColMargin + radius - fabs(dotZ);
	if( distX  > Epsilon && distY > Epsilon && distZ  > Epsilon)
	{
		return true;
	}

	return false;
}
bool SphereFindBox(const vector3D& center, const real radius, const Bbox& q, vector3D&col_point, vector3D& col_normal)
{
   //get box closest point to sphere center by clamping
	real myColMargin=Epsilon;

	real dotX = Dot( center - q.center, q.getXaxis());
	real dotY = Dot( center - q.center, q.getYaxis());
	real dotZ = Dot( center - q.center, q.getZaxis());

	real boxmaxX = q.size.getX(); 
	real boxmaxY = q.size.getY(); 
	real boxmaxZ = q.size.getZ(); 

	real boxminX = - q.size.getX(); 
	real boxminY = - q.size.getY(); 
	real boxminZ = - q.size.getZ(); 

   real x = VALUE_MAX(boxminX, VALUE_MIN(dotX, boxmaxX));
   real y = VALUE_MAX(boxminY, VALUE_MIN(dotY, boxmaxY));
   real z = VALUE_MAX(boxminZ, VALUE_MIN(dotZ, boxmaxZ));

  // distance from point over cube to center is the same in locals coords or world coord!
  real distance = (vector3D(x,y,z) -  vector3D(dotX,dotY,dotZ)).squaredLength();
  
  if( (distance - myColMargin) < (radius*radius))
  {
	  //collision
	  col_point = x*q.getXaxis() + y*q.getYaxis() + z*q.getZaxis();
	  col_point += q.center;

	  // TODO: if sphrere center is inside the box col_normal is undefined
	  col_normal =  center - col_point;
	  col_normal.normalize();

	  return true;
  }
  else
  {	
	//sphere and box  arent really colliding
	return false;
  }
}

bool boxTestBox(const Bbox& p, const Bbox& q)
{
// Separation-Axis theorem

    // compute difference of box centers, D = C1-C0
    vector3D kD = q.center - p.center;

    real aafC[3][3];     // matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
    real aafAbsC[3][3];  // |c_{ij}|
    real afAD[3];        // Dot(A_i,D)
    real fR0, fR1, fR;   // interval radii and distance between centers
    real fR01;           // = R0 + R1
    
    // axis C0+t*A0
    aafC[0][0] = Dot( p.getXaxis(), q.getXaxis());
    aafC[0][1] = Dot( p.getXaxis(), q.getYaxis());
    aafC[0][2] = Dot( p.getXaxis(), q.getZaxis());
    afAD[0] = Dot( p.getXaxis(), kD);
    aafAbsC[0][0] = fabs(aafC[0][0]);
    aafAbsC[0][1] = fabs(aafC[0][1]);
    aafAbsC[0][2] = fabs(aafC[0][2]);
    fR = fabs(afAD[0]);
    fR1 = q.size.getX()*aafAbsC[0][0]+q.size.getY()*aafAbsC[0][1]+q.size.getZ()*aafAbsC[0][2];
    fR01 = p.size.getX() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1
    aafC[1][0] = Dot( p.getYaxis(), q.getXaxis());
    aafC[1][1] = Dot( p.getYaxis(), q.getYaxis());
    aafC[1][2] = Dot( p.getYaxis(), q.getZaxis());
    afAD[1] = Dot( p.getYaxis(), kD);
    aafAbsC[1][0] = fabs(aafC[1][0]);
    aafAbsC[1][1] = fabs(aafC[1][1]);
    aafAbsC[1][2] = fabs(aafC[1][2]);
    fR = fabs(afAD[1]);
    fR1 = q.size.getX()*aafAbsC[1][0]+q.size.getY()*aafAbsC[1][1]+q.size.getZ()*aafAbsC[1][2];
    fR01 = p.size.getY() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2
    aafC[2][0] = Dot( p.getZaxis(), q.getXaxis());
    aafC[2][1] = Dot( p.getZaxis(), q.getYaxis());
    aafC[2][2] = Dot( p.getZaxis(), q.getZaxis());
    afAD[2] = Dot( p.getZaxis(), kD);
    aafAbsC[2][0] = fabs(aafC[2][0]);
    aafAbsC[2][1] = fabs(aafC[2][1]);
    aafAbsC[2][2] = fabs(aafC[2][2]);
    fR = fabs(afAD[2]);
    fR1 = q.size.getX()*aafAbsC[2][0]+q.size.getY()*aafAbsC[2][1]+q.size.getZ()*aafAbsC[2][2];
    fR01 = p.size.getZ() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B0
    fR = fabs(Dot( q.getXaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][0]+p.size.getY()*aafAbsC[1][0]+p.size.getZ()*aafAbsC[2][0];
    fR01 = fR0 + q.size.getX();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B1
    fR = fabs(Dot( q.getYaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][1]+p.size.getY()*aafAbsC[1][1]+p.size.getZ()*aafAbsC[2][1];
    fR01 = fR0 + q.size.getY();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B2
    fR = fabs(Dot( q.getZaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][2]+p.size.getY()*aafAbsC[1][2]+p.size.getZ()*aafAbsC[2][2];
    fR01 = fR0 + q.size.getZ();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB0
    fR = fabs(afAD[2]*aafC[1][0]-afAD[1]*aafC[2][0]);
    fR0 = p.size.getY()*aafAbsC[2][0] + p.size.getZ()*aafAbsC[1][0];
    fR1 = q.size.getY()*aafAbsC[0][2] + q.size.getZ()*aafAbsC[0][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB1
    fR = fabs(afAD[2]*aafC[1][1]-afAD[1]*aafC[2][1]);
    fR0 = p.size.getY()*aafAbsC[2][1] + p.size.getZ()*aafAbsC[1][1];
    fR1 = q.size.getX()*aafAbsC[0][2] + q.size.getZ()*aafAbsC[0][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB2
    fR = fabs(afAD[2]*aafC[1][2]-afAD[1]*aafC[2][2]);
    fR0 = p.size.getY()*aafAbsC[2][2] + p.size.getZ()*aafAbsC[1][2];
    fR1 = q.size.getX()*aafAbsC[0][1] + q.size.getY()*aafAbsC[0][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB0
    fR = fabs(afAD[0]*aafC[2][0]-afAD[2]*aafC[0][0]);
    fR0 = p.size.getX()*aafAbsC[2][0] + p.size.getZ()*aafAbsC[0][0];
    fR1 = q.size.getY()*aafAbsC[1][2] + q.size.getZ()*aafAbsC[1][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB1
    fR = fabs(afAD[0]*aafC[2][1]-afAD[2]*aafC[0][1]);
    fR0 = p.size.getX()*aafAbsC[2][1] + p.size.getZ()*aafAbsC[0][1];
    fR1 = q.size.getX()*aafAbsC[1][2] + q.size.getZ()*aafAbsC[1][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB2
    fR = fabs(afAD[0]*aafC[2][2]-afAD[2]*aafC[0][2]);
    fR0 = p.size.getX()*aafAbsC[2][2] + p.size.getZ()*aafAbsC[0][2];
    fR1 = q.size.getX()*aafAbsC[1][1] + q.size.getY()*aafAbsC[1][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB0
    fR = fabs(afAD[1]*aafC[0][0]-afAD[0]*aafC[1][0]);
    fR0 = p.size.getX()*aafAbsC[1][0] + p.size.getY()*aafAbsC[0][0];
    fR1 = q.size.getY()*aafAbsC[2][2] + q.size.getZ()*aafAbsC[2][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB1
    fR = fabs(afAD[1]*aafC[0][1]-afAD[0]*aafC[1][1]);
    fR0 = p.size.getX()*aafAbsC[1][1] + p.size.getY()*aafAbsC[0][1];
    fR1 = q.size.getX()*aafAbsC[2][2] + q.size.getZ()*aafAbsC[2][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB2
    fR = fabs(afAD[1]*aafC[0][2]-afAD[0]*aafC[1][2]);
    fR0 = p.size.getX()*aafAbsC[1][2] + p.size.getY()*aafAbsC[0][2];
    fR1 = q.size.getX()*aafAbsC[2][1] + q.size.getY()*aafAbsC[2][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    return true;
}

void box2vertex(const Bbox& _box, vector3D* vertexList)
{	
  static vector3D sizeX;
  static vector3D sizeY;
  static vector3D sizeZ;

  sizeX = vector3D( _box.orientation(0,0),_box.orientation(1,0),_box.orientation(2,0));  
  sizeY = vector3D( _box.orientation(0,1),_box.orientation(1,1),_box.orientation(2,1));  
  sizeZ = vector3D( _box.orientation(0,2),_box.orientation(1,2),_box.orientation(2,2));  

  sizeX *=_box.size.getX();
  sizeY *=_box.size.getY();
  sizeZ *=_box.size.getZ();

  vertexList[0]=sizeX + sizeY + sizeZ;
  vertexList[1]=sizeX + sizeY - sizeZ;
  vertexList[2]=sizeX - sizeY -	sizeZ;
  vertexList[3]=sizeX - sizeY + sizeZ;
  vertexList[4]= -vertexList[0];
  vertexList[5]= -vertexList[3];
  vertexList[6]= -vertexList[2];
  vertexList[7]= -vertexList[1];
  vertexList[0]+=_box.center;	
  vertexList[1]+=_box.center;	
  vertexList[2]+=_box.center;	
  vertexList[3]+=_box.center;	
  vertexList[4]+=_box.center;	
  vertexList[5]+=_box.center;	
  vertexList[6]+=_box.center;	
  vertexList[7]+=_box.center;	
}

bool boxFindBox(const Bbox& p, const Bbox& q, vector3D* col_point, vector3D* col_normal, int& nPoints)
{
	static int counter =0;
	counter++;

//	assert(counter<20);

	real myColMargin = (Epsilon*10000.0f);

	nPoints=0;
	vector3D pvertexes[8];
	vector3D qvertexes[8];

	box2vertex(p, pvertexes);
	box2vertex(q, qvertexes);

	edge_p0 = vector3D();
	edge_p1 = vector3D();
	edge_q0 = vector3D();
	edge_q1 = vector3D();

	real depth_p[4], min_depth_p=REAL_MAX;
	real depth_q[4], min_depth_q=REAL_MAX;
	vector3D gess_point_p[4], gess_normal_p;
	vector3D gess_point_q[4], gess_normal_q;
	int min_vertex_p=-1, min_vertex_q=-1;
	int np=0, nq=0;

	//count vertexes of p  inside faces of q
	int gess_col_face_q=-1;
	int hits, min_hits=9;
	for( int j=0; j < 6; j++)
	{
		hits=0; 
		vector3D q_axis = q.axis(j);
		real extend = q.size(j%3);
		for( int k=0; k < 8; k++)
		{	
			if(Dot(pvertexes[k] -  q.center, q_axis) < (extend + myColMargin))hits++;					
		}
		if( hits < min_hits)
		{
			min_hits = hits;
			gess_col_face_q = j;
		}
	}					
			
	if( gess_col_face_q >=0)
	{
		//choose the most penetraing point in face of q
		real extend = q.size(gess_col_face_q%3);
		gess_normal_q = q.axis(gess_col_face_q);

		for( int i=0; i < 8; i++)
		{		
			real depth = Dot(pvertexes[i] -  q.center, gess_normal_q);
			if( depth > Epsilon && depth< (extend + myColMargin))
			{
				gess_point_p[np] = pvertexes[i];
				depth_p[np] = depth;
				np++;
				if( depth < min_depth_p)
				{
					min_vertex_p = i;
					min_depth_p=depth;
				}
			}		
			if( np  >= 4)break;
		}
	}

	//count vertexes of q  inside faces of p
	int gess_col_face_p=-1;
	min_hits=9;
	for( int j=0; j < 6; j++)
	{
		hits=0; 
		vector3D p_axis = p.axis(j);
		real extend = p.size(j%3);
		for( int k=0; k < 8; k++)
		{	
			if(Dot(qvertexes[k] -  p.center, p_axis) < ( extend + myColMargin))hits++;					
		}
		if( hits < min_hits)
		{
			min_hits = hits;
			gess_col_face_p = j;
		}
	}					
			
	if( gess_col_face_p >=0)
	{
		//choose the most penetraing point in face of p
		real extend = p.size(gess_col_face_p%3);
		gess_normal_p = p.axis(gess_col_face_p);

		for( int i=0; i < 8; i++)
		{		
			real depth = Dot(qvertexes[i] -  p.center, gess_normal_p);
			if( depth > Epsilon && depth< (extend + myColMargin))
			{
				gess_point_q[nq] = qvertexes[i];
				depth_q[nq] = depth;
				nq++;
				if( depth < min_depth_q)
				{
					min_vertex_q = i;
					min_depth_q=depth;
				}
			}		
			if( nq  >= 4)break;
		}
	}

	//now collision point must be the less penatrating of p and q( ex: face colliding in a cortner)
	if(  np == 0 || ( nq > 0 && min_depth_q  > min_depth_p) )
	{
		// choose vertexes of q for futher testing
		// this text is aproximate so the result is true with a error margin
		int numVertexEqualDepth=0;
		for( int i=0; i < nq; i++)
		{		
			if( depth_q[i] < (min_depth_q + myColMargin))
			{
				numVertexEqualDepth++;
			}
		}
		if( numVertexEqualDepth >=3)
		{
			//face-face collision found!
			for( int i=0; i < nq; i++)
			{		
				if( depth_q[i] < (min_depth_q + myColMargin))
				{
					//clamp external vertices of q over p
					vector3D v = gess_point_q[i];
					real dotX = Dot( v - p.center, p.getXaxis());
					real dotY = Dot( v - p.center, p.getYaxis());
					real dotZ = Dot( v - p.center, p.getZaxis());

					real clampx = VALUE_MAX(- p.size.getX(), VALUE_MIN(dotX, p.size.getX()));
					real clampy = VALUE_MAX(- p.size.getY(), VALUE_MIN(dotY, p.size.getY()));
					real clampz = VALUE_MAX(- p.size.getZ(), VALUE_MIN(dotZ, p.size.getZ()));

					vector3D clamp_q = clampx*p.getXaxis() + clampy*p.getYaxis() + clampz*p.getZaxis();
					clamp_q += p.center;

					col_point[nPoints] = clamp_q;
					col_normal[nPoints] = -gess_normal_p;//col normal points towards p		
					nPoints++;
				}
			}
			return true;
		}
		else
		if( numVertexEqualDepth ==2)
		{
			//edge-face collision found!
			for( int i=0; i < nq; i++)
			{		
				if( depth_q[i] < (min_depth_q + myColMargin))
				{
					//clamp external vertices of q over p
					vector3D v = gess_point_q[i];
					real dotX = Dot( v - p.center, p.getXaxis());
					real dotY = Dot( v - p.center, p.getYaxis());
					real dotZ = Dot( v - p.center, p.getZaxis());

					real clampx = VALUE_MAX(- p.size.getX(), VALUE_MIN(dotX, p.size.getX()));
					real clampy = VALUE_MAX(- p.size.getY(), VALUE_MIN(dotY, p.size.getY()));
					real clampz = VALUE_MAX(- p.size.getZ(), VALUE_MIN(dotZ, p.size.getZ()));

					vector3D clamp_q = clampx*p.getXaxis() + clampy*p.getYaxis() + clampz*p.getZaxis();
					clamp_q += p.center;

					col_point[nPoints] = clamp_q;
					col_normal[nPoints] = -gess_normal_p;//col normal points towards p		
					nPoints++;
				}
			}
			return true;
		}
		else
		if(SphereTestBox(qvertexes[min_vertex_q], Epsilon, p))
		{
			//vertex-face collision found!
			col_point[nPoints] = qvertexes[min_vertex_q];
			col_normal[nPoints] = -gess_normal_p;//col normal points towards p		
			nPoints++;
			return true;

		}
		else
		{
			//must be edge-edge collision if boxes are really colliding
		}
	}
	else
	{
		// choose vertexes of p for futher testing
		int numVertexEqualDepth=0;
		for( int i=0; i < np; i++)
		{		
			if( depth_p[i] < (min_depth_p +  myColMargin))
			{
				numVertexEqualDepth++;
			}
		}
		if( numVertexEqualDepth >=3)
		{
			//face-face collision found!
 			for( int i=0; i < np; i++)
			{		
				if( depth_p[i] < (min_depth_p + myColMargin))
				{
					//clamp external vertices of p over q
					vector3D v = gess_point_p[i];
					real dotX = Dot( v - q.center, q.getXaxis());
					real dotY = Dot( v - q.center, q.getYaxis());
					real dotZ = Dot( v - q.center, q.getZaxis());

					real clampx = VALUE_MAX(- q.size.getX(), VALUE_MIN(dotX, q.size.getX()));
					real clampy = VALUE_MAX(- q.size.getY(), VALUE_MIN(dotY, q.size.getY()));
					real clampz = VALUE_MAX(- q.size.getZ(), VALUE_MIN(dotZ, q.size.getZ()));

					vector3D clamp_p = clampx*q.getXaxis() + clampy*q.getYaxis() + clampz*q.getZaxis();
					clamp_p += q.center;

					col_point[nPoints] = clamp_p;
					col_normal[nPoints] = gess_normal_q;//col normal points towards p		
					nPoints++;
				}
			}
			return true;
		}
		else
		if( numVertexEqualDepth ==2)
		{
			//edge-face collision found!
			for( int i=0; i < np; i++)
			{		
				if( depth_p[i] < (min_depth_p +  myColMargin))
				{
					//clamp external vertices of p over q
					vector3D v = gess_point_p[i];
					real dotX = Dot( v - q.center, q.getXaxis());
					real dotY = Dot( v - q.center, q.getYaxis());
					real dotZ = Dot( v - q.center, q.getZaxis());

					real clampx = VALUE_MAX(- q.size.getX(), VALUE_MIN(dotX, q.size.getX()));
					real clampy = VALUE_MAX(- q.size.getY(), VALUE_MIN(dotY, q.size.getY()));
					real clampz = VALUE_MAX(- q.size.getZ(), VALUE_MIN(dotZ, q.size.getZ()));

					vector3D clamp_p = clampx*q.getXaxis() + clampy*q.getYaxis() + clampz*q.getZaxis();
					clamp_p += q.center;

					col_point[nPoints] = clamp_p;
					col_normal[nPoints] = gess_normal_q;//col normal points towards p		
					nPoints++;
				}
			}
			return true;
		}
		else
		if(SphereTestBox(pvertexes[min_vertex_p], Epsilon, q))
		{
			//vertex-face collision found!
			col_point[nPoints] = pvertexes[min_vertex_p];
			col_normal[nPoints] = gess_normal_q;//col normal points towards p		
			nPoints++;
			return true;

		}
		else
		{
			//must be edge-edge collision if boxes are really colliding
		}
	}


	//check edge-edge collision
	//build edges
	poly_edge edges[12];
	edges[0].i0=0;edges[0].i1=1;
	edges[1].i0=1;edges[1].i1=2;
	edges[2].i0=2;edges[2].i1=3;
	edges[3].i0=3;edges[3].i1=0;

	edges[4].i0=4;edges[4].i1=5;
	edges[5].i0=5;edges[5].i1=6;
	edges[6].i0=6;edges[6].i1=7;
	edges[7].i0=7;edges[7].i1=4;

	edges[8].i0=3;edges[8].i1=7;
	edges[9].i0=2;edges[9].i1=4;

	edges[10].i0=0;edges[10].i1=6;
	edges[11].i0=1;edges[11].i1=5;

	real min_dist_edges=REAL_MAX;
	real min_dist_edges2=REAL_MAX;
	real t,s;
//	real t2,s2;
	vector3D edge_point, edge_point2;
	vector3D edge_normal, edge_normal2;
	int e1=-1,e2=-1;
	int e12=-1,e22=-1;
	hits=0;

	//search for the nearest couple of edges
	for(int i=0; i < 12; i++)
	{
//not working , removed
//		if( min_vertex_p>=0 && !( edges[i].i0==min_vertex_p || edges[i].i1==min_vertex_p))continue;

		vector3D p0=pvertexes[edges[i].i0];
		vector3D p1=pvertexes[edges[i].i1];		
		vector3D pm = (p0+p1)*.5f;
		for(int j=0; j < 12; j++)
		{
//			if( min_vertex_q>=0 && !( edges[j].i0==min_vertex_q || edges[j].i1==min_vertex_q))continue;

			vector3D q0=qvertexes[edges[j].i0];
			vector3D q1=qvertexes[edges[j].i1];
			vector3D qm = (q0+q1)*.5f;
			
			vector3D guess_normal = CrossProduct(p1-p0, q1-q0);
			if(guess_normal.squaredLength() <  myColMargin)continue;//edges are collineal

			real dist = sqrDistanceSegs(p0, p1, q0, q1, t, s);
			if( (t < Epsilon || t > (1-Epsilon)) && (s < Epsilon || s > (1-Epsilon)))
			{
				//edges arent crossing
				continue;
			}

			if( dist < min_dist_edges)
			{
				min_dist_edges2=min_dist_edges;
				edge_point2 = edge_point; 
				edge_normal2 = edge_normal;
				e12 = e1;
				e22 = e2;

				min_dist_edges=dist;
				edge_point = p0 + t*(p1-p0); 
				edge_normal = CrossProduct(p1-p0, q1-q0);
				edge_normal.normalize();
				e1 = i;
				e2 = j;
				hits++;
			}
			else
			if( dist < min_dist_edges2)
			{
				//we need to store 2 nearest pair of edges to make sure
				min_dist_edges2=dist;
				edge_point2 = p0 + t*(p1-p0); 
				edge_normal2 = CrossProduct(p1-p0, q1-q0);
				edge_normal2.normalize();
				e12 = i;
				e22 = j;
				hits++;
			}
		}
	}

	if(e1 < 0 )
	{
		//error
		assert(0);
		return false;
	}			

	if( Dot(p.center-q.center,edge_normal) < 0.0f)
	{
		edge_normal=-edge_normal;
	}

	real dotX = fabs(Dot( edge_point  - q.center, q.getXaxis()));
	real dotY = fabs(Dot( edge_point  - q.center, q.getYaxis()));
	real dotZ = fabs(Dot( edge_point  - q.center, q.getZaxis()));
	{
		//inside
		col_point[nPoints] = edge_point;
		col_normal[nPoints] = edge_normal;	
		nPoints++;
   		//draw collion info
		//edge_p0 = pvertexes[edges[e1].i0];
		//edge_p1 = pvertexes[edges[e1].i1];
		//edge_q0 = qvertexes[edges[e2].i0];
		//edge_q1 = qvertexes[edges[e2].i1];			
	}

	if( hits >=2)
	{
		if( Dot(p.center-q.center,edge_normal2) < 0.0f)
		{
			edge_normal2=-edge_normal2;
		}

		// if second nearest point is inside the boxes, we have missed an edge-face collision for floating point ossues, so add the point
		real dotX = fabs(Dot( edge_point2  - q.center, q.getXaxis()));
		real dotY = fabs(Dot( edge_point2  - q.center, q.getYaxis()));
		real dotZ = fabs(Dot( edge_point2  - q.center, q.getZaxis()));
		if( dotX < (q.size.getX()+ myColMargin) && 
			dotY < (q.size.getY()+ myColMargin) &&
			dotZ < (q.size.getZ()+ myColMargin))
		{
			//inside
			col_point[nPoints] = edge_point2;
			col_normal[nPoints] = edge_normal2;	
			nPoints++;
		}
	}
				
	assert(nPoints > 0 && nPoints < 16);
	////draw collion info
	//for( int i=0; i < 4; i++)
	//{
	//	last_col_point[i] = col_point[i];
	//	last_col_normal[i] = col_normal[i];
	//}
	return true;
}
bool rayfindSphere (const vector3D& origin, const vector3D& direction,
						const vector3D &center, const real radius,
						int& riQuantity, vector3D akPoint[2])
{
    // set up quadratic Q(t) = a*t^2 + 2*b*t + c
    vector3D kDiff = origin - center;
    real fA = direction.squaredLength();
    real fB = Dot(kDiff,direction);
    real fC = kDiff.squaredLength() - radius*radius;

    real afT[2];
    real fDiscr = fB*fB - fA*fC;
    if ( fDiscr < 0.0f )
    {
        riQuantity = 0;
        return false;
    }
    else if ( fDiscr > 0.0f )
    {
        real fRoot =sqrt(fDiscr);
        real fInvA = 1.0f/fA;
        afT[0] = (-fB - fRoot)*fInvA;
        afT[1] = (-fB + fRoot)*fInvA;

        if ( afT[0] >= 0.0f )
        {
            riQuantity = 2;
            akPoint[0] = origin + afT[0]*direction;
            akPoint[1] = origin + afT[1]*direction;
            return true;
        }
        else if ( afT[1] >= 0.0f )
        {
            riQuantity = 1;
            akPoint[0] = origin + afT[1]*direction;
            return true;
        }
        else
        {
            riQuantity = 0;
            return false;
        }
    }
    else
    {
        afT[0] = -fB/fA;
        if ( afT[0] >= 0.0f )
        {
            riQuantity = 1;
            akPoint[0] = origin + afT[0]*direction;
            return true;
        }
        else
        {
            riQuantity = 0;
		   return false;
        }
    }
}

static bool Clip (real fDenom, real fNumer, real& rfT0, real& rfT1)
{
    // Return value is 'true' if line segment intersects the current test
    // plane.  Otherwise 'false' is returned in which case the line segment
    // is entirely clipped.

    if ( fDenom > 0.0f )
    {
        if ( fNumer > fDenom*rfT1 )
            return false;
        if ( fNumer > fDenom*rfT0 )
            rfT0 = fNumer/fDenom;
        return true;
    }
    else if ( fDenom < 0.0f )
    {
        if ( fNumer > fDenom*rfT0 )
            return false;
        if ( fNumer > fDenom*rfT1 )
            rfT1 = fNumer/fDenom;
        return true;
    }
    else
    {
        return fNumer <= 0.0f;
    }
}

bool rayfindBox (const vector3D& origin, const vector3D& direction,
				 const Bbox& rkBox, int& riQuantity,vector3D akPoint[2])
{
    // convert ray to box coordinates
    vector3D kDiff = origin - rkBox.center;
    vector3D localOrigin(
        Dot(kDiff, rkBox.getXaxis()),
        Dot(kDiff, rkBox.getYaxis()),
        Dot(kDiff, rkBox.getZaxis())
		);

    vector3D localDirection(
        Dot(direction,rkBox.getXaxis()),
        Dot(direction,rkBox.getYaxis()),
        Dot(direction,rkBox.getZaxis())
		);

    real fT0 = 0.0f, fT1 = REAL_MAX;
//    bool bIntersects = FindIntersection(kOrigin,kDirection,rkBox.Extents(),fT0,fT1);
    real fSaveT0 = fT0, fSaveT1 = fT1;
	vector3D extent = rkBox.size;

    bool bNotEntirelyClipped =
        Clip(localDirection(0),-localOrigin(0)-extent(0),fT0,fT1) &&
        Clip(-localDirection(0),localOrigin(0)-extent(0),fT0,fT1) &&
        Clip(localDirection(1),-localOrigin(1)-extent(1),fT0,fT1) &&
        Clip(-localDirection(1),localOrigin(1)-extent(1),fT0,fT1) &&
        Clip(localDirection(2),-localOrigin(2)-extent(2),fT0,fT1) &&
        Clip(-localDirection(2),localOrigin(2)-extent(2),fT0,fT1);

    bool bIntersects =  bNotEntirelyClipped && ( fT0 != fSaveT0 || fT1 != fSaveT1 );


    if ( bIntersects )
    {
        if ( fT0 > 0.0f )
        {
            riQuantity = 2;
            akPoint[0] = origin + fT0*direction;
            akPoint[1] = origin + fT1*direction;
        }
        else  // fT0 == 0
        {
            riQuantity = 1;
            akPoint[0] = origin + fT1*direction;
        }
    }
    else
    {
        riQuantity = 0;
    }

    return bIntersects;
}



