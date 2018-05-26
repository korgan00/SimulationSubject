/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#if !defined(_GEO3D_H_)

#include <math.h>       
#include <assert.h>

#if defined(Length)
#undef Length
#endif

#pragma warning (disable:4244)      // int to float conversion
#pragma warning (disable:4514)      // unreferenced inline removed
#pragma warning (default:4244)      // int to float conversion


#include <float.h>      
#include <assert.h>

typedef float real;

real const REAL_MAX = FLT_MAX;
real const REAL_MIN = FLT_MIN;

real const PI = real(3.14159265358979323846);
real const Epsilon = real(0.00001);
real const ColMargin = Epsilon*1000.0f;

#define VALUE_MIN(a,b) (((a) > (b))?(b):(a))
#define VALUE_MAX(a,b) (((a) > (b))?(a):(b))

real GenerateUnitRandomReal( void );

class vector3D;
class matrix3D;

class vector3D 
{
public:

    inline vector3D( void );
    inline vector3D( real X, real Y, real Z = 0);

    inline vector3D &operator+=(vector3D const &addend);
    inline vector3D &operator-=(vector3D const &lessor);
    inline vector3D &operator*=(real const multiplier);
    inline vector3D &operator/=(real const divisor);
    inline real &operator()( int unsigned Index )
	{   
		assert(Index < 3);
		return element[Index];
	}

    inline real const &operator()( int unsigned Index ) const
	{   
		assert(Index < 3);
		return element[Index];
	}
    inline void set( int Index, real Value )
	{
		assert(Index< 3);
	    element[Index] = Value;
	}
    real const *getArray( void ) const { return element; };

	inline real getX( void ) const{ return element[0];}
    inline real getY( void ) const{ return element[1];}
    inline real getZ( void ) const{ return element[2];}

	inline void setX( real value ){ element[0]=value;}
    inline void setY( real value ){ element[1]=value;}
    inline void setZ( real value ){ element[2]=value;}

    vector3D &normalize( void );
    real length( void ) const;
    real squaredLength( void ) const;

private:
    real element[3];
};


class matrix3D
{
public:

    matrix3D( void );
    matrix3D( real (*pElements)[3] );

    enum mode { SkewSymmetric, Cero };
    inline matrix3D( mode _Cero );
    inline matrix3D( vector3D const &CrossVector, mode _SkewSymmetric );
    
    inline matrix3D &operator+=( matrix3D const &M );

	void setIdentity();
    inline real &operator()( int unsigned Row, int unsigned Column );
    inline real const &operator()( int unsigned Row, int unsigned Column ) const;

    inline real GetElement( int Row, int Column ) const;
    inline matrix3D &SetElement( int Row, int Column, real Value );

protected:

    real element[3][3];
};

inline vector3D operator+( vector3D const &Operand1,
        vector3D const &Operand2 );

inline vector3D operator-( vector3D const &Operand1,
        vector3D const &Operand2 );

inline vector3D operator*( vector3D const &Operand,
        real const &Multiplier );

inline vector3D operator*( real const &Multiplier,
        vector3D const &Multiplicand );
      
inline vector3D operator/( vector3D const &Quotient,
        real const &Divisor );

inline vector3D operator-( vector3D const &Operand1 );

inline vector3D CrossProduct( vector3D const &Operand1,
        vector3D const &Operand2 );

inline vector3D normalize( vector3D const &v );

void OrthonormalizeOrientation( matrix3D &Orientation );

inline real Dot( vector3D const &Operand1, vector3D const &Operand2 );

inline matrix3D Transpose( matrix3D const &Matrix );

matrix3D operator*( matrix3D const &Multiplicand,
        matrix3D const &Multiplier );

vector3D operator*( matrix3D const &Operand,
        vector3D const &Multiplier );

inline matrix3D operator-( matrix3D const &Matrix );
inline matrix3D operator+( matrix3D const &Operand1,
        matrix3D const &Operand2 );
inline matrix3D operator-( matrix3D const &Operand1,
        matrix3D const &Operand2 );
inline matrix3D operator*( real const Value, matrix3D const &Matrix );
inline matrix3D operator*( matrix3D const &Matrix, real const Value );

struct poly_edge
{
	int  i0,i1;
	int  f0,f1;
};
struct Bbox  
{
    vector3D center;
    matrix3D orientation;
    vector3D size;
    real radio;

    vector3D getXaxis()const{ return vector3D( orientation(0,0),orientation(1,0),orientation(2,0));}
    vector3D getYaxis()const{ return vector3D( orientation(0,1),orientation(1,1),orientation(2,1));}
    vector3D getZaxis()const{ return vector3D( orientation(0,2),orientation(1,2),orientation(2,2));}

	vector3D axis( int index)const{
		if(index==0)return getXaxis();
		if(index==1)return getYaxis();		
		if(index==2)return getZaxis();		
		if(index==3)return (-getXaxis());		
		if(index==4)return (-getYaxis());		
		return (-getZaxis());
	}

    Bbox(){};
	Bbox(matrix3D O,vector3D p,vector3D s)
	{
		for(int i=0; i < 3; i++)
			for(int j=0; j < 3;j++)
				orientation(i,j)=O(i,j);

		center=p;
		size=s;
	}

};
void box2vertex(const Bbox& _box, vector3D* vertexList);
bool SphereTestBox(const vector3D& center, const real radius, const Bbox& q);
bool SphereFindBox(const vector3D& center, const real radius, const Bbox& q, vector3D&col_point, vector3D& col_normal);
bool boxTestBox(const Bbox& p, const Bbox& q);
bool boxFindBox(const Bbox& p, const Bbox& q, vector3D* col_point, vector3D* col_normal, int& nPoints);
bool rayfindSphere (const vector3D& origin, const vector3D& direction,
						const vector3D &center, const real radius,
						int& riQuantity, vector3D akPoint[2]);
bool rayfindBox (const vector3D& origin, const vector3D& direction,
				 const Bbox& rkBox, int& riQuantity,vector3D akPoint[2]);


/*----------------------------------------------------------------------------

inline function definitions.

*/

inline vector3D &vector3D::operator+=(vector3D const &Operand)
{
    setX(getX() + Operand.getX());
    setY(getY() + Operand.getY());
    setZ(getZ() + Operand.getZ());

    return(*this);
}

inline vector3D operator+( vector3D const &Operand1,
        vector3D const &Operand2 )
{
    return vector3D(Operand1.getX() + Operand2.getX(),
                    Operand1.getY() + Operand2.getY(),
                    Operand1.getZ() + Operand2.getZ());
}

inline vector3D &vector3D::operator-=(vector3D const &Operand)
{
    setX(getX() - Operand.getX());
    setY(getY() - Operand.getY());
    setZ(getZ() - Operand.getZ());

    return(*this);
}

inline vector3D operator-( vector3D const &Operand1,
        vector3D const &Operand2 )
{
    return vector3D(Operand1.getX() - Operand2.getX(),
                    Operand1.getY() - Operand2.getY(),
                    Operand1.getZ() - Operand2.getZ());
}

inline vector3D operator-( vector3D const &Operand1 )
{
    return vector3D(-Operand1.getX(),-Operand1.getY(),-Operand1.getZ());
}

inline vector3D &vector3D::operator*=(real const multiplier)
{
    setX(getX() * multiplier);
    setY(getY() * multiplier);
    setZ(getZ() * multiplier);

    return(*this);
}

inline vector3D operator*( vector3D const &Operand,
        real const &Multiplier )
{
    return vector3D(Operand.getX() * Multiplier,
                    Operand.getY() * Multiplier,
                    Operand.getZ() * Multiplier);
}

inline vector3D operator*( real const &Multiplier,
        vector3D const &Operand ) 
{
    return Operand* Multiplier;
}
      
inline vector3D &vector3D::operator/=(real const divisor)
{
    setX(getX() / divisor);
    setY(getY() / divisor);
    setZ(getZ() / divisor);

    return(*this);
}

inline vector3D operator/( vector3D const &Quotient,
        real const &Divisor )
{
    return Quotient * (real(1.0)/Divisor);
}

inline vector3D CrossProduct( vector3D const &Operand1,
        vector3D const &Operand2 )
{
    real X = Operand1.getY() * Operand2.getZ() -
                Operand1.getZ() * Operand2.getY();
    real Y = Operand1.getZ() * Operand2.getX() -
                Operand1.getX() * Operand2.getZ();
    real Z = Operand1.getX() * Operand2.getY() -
                Operand1.getY() * Operand2.getX();

    return vector3D(X,Y,Z);
}

inline real Dot( vector3D const &Operand1, vector3D const &Operand2 )
{
    return Operand1.getX() * Operand2.getX() +
            Operand1.getY() * Operand2.getY() +
            Operand1.getZ() * Operand2.getZ();
}

inline vector3D::vector3D( void )
{
    element[0] = 0.0f;
    element[1] = 0.0f;
    element[2] = 0.0f;
}

inline vector3D::vector3D( real X, real Y, real Z ) 
{
	element[0]=X;
	element[1]=Y;
	element[2]=Z;
}


inline real &matrix3D::operator()( int unsigned Row, int unsigned Column )
{
    assert((Row < 3) && (Column < 3));
    return element[Row][Column];
}

inline real const &matrix3D::operator()( int unsigned Row,
        int unsigned Column ) const
{
    assert((Row < 3) && (Column < 3));
    return element[Row][Column];
}

inline real matrix3D::GetElement( int Row, int Column ) const
{
    return element[Row][Column];
}

inline matrix3D &matrix3D::SetElement( int Row, int Column, real Value )
{
    element[Row][Column] = Value;

    return *this;
}

inline matrix3D::matrix3D( real (*pElements)[3] )
{
    assert(pElements);

    matrix3D &M = *this;
    for(int i = 0;i < 3;i++)
    {
        for(int j = 0;j < 3;j++)
        {
            M(i,j) = pElements[i][j];
        }
    }
}
    
inline matrix3D::matrix3D( vector3D const &Vector, mode Mode )
{
    assert(Mode == SkewSymmetric);

    matrix3D &M = *this;

    M(0,0) =  real(0);          M(0,1) = -Vector.getZ(); M(0,2) =  Vector.getY();
    M(1,0) =  Vector.getZ(); M(1,1) =  real(0);          M(1,2) = -Vector.getX();
    M(2,0) = -Vector.getY(); M(2,1) =  Vector.getX(); M(2,2) = real(0);
}

inline matrix3D::matrix3D( mode Mode )
{
    assert(Mode == Cero);

    for(int counter = 0;counter < 3;counter++)
    {
        element[counter][0] = real(0);
        element[counter][1] = real(0);
        element[counter][2] = real(0);
    }
}

inline matrix3D Transpose( matrix3D const &Matrix )
{
    matrix3D result;

    for(int counter = 0;counter < 3;counter++)
    {
        result(0,counter) = Matrix(counter,0);
        result(1,counter) = Matrix(counter,1);
        result(2,counter) = Matrix(counter,2);
    }

    return result;
}   

inline matrix3D operator+( matrix3D const &Operand1,
        matrix3D const &Operand2 )
{
    matrix3D result;

    for(int counter = 0;counter < 3;counter++)
    {
        result(0,counter) = Operand1(0,counter) + Operand2(0,counter);
        result(1,counter) = Operand1(1,counter) + Operand2(1,counter);
        result(2,counter) = Operand1(2,counter) + Operand2(2,counter);
    }

    return result;
}   
    
inline matrix3D operator-( matrix3D const &Operand1,
        matrix3D const &Operand2 )
{
    matrix3D result;

    for(int counter = 0;counter < 3;counter++)
    {
        result(0,counter) = Operand1(0,counter) - Operand2(0,counter);
        result(1,counter) = Operand1(1,counter) - Operand2(1,counter);
        result(2,counter) = Operand1(2,counter) - Operand2(2,counter);
    }

    return result;
}   
    
inline matrix3D operator*( real const Value, matrix3D const &Matrix )
{
    matrix3D result;

    for(int counter = 0;counter < 3;counter++)
    {
        result(0,counter) = Value * Matrix(0,counter);
        result(1,counter) = Value * Matrix(1,counter);
        result(2,counter) = Value * Matrix(2,counter);
    }

    return result;
}   

inline matrix3D &matrix3D::operator+=( matrix3D const &Operand1 )
{
    for(int counter = 0;counter < 3;counter++)
    {
        element[0][counter] += Operand1(0,counter);
        element[1][counter] += Operand1(1,counter);
        element[2][counter] += Operand1(2,counter);
    }

    return *this;
}   

inline matrix3D operator*( matrix3D const &Matrix, real const Value )
{
    return Value * Matrix;
}

inline matrix3D operator-( matrix3D const &Matrix )
{
    matrix3D result;

    for(int counter = 0;counter < 3;counter++)
    {
        result(0,counter) = -Matrix(0,counter);
        result(1,counter) = -Matrix(1,counter);
        result(2,counter) = -Matrix(2,counter);
    }

    return result;
}   

inline vector3D normalize( vector3D const &A )
{
    vector3D Return = A;
    return Return.normalize();
}

inline void BuildOpenGLTransformMatrix( matrix3D const &Orientation,
                vector3D const &Position, float *glMat)
{
    assert(glMat);
    float (*glMat44)[4] = (float(*)[4])glMat;
    int i;
    
    for(i = 0;i < 3;i++)
    {
        for(int j = 0;j < 3;j++)
        {
            glMat44[j][i] = Orientation(i,j);
        }
    }

    glMat44[0][3] = glMat44[1][3] = glMat44[2][3] = 0.0f; glMat44[3][3] = 1.0f;

    for(i = 0;i < 3;i++)
    {
        glMat44[3][i] = Position(i);
    }
}

#define _GEO3D_H_
#endif
