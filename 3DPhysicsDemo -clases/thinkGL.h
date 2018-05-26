#if !defined(THINKGL_H)
#define THINKGL_H

#include <vector>

struct thkTransform
{
	float aM[16];
	float radius;
	float color[3];
};
struct thkline
{
	float start[3];
	float end[3];
	float color[3];
};

struct thkPoint
{
	float pos[3];
	float color[3];
};

class thinkGL
{
private:
	std::vector< thkTransform > boxTransform;	
	std::vector< thkTransform > sphereTransform;	
	std::vector< thkline > drawLines;	
	std::vector< thkPoint > drawPoints;	

public:
	float glWorldSize;
	bool bDrawCollisionInfo;

	bool Init( double width, double height);
	void StoreBox( int index, float* color, float sx,float sy,float sz);
    void Render( void );
	void clearView( float zoom,  float height, float angle);
	void printGLtext(int x, int y, char* String, int size);
	void getRayThrough(int x,int y, 
						   double& nearx, double& neary, double& nearz,
						   double& farx, double& fary, double& farz);

	void addBoxTransform(thkTransform& trs){ boxTransform.push_back(trs);}
	void addSphereTransform(thkTransform& trs){ sphereTransform.push_back(trs);}
	void clearBoxTransformList(){ boxTransform.clear();}
	void clearSphereTransformList(){ sphereTransform.clear();}
	void clearLinesList(){ drawLines.clear();}
	void clearPointsList(){ drawPoints.clear();}
	void clearDrawInfo();
	void setWordlSize(float size){ glWorldSize=size;}
	void addLine(const float *v1, const float *v2, float* color){
		thkline  line;
		line.start[0] = v1[0];line.start[1] = v1[1];line.start[2] = v1[2];
		line.end[0] = v2[0];line.end[1] = v2[1];line.end[2] = v2[2];
		line.color[0]=color[0];line.color[1]=color[1];line.color[2]=color[2];
		drawLines.push_back(line);
	}
	void addPoint(const float *v1, float* color){
		thkPoint  point;
		point.pos[0] = v1[0];point.pos[1] = v1[1];point.pos[2] = v1[2];
		point.color[0]=color[0];point.color[1]=color[1];point.color[2]=color[2];
		drawPoints.push_back(point);
	}

	thinkGL(){bDrawCollisionInfo=false;glWorldSize=0.0; boxTransform.clear();sphereTransform.clear();}
	~thinkGL(){ boxTransform.clear();sphereTransform.clear();}

	static void glutMouseFuncCallback(int button, int state, int x, int y);

};


#endif

