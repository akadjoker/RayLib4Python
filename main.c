#include <stdio.h>
#include <stdlib.h>
#define GESTURES_IMPLEMENTATION
#define RLGL_IMPLEMENTATION
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
//#include "rlgl.h"

#include "chipmunk.h"

#define SAFE_DELETE(p)           do { delete (p); (p) = nullptr; } while(0)
//#define SAFE_DELETE_ARRAY(p)     do { if(p) { delete[] (p); (p) = nullptr; } } while(0)
#define SAFE_FREE(p)             do { if(p) { free(p); (p) = NULL; } } while(0)
#define SAFE_RELEASE(p)          do { if(p) { (p)->release(); } } while(0)
#define SAFE_RELEASE_NULL(p)     do { if(p) { (p)->release(); (p) = nullptr; } } while(0)
#define SAFE_RETAIN(p)           do { if(p) { (p)->retain(); } } while(0)



Color newColor(float r, float g, float b, float a)
{
  Color c;
  c.r=r*255.0f;
  c.g=g*255.0f;
  c.b=b*255.0f;
  c.a=a*255.0f;
  return c;

}
static Vector2* cpVertArray2ccpArrayN(const cpVect* cpVertArray, unsigned int count)
{
    if (count == 0) return NULL;
    Vector2* pPoints = malloc( count* sizeof(Vector2)) ;

    for (unsigned int i = 0; i < count; ++i)
    {
        pPoints[i].x = cpVertArray[i].x;
        pPoints[i].y = cpVertArray[i].y;
    }
    return pPoints;
}

static Color ColorForBody(cpBody *body)
{
	if (cpBodyIsRogue(body) || cpBodyIsSleeping(body))
    {
		return newColor(0.5f, 0.5f, 0.5f ,0.5f);
	}
    else if (body->CP_PRIVATE(node).idleTime > body->CP_PRIVATE(space)->sleepTimeThreshold)
    {
		return newColor(0.33f, 0.33f, 0.33f, 0.5f);
	}
    else
    {
		return newColor(1.0f, 0.0f, 0.0f, 0.5f);
	}
}

static inline unsigned char uclerp(unsigned char f1, unsigned char f2, unsigned char t)
{
	return f1*(255 - t) + f2*t;
}

void updateShape(void *ptr, void* unused)
{
cpShape *shape = (cpShape*)ptr;
}
static void DrawShape(cpShape *shape,  void* unused)
{

    cpBody *body = shape->body;
	Color color = ColorForBody(body);

	switch (shape->CP_PRIVATE(klass)->type)
    {
		case CP_CIRCLE_SHAPE:
        {
            cpCircleShape *circle = (cpCircleShape *)shape;
            cpVect center = circle->tc;
            cpFloat radius = circle->r;

            DrawCircle((int)center.x,(int)center.y,cpfmax(radius, 1.0),color);

            int linex0=(int)center.x;
            int liney0=(int)center.y;

            cpVect endline = cpvadd(center, cpvmult(body->rot, radius));

              int linex2=(int)endline.x;
              int liney2=(int)endline.y;

            DrawLine(linex0,liney0,linex2,liney2,color);

        }
             break;
		case CP_SEGMENT_SHAPE:
        {
            cpSegmentShape *seg = (cpSegmentShape *)shape;
          //  renderer->drawSegment(cpVert2Point(seg->ta), cpVert2Point(seg->tb), cpfmax(seg->r, 2.0), color);

            int linex0=(int)seg->ta.x;
            int liney0=(int)seg->ta.y;
            int linex2=(int)seg->tb.x;
            int liney2=(int)seg->tb.y;

         DrawLine(linex0,liney0,linex2,liney2,color);

        }
            break;
		case CP_POLY_SHAPE:
        {
            cpPolyShape *poly = (cpPolyShape *)shape;
            Color line  = Fade(color,0.5f);
           Vector2* pPoints = cpVertArray2ccpArrayN(poly->tVerts, poly->numVerts);
         //   renderer->drawPolygon(pPoints, poly->numVerts, color, 1.0, line);

         DrawTriangleFan(pPoints,poly->numVerts,line);
         DrawLineStrip(pPoints,poly->numVerts,color);
            SAFE_FREE(pPoints);
        }
            break;
		default:
			cpAssertHard(false, "Bad assertion in DrawShape()");
	}

}

static void DrawConstraint(cpConstraint *constraint,  void* unused)
{
    Color CONSTRAINT_COLOR = newColor(0,1,0,0.5f);

    cpBody *body_a = constraint->a;
	cpBody *body_b = constraint->b;

	const cpConstraintClass *klass = constraint->CP_PRIVATE(klass);
	if (klass == cpPinJointGetClass())
    {
		cpPinJoint *joint = (cpPinJoint *)constraint;

		cpVect a = cpBodyLocal2World(body_a, joint->anchr1);
		cpVect b = cpBodyLocal2World(body_b, joint->anchr2);



		int pointAX=(int)a.x;
		int pointAY=(int)a.y;
		int pointBX=(int)b.x;
		int pointBY=(int)b.y;


		DrawCircle(pointAX,pointAY,3.0f,CONSTRAINT_COLOR);
		DrawCircle(pointBX,pointBY,3.0f,CONSTRAINT_COLOR);


		DrawLine(pointAX,pointAY,pointBX,pointBY,CONSTRAINT_COLOR);

       // renderer->drawDot(cpVert2Point(a), 3.0, CONSTRAINT_COLOR);
       // renderer->drawDot(cpVert2Point(b), 3.0, CONSTRAINT_COLOR);
       // renderer->drawSegment(cpVert2Point(a), cpVert2Point(b), 1.0, CONSTRAINT_COLOR);
	}
    else if (klass == cpSlideJointGetClass())
    {
		cpSlideJoint *joint = (cpSlideJoint *)constraint;

		cpVect a = cpBodyLocal2World(body_a, joint->anchr1);
		cpVect b = cpBodyLocal2World(body_b, joint->anchr2);

     //   renderer->drawDot(cpVert2Point(a), 3.0, CONSTRAINT_COLOR);
      //  renderer->drawDot(cpVert2Point(b), 3.0, CONSTRAINT_COLOR);
      //  renderer->drawSegment(cpVert2Point(a), cpVert2Point(b), 1.0, CONSTRAINT_COLOR);


		int pointAX=(int)a.x;
		int pointAY=(int)a.y;
		int pointBX=(int)b.x;
		int pointBY=(int)b.y;


		DrawCircle(pointAX,pointAY,3.0f,CONSTRAINT_COLOR);
		DrawCircle(pointBX,pointBY,3.0f,CONSTRAINT_COLOR);

		DrawLine(pointAX,pointAY,pointBX,pointBY,CONSTRAINT_COLOR);
	}
    else if (klass == cpPivotJointGetClass())
    {
		cpPivotJoint *joint = (cpPivotJoint *)constraint;

		cpVect a = cpBodyLocal2World(body_a, joint->anchr1);
		cpVect b = cpBodyLocal2World(body_b, joint->anchr2);

		int pointAX=(int)a.x;
		int pointAY=(int)a.y;
		int pointBX=(int)b.x;
		int pointBY=(int)b.y;


		DrawCircle(pointAX,pointAY,3.0f,CONSTRAINT_COLOR);
		DrawCircle(pointBX,pointBY,3.0f,CONSTRAINT_COLOR);

		DrawLine(pointAX,pointAY,pointBX,pointBY,CONSTRAINT_COLOR);

      //  renderer->drawDot(cpVert2Point(a), 3.0, CONSTRAINT_COLOR);
      //  renderer->drawDot(cpVert2Point(b), 3.0, CONSTRAINT_COLOR);
	}
    else if (klass == cpGrooveJointGetClass())
    {
		cpGrooveJoint *joint = (cpGrooveJoint *)constraint;

		cpVect a = cpBodyLocal2World(body_a, joint->grv_a);
		cpVect b = cpBodyLocal2World(body_a, joint->grv_b);
		cpVect c = cpBodyLocal2World(body_b, joint->anchr2);


		int pointAX=(int)a.x;
		int pointAY=(int)a.y;
		int pointBX=(int)b.x;
		int pointBY=(int)b.y;
        int pointCX=(int)c.x;
		int pointCY=(int)c.y;



		DrawCircle(pointCX,pointCY,3.0f,CONSTRAINT_COLOR);

		DrawLine(pointAX,pointAY,pointBX,pointBY,CONSTRAINT_COLOR);

     //   renderer->drawDot(cpVert2Point(c), 3.0, CONSTRAINT_COLOR);
     //   renderer->drawSegment(cpVert2Point(a), cpVert2Point(b), 1.0, CONSTRAINT_COLOR);
	}
    else if (klass == cpDampedSpringGetClass())
    {
		// TODO
	}
    else
    {
        		printf("Cannot draw constraint\n");
	}
}
