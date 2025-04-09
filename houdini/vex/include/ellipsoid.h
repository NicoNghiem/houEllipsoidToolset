#ifndef __ellipsoid_h__
#define __ellipsoid_h__

#include <metric.h>

/*
Struct to describe an ellipsoid
*/

struct ellipsoid
{
    metric Q;
    vector center;

    // Get methods

    vector getCenter()
    {
        return this.center;
    }

    metric getMetric()
    {
        return this.Q;
    }

    matrix3 getTensor()
    {
        return this.Q->getTensor();
    }

    matrix3 getInverseTensor()
    {
        return this.Q->getInverseTensor();
    }

    matrix3 getSqrtTensor()
    {
        return this.Q->getSqrtTensor();
    }

    matrix3 getInverseSqrtTensor()
    {
        return this.Q->getInverseSqrtTensor();
    }

    // set methods
    void setCenter(const vector P)
    {
        this.center = P;
    }

    void setMetric(const metric Qnew)
    {
        this.Q = Qnew;
    }

    // Operations

    void translate(vector t)
    {
        translate(this.center, t);
    }

    void rotate(matrix3 R)
    {
        this.Q->rotateTensor(R);
    }

    void prerotate(matrix3 R)
    {
        this.Q->preRotateTensor(R);
    }

    // Ellipsoid evaluation

    // Evaluate the outer shell of the Ellipsoid in the direction dir
    vector evaluateP(const vector dir)
    {
        return this->getCenter() + (1. / norm(this->getMetric(), dir)) * dir;
    }

    // Evaluates the Normal of the Ellipsoid along the direction dir
    vector evaluateN(const vector dir)
    {
        matrix R = Q->getSqrtTensor();
        return normalize(transpose(R) * R * dir);
    }

    // Test if a point is inside an ellipsoid
    int isInside(const vector P)
    {
        metric M = this->getMetric();
        matrix3 T = M->getTensor();
        vector cP = this->getCenter();
        cP -= P;
        return dot(cP, T * cP)<=1;
    }
}

/*
Constructors
*/

function ellipsoid ellipsoidFromPoint(const int geo; const int pt; const string orientAttrib ; const string scaleAttrib)
{
    ellipsoid E;
    metric Q = metricFromPoint(geo,pt,orientAttrib,scaleAttrib);

    E.center = vector(point(geo,'P',pt));
    E.Q = Q;
    return E;
}

function ellipsoid ellipsoidFromPoint(const int geo,pt)
{
    return ellipsoidFromPoint(geo,pt,'orient','scale');
}

function ellipsoid ellipsoidFromMetricCenter(const metric Q;const vector center)
{
    ellipsoid E;
    E.center = center;
    E.Q = Q;
    return E;
}

function ellipsoid ellipsoidFromMatrixCenter(const matrix3 Q; const vector center)
{
    metric M = metricFromMatrix(Q);
    return ellipsoidFromMetricCenter(M,center);
}

// Set methods

// Set the point to become a container for the ellipsoid
function void setEllipsoid(int geo,pt; ellipsoid E)
{
    vector P = E->getCenter();
    matrix3 T = E->getInverseTensor();
    vector scale;
    matrix3 R = diagonalizesymmetric(T, scale);
    vector4 orient = quaternion(R);
    setpointattrib(geo, 'P', pt, P);
    setpointattrib(geo, 'orient', pt, orient);
    setpointattrib(geo,'scale',pt,sqrt(scale));
}

// Add an ellipsoid to the geometry
function int addEllipsoid(int geo; ellipsoid E)
{
    int pt = addpoint(geo, 0);
    setEllipsoid(geo,pt,E);
    return pt;
}

function int addPrimEllipsoid(int geo; ellipsoid E)
{
    int pt = addEllipsoid(geo, E);
    int prim = addprim(0,'sphere',pt);
    matrix3 transform = E->getInverseSqrtTensor();
    setprimintrinsic(geo,'transform',prim,transform);
    return prim;
}

#endif