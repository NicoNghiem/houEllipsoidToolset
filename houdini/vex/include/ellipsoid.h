#ifndef __ellipsoid_h__
#define __ellipsoid_h__

#include <metric.h>

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
        matrix3 R = Q->getInverseSqrtTensor();
        return this->getCenter() + R * dir;
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

// --------------------
// ----- Methods ------
// --------------------

// Ellipsoidal Calculus

// Return the Ellipsoid that approximates the Minkowski Sum of E0 and E1
function ellipsoid minkowskiExternalApproximator(const ellipsoid E0, E1)
{
    vector diff = E1->getCenter() - E0->getCenter();
    vector dir = normalize(diff);

    metric M0 = E0->getMetric();
    metric M1 = E1->getMetric();
    
    matrix3 Q0 = M0->getInverseTensor();
    matrix3 Q1 = M1->getInverseTensor();
    
    float l0 = sqrt(dot(dir, Q0*dir));
    float l1 = sqrt(dot(dir, Q1*dir));
    matrix3 Qstar = invert((l0+l1) * (1./l0 * Q0 + 1./l1 * Q1));
    metric Mstar = metricFromMatrix(Qstar);
    vector origin = {0.,0.,0.};
    return ellipsoidFromMetricCenter(Mstar, origin);
}

// Utils

// Intersection Methods

// Returns if E0 and E1 collide 
function int collide(const ellipsoid E0, E1 ; vector displacement)
{

}

#endif