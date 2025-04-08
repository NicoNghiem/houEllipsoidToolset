#ifndef __ellipsoidtoolset_h__
#define __ellipsoidtoolset_h__

#include <ellipsoid.h>
#include <hyperplane.h>

// --------------------
// ----- Methods ------
// --------------------

// Ellipsoidal Calculus

// Ellipsoid / Hyperplane toolset

// Returns the distance between an ellipsoid and an hyperplane. This value can be negative
function float distance(const ellipsoid E ; const hyperplane H)
{
    float offset = H->getOffset();
    vector N = H->getN();
    vector center = E->getCenter();
    metric M = E->getMetric();
    matrix3 Q = M->getInverseSqrtTensor();

    return (abs(offset - dot(center, N)) - dot(N, Q*N)) / length(center);
}


// Ellipsoid / Ellipsoid toolset

// Return the Ellipsoid that approximates the Minkowski Sum of E0 and E1 along direction l
function ellipsoid minkowskiExternalApproximator(const ellipsoid E0, E1; const vector l)
{
    vector dir = normalize(l);
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

// Return the Ellipsoid that approximates the Minkowski Sum of E0 and E1 along direction C0-C1
function ellipsoid minkowskiExternalApproximator(const ellipsoid E0, E1)
{
    vector diff = E1->getCenter() - E0->getCenter();
    return minkowskiExternalApproximator(E0,E1,diff);
}

// Utils

// Intersection Methods

// Returns if E0 and E1 collide and stores the displacement necessary to push E1 to fix the collision
function int collide(const ellipsoid E0, E1 ; vector displacement)
{
    vector l = E1->getCenter() - E0->getCenter();
    ellipsoid Estar = minkowskiExternalApproximator(E0, E1,l);
    matrix3 Qstar = Estar->getInverseSqrtTensor();
    float dist = norm(Estar->getMetric(), l);
    displacement = Qstar * (( 1. - dist) * normalize(l));
    return Estar->isInside(l);
}

// Returns if pt0 and pt1 collide and stores the displacement necessary to push pt1 to fix the collision
function int collide(const int geo, pt0, pt1; vector displacement)
{
    ellipsoid E0 = ellipsoidFromPoint(geo, pt0);
    ellipsoid E1 = ellipsoidFromPoint(geo, pt1);

    return collide(E0,E1,displacement);
}

#endif