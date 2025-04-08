#ifndef __hyperplane_h__
#define __hyperplane_h__

/*
Struct for describing an Hyperplane
*/

struct hyperplane
{
    vector N;
    float offset;

    // Get methods

    vector getN()
    {
        return this.N;
    }

    float getOffset()
    {
        return this.offset;
    }

    // Set methods

    void setN(vector newN)
    {
        this.N = normalize(newN);
    }

    void setOffset(float newOffset)
    {
        this.offset = newOffset;
    }

    // Translate the Hyperplane
    void translate(vector t)
    {
        this.offset += dot(t, this->getN());
    }

    // Move the Hyperplane to the given position
    void moveTo(vector pos)
    {
        this.offset = dot(pos, this->getN());
    }

    // Project methods
    
    // Project P onto the hyperplane
    vector project(vector P)
    {
        vector thisN = this->getN();
        return P - dot(thisN,P) * thisN;
    }


}

/*
Constructors
*/

// Builds an Hyperplane from a normal vector and a float offset
function hyperplane hyperplaneFromOffset(vector newN; float newOffset)
{
    hyperplane H;
    H->setN(newN);
    H->setOffset(newOffset);
    return H;
}

// Builds an Hyperplane from a normal vector and a position
function hyperplane hyperplaneFromPosition(vector newN; vector P)
{
    hyperplane H;
    H->setN(newN);
    H->moveTo(P);
    return H;
}

// Builds an Hyperplane from a Point. Assumes we are using the N and P attributes
function hyperplane hyperplaneFromPoint(const int geo, pt)
{
    vector N = point(geo,'N',pt);
    vector center = point(geo,'P',pt);
    return hyperplaneFromPosition(N,center);
}

/*
Set methods
*/

// Set an hyperplane
function void setHyperplane(const int geo, pt; const hyperplane H)
{
    vector N = H->getN();
    vector center = H->getCenter();
    setpointattrib(geo,'P',pt,center);
    setpointattrib(geo,'N',pt,N);
}

// Creates a primitive to represent the hyperplane. Mostly used for debug purposes
function int addDebugHyperplane(const int geo; hyperplane H; const vector P; const float radius)
{
    // creates a circle that belongs to the hyperplane
    // the center of the circle is the closest point on the hyperplane to P
    vector center = H->project(P);
    int pt = addpoint(geo,center);
    int prim = addprim(geo,'circle',pt);
    vector N = H->getN();
    matrix3 transform = dihedral({0,0,-1} , N); // Houdini's default circle is along XY
    scale(transform, radius);
    setprimintrinsic(geo,'transform',prim,transform);
    return prim;
}

#endif