#ifndef __metric_h__
#define __metric_h__

struct metric
{
    matrix3 tensor;
    matrix3 invTensor;
    matrix3 sqrtTensor;
    matrix3 invSqrtTensor;
    int init=0;

    void init()
    {
        // Invert the tensor and stores it in memory to avoid recomputing an inverse all the time
        if(this.init==0)
        {
            this.invTensor = invert(this.tensor);
            // compute sqrt of tensor. Assumes it is diagonal symmetric
            vector diag;
            matrix3 Q = diagonalizesymmetric(this.tensor, diag);
            matrix3 D = ident();
            scale(D, sqrt(diag));
            this.sqrtTensor = transpose(Q) * D * Q;
            this.invSqrtTensor = invert(this.sqrtTensor);
            this.init = 1;
        }
        return;
    }

    // Set Methods
    void setTensor(matrix3 T)
    {
        tensor = T;
        this.init = 0;
        return;
    }

    void rotateTensor(matrix3 R)
    {
        tensor *= R;
        init=0;
    }

    void preRotateTensor(matrix3 R)
    {
        tensor = R * tensor;
        init =0;
    }

    // Get Methods
    matrix3 getTensor()
    {
        return this.tensor;
    }

    matrix3 getInverseTensor()
    {        
        this->init();
        return this.invTensor;
    }
    
    matrix3 getSqrtTensor()
    {
        this->init();
        return this.sqrtTensor;
    }

    matrix3 getInverseSqrtTensor()
    {
        this->init();
        return this.invSqrtTensor;
    }
}

// --------------------
// --- CONSTRUCTORS ---
// --------------------

function metric metricFromMatrix(matrix3 mat)
{
    metric Q;
    Q->setTensor(mat);
    return Q;
}

function metric metricFromOrientScale(vector4 orient; vector scale)
{
    matrix3 S = ident();
    scale(S, 1. / (scale*scale));
    matrix3 O = qconvert(orient);
    
    metric Q;
    Q->setTensor(transpose(O) * S * O);

    return Q;
}

function metric metricFromPoint(int geo; int pt ; string orientAttrib ; string scaleAttrib)
{
    vector4 orient = point(geo,orientAttrib,pt);
    vector scale = point(geo,scaleAttrib,pt);

    return metricFromOrientScale(orient,scale);
}

function metric metricFromPoint(int geo; int pt)
{
    return metricFromPoint(geo,pt,'orient','scale');
}

// --------------------
// -----  METHOD ------
// --------------------

// Overloading norm and dist with a given metric
function float norm2(metric Q ; vector x)
{
    return dot(x , Q->getTensor() * x);
}

function float norm(metric Q ; vector x)
{
    return sqrt(norm2(Q,x));
}

function float dist2(metric Q ; vector x,y)
{
    return norm2(Q, x-y);
}

function float dist(metric Q ; vector x,y)
{
    return norm(Q, x-y);
}


#endif