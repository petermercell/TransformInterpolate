// Transform interpolation plugin
// Takes two inputs and dissolves between their transform matrices
// Partially based on cornerpin2d example plugin, although not much is left from it

#include "DDImage/Convolve.h"
#include "DDImage/DDWindows.h"
#include "DDImage/Transform.h"
#include "DDImage/Format.h"
#include "DDImage/ViewerContext.h"
#include "DDImage/gl.h"
#include "DDImage/Knobs.h"

// General includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>

using namespace DD::Image;

static const char* const CLASS = "TransformInterpolate";
static const char* const HELP = "Version 1.2. Node takes two inputs and dissolves between their transform matrices with logic similar to merge node. B input is the default transform and A is additional.\nUse the factor knob to change the transform mix.\n\nUse Corneroin Based Interpolation knob switches from decomposition based interpolation to cornerpin points based interpolation. It can handle some cases better but doesn't do rotations well.\n\nUnder extra settings, CP compensation can be used to add cornerpin based compensation on top of decomposition method to get the best of both worlds. Try it in extreme cases and see if it gives better results.";

#define MAX_INPUTS 2

struct xyStruct
{
  double x, y;
  bool enable;
};


// Generic helper for linearly interpolating two vectors
Vector3 lerpVector3(Vector3 A, Vector3 B, float factor)
{
    Vector3 result;
    float invFactor = 1.0 - factor;
    result.x = A.x * factor + B.x * invFactor;
    result.y = A.y * factor + B.y * invFactor;
    result.z = A.z * factor + B.z * invFactor;
    return result;
}


class TransformInterpolate : public Transform
{

    float _dissolve;
    float _dissolveToA;
    float _dissolveToB;
    float _compensationFactor;
    Matrix4 mA, mB;
    bool _cornerpin_components;
    bool _use_shear;

    int _bakeStart;
    int _bakeEnd;

    float widthA, widthB, heightA, heightB;

    // Full interpolated forward matrix (mA+mB dissolved) — what maps original
    // source corners to their final on-screen positions.  Cached by setMatrix()
    // and exposed row-major via MultiFloat_knob so Python can read it directly.
    Matrix4 _interpolatedMatrix;

    Matrix4 createCornerPinMatrix(xyStruct from[4], xyStruct to[4]);
    Matrix4 dissolveMatricesWithCompensation(Matrix4 mA, Matrix4 mB, float factor);
    Matrix4 dissolveMatricesByComponent(Matrix4 mA, Matrix4 mB, float factor);
    Matrix4 dissolveCornerpinPointsToMatrix(xyStruct a[4], xyStruct b[4], float factor);
    void setCornerPinMatrix(xyStruct c[4], Matrix4& q);
    Matrix4 dissolveMatricesByCornerpin(Matrix4 mA, Matrix4 mB, float factor);
    void setMatrix(Matrix4& matrix);

    public:

    TransformInterpolate(Node* node) : Transform(node)
    {
        _dissolve = 0.5;
        widthA = widthB = heightA = heightB = 0.0;
        _compensationFactor = 0.0f;
        _bakeStart = 1001;
        _bakeEnd = 1100;

        // Initialise interpolated matrix to identity
        _interpolatedMatrix.makeIdentity();

        const Format& format = input_format();
    }

    void matrixAt(const OutputContext& context, Matrix4& matrix);
    int maximum_inputs() const { return MAX_INPUTS; }
    int minimum_inputs() const { return 2; }
    void knobs(Knob_Callback f) override;
    int knob_changed(Knob* k) override;
    void _validate(bool for_real) override;
    void draw_handle(ViewerContext* ctx) override;
    void bake_to_cornerpin();

    static const Description desc;
    const char* Class() const override { return CLASS; }
    const char* node_help() const override { return HELP; }
};


// Helper function to create a matrix that moves from one set of points to another
// Essentially it is the original cornerpin node
Matrix4 TransformInterpolate::createCornerPinMatrix(xyStruct from[4], xyStruct to[4])
{
    Matrix4 p, q;
    setCornerPinMatrix(from, p);
    setCornerPinMatrix(to, q);

    return q * p.inverse();
}

// Dissolves two matrices based on decomposed components
// This variant is based on DDImage::Matrix4::decompose method
// and has ADDITIONAL compensation through cornerpinning points to target
Matrix4 TransformInterpolate::dissolveMatricesWithCompensation(Matrix4 mA, Matrix4 mB, float factor)
{
    Vector3 pivot;
    Vector3 rotationA;
    Vector3 translationA;
    Vector3 scaleA;
    Vector3 shearA;
    Vector3 rotationB;
    Vector3 translationB;
    Vector3 scaleB;
    Vector3 shearB;
    Vector3 projectionA;
    Vector3 projectionB;

    // Read projection components from matrices
    projectionA = Vector3(mA.a30, mA.a31, mA.a32);
    projectionB = Vector3(mB.a30, mB.a31, mB.a32);

    // Reset projection components to zero
    mA.a30 = mA.a31 = mA.a32 = 0.0;
    mB.a30 = mB.a31 = mB.a32 = 0.0;

    mA.decompose(rotationA, translationA, scaleA, shearA, Matrix4::eZXY);
    mB.decompose(rotationB, translationB, scaleB, shearB, Matrix4::eZXY);

    Vector3 translationOut = lerpVector3(translationA, translationB, factor);
    Vector3 rotationOut = lerpVector3(rotationA, rotationB, factor);
    Vector3 scaleOut = lerpVector3(scaleA, scaleB, factor);
    Vector3 shearOut = lerpVector3(shearA, shearB, factor);
    Vector3 projectionOut = lerpVector3(projectionA, projectionB, factor);

    //Rebuild new matrix from input matrices
    Matrix4 outputMatrix;
    if (_use_shear)
    {
        outputMatrix.set(Matrix4::eSRT, Matrix4::eZXY, pivot, translationOut, rotationOut, scaleOut, shearOut);
    } else {
        outputMatrix.set(Matrix4::eSRT, Matrix4::eZXY, pivot, translationOut, rotationOut, scaleOut, Vector3(0.0, 0.0, 0.0));
    }

    outputMatrix.a30 = projectionOut.x;
    outputMatrix.a31 = projectionOut.y;
    outputMatrix.a32 = projectionOut.z;

    //-----------------------------------------------------------

    // Compensation mechanism is based on calculating corner positions
    // of output matrix and comparing it with corner points of cornerpin based transform
    // Additionally it is weighted by whether B or A or both has perspective component,
    // if either doesn't have it, compensation can be rolled off at either end

    xyStruct scA[4], scB[4], scO[4];
    Vector4 ptA[4], ptB[4], ptO[4];

    // Run image corners through input matrices to produce onscreen points
    float sx, sy;
    sx = widthB;
    sy = heightB;

    // These points are corners of input A
    ptA[0] = mA * Vector4(0.0, 0.0, 1.0, 1.0);
    ptA[1] = mA * Vector4(sx, 0.0, 1.0, 1.0);
    ptA[2] = mA * Vector4(sx, sy, 1.0, 1.0);
    ptA[3] = mA * Vector4(0.0, sy, 1.0, 1.0);
    scA[0].x = ptA[0].x;
    scA[0].y = ptA[0].y;
    scA[1].x = ptA[1].x;
    scA[1].y = ptA[1].y;
    scA[2].x = ptA[2].x;
    scA[2].y = ptA[2].y;
    scA[3].x = ptA[3].x;
    scA[3].y = ptA[3].y;

    // These points are corners of input B
    ptB[0] = mB * Vector4(0.0, 0.0, 1.0, 1.0);
    ptB[1] = mB * Vector4(sx, 0.0, 1.0, 1.0);
    ptB[2] = mB * Vector4(sx, sy, 1.0, 1.0);
    ptB[3] = mB * Vector4(0.0, sy, 1.0, 1.0);
    scB[0].x = ptB[0].x;
    scB[0].y = ptB[0].y;
    scB[1].x = ptB[1].x;
    scB[1].y = ptB[1].y;
    scB[2].x = ptB[2].x;
    scB[2].y = ptB[2].y;
    scB[3].x = ptB[3].x;
    scB[3].y = ptB[3].y;

    // These points are corners of current output
    ptO[0] = outputMatrix * Vector4(0.0, 0.0, 1.0, 1.0);
    ptO[1] = outputMatrix * Vector4(sx, 0.0, 1.0, 1.0);
    ptO[2] = outputMatrix * Vector4(sx, sy, 1.0, 1.0);
    ptO[3] = outputMatrix * Vector4(0.0, sy, 1.0, 1.0);
    scO[0].x = ptO[0].x;
    scO[0].y = ptO[0].y;
    scO[1].x = ptO[1].x;
    scO[1].y = ptO[1].y;
    scO[2].x = ptO[2].x;
    scO[2].y = ptO[2].y;
    scO[3].x = ptO[3].x;
    scO[3].y = ptO[3].y;

    // We need the FROM matrix from cornerpin because otherwise our result is
    // Scaled too big. Multiplying our output with the inverse of "null" gives our final proper result
    Matrix4 nullMatrix;
    xyStruct nullPoints[4];
    nullPoints[0].x = 0.0;
    nullPoints[0].y = 0.0;
    nullPoints[1].x = sx;
    nullPoints[1].y = 0.0;
    nullPoints[2].x = sx;
    nullPoints[2].y = sy;
    nullPoints[3].x = 0.0;
    nullPoints[3].y = sy;

    Matrix4 fromMatrix, toMatrix;
    fromMatrix = outputMatrix;
    toMatrix = mB;
    Matrix4 compensationMatrixB = createCornerPinMatrix(scO, scB);
    Matrix4 compensationMatrixA = createCornerPinMatrix(scO, scA);
    nullMatrix.makeIdentity();

    // Calculate new factor to dissolve from original output to compensationMatrixB
    float compFactorB, compFactorA;

    // This center based compensation is way better than the previous one
    {
        if (_dissolve < 0.5)
        {
            compFactorB = _compensationFactor * (1.0 - _dissolve * 2.0);
            if (compFactorB < 0.0) compFactorB = 0.0;
            if (compFactorB > 1.0) compFactorB = 1.0;
            outputMatrix = dissolveMatricesByComponent(compensationMatrixB, nullMatrix, compFactorB) * outputMatrix;
        } else {
            compFactorA = _compensationFactor * ((_dissolve - 0.5) * 2.0);
            if (compFactorA < 0.0) compFactorA = 0.0;
            if (compFactorA > 1.0) compFactorA = 1.0;
            outputMatrix = dissolveMatricesByComponent(compensationMatrixA, nullMatrix, compFactorA) * outputMatrix;
        }

        Vector4 currentCenter = outputMatrix * Vector4(widthB * 0.5, heightB * 0.5, 1.0, 1.0);
        Vector4 ACenter = mA * Vector4(widthB * 0.5, heightB * 0.5, 1.0, 1.0);
        Vector4 BCenter = mB * Vector4(widthB * 0.5, heightB * 0.5, 1.0, 1.0);
        Vector4 idealCenter = ACenter * _dissolve + BCenter * (1.0 - _dissolve);
        Vector4 delta = idealCenter - currentCenter;
        Matrix4 deltaMatrix;
        deltaMatrix.makeIdentity();
        deltaMatrix.translate(delta.x, delta.y);

        outputMatrix = deltaMatrix * outputMatrix;
    }

    return outputMatrix;
}


// Component based dissolve simply lerps between matrix array values
// This works great except doesn't handle rotations and scaling properly
// Produces identical result to cornerpin point based interpolation
Matrix4 TransformInterpolate::dissolveMatricesByComponent(Matrix4 mA, Matrix4 mB, float factor)
{
    //Rebuild new matrix from input matrices
    Matrix4 outputMatrix;
    Matrix4 imA, imB;
    Matrix4 id;

    id.makeIdentity();

    outputMatrix = mA * factor + mB * (1.0 - factor);

    return outputMatrix;
}


// This method takes two sets of cornerpin points and interpolates from one set to another
// Then produces a matrix from resulting points
Matrix4 TransformInterpolate::dissolveCornerpinPointsToMatrix(xyStruct a[4], xyStruct b[4], float factor)
{
    Matrix4 outputMatrix;
    xyStruct oc[4];
    float invfactor = 1.0 - factor;

    outputMatrix.makeIdentity();

    oc[0].x = a[0].x * factor + b[0].x * invfactor;
    oc[0].y = a[0].y * factor + b[0].y * invfactor;
    oc[1].x = a[1].x * factor + b[1].x * invfactor;
    oc[1].y = a[1].y * factor + b[1].y * invfactor;
    oc[2].x = a[2].x * factor + b[2].x * invfactor;
    oc[2].y = a[2].y * factor + b[2].y * invfactor;
    oc[3].x = a[3].x * factor + b[3].x * invfactor;
    oc[3].y = a[3].y * factor + b[3].y * invfactor;


    // Additional idea
    // Take the incoming points and produce cornerpin matrices from them.
    // Zero out perspective parts to get basic SRT matrix
    // Decompose it to components and interpolate scale, rotation and transform
    // Rebuild resulting matrix
    // Convert resulting matrix to cornerpin representation again
    // Compare it with the cornerpin matrix produced by cornerpin based interpolation
    // There will be residue in some cases
    // Remove the residue by creating a cornerpin matrix that uses result points as "from" and cp result as "to"

    setCornerPinMatrix(oc, outputMatrix);
    return outputMatrix;
}


// Convert four corners of cornerpin to matrix4
// Code from Nuke NDK Cornerpin plugin
void TransformInterpolate::setCornerPinMatrix(xyStruct c[4], Matrix4& q)
{
    q.makeIdentity();
    // Deltas of x and y components of points
    // this gives some kind of perspective component because is non-zero only if sides are not equal length
    double dx3 = (c[0].x - c[1].x) + (c[2].x - c[3].x);
    double dy3 = (c[0].y - c[1].y) + (c[2].y - c[3].y);

    // If deltas are zero, it is affine transform
    if (dx3 == 0 && dy3 == 0) {
        q.a00 = c[1].x - c[0].x;
        q.a01 = c[2].x - c[1].x;
        q.a03 = c[0].x;
        q.a10 = c[1].y - c[0].y;
        q.a11 = c[2].y - c[1].y;
        q.a13 = c[0].y;
    }
    else {
        double dx1 = c[1].x - c[2].x;
        double dy1 = c[1].y - c[2].y;
        double dx2 = c[3].x - c[2].x;
        double dy2 = c[3].y - c[2].y;
        double z = (dx1 * dy2 - dx2 * dy1);
        q.a30 = (dx3 * dy2 - dx2 * dy3) / z;
        q.a31 = (dx1 * dy3 - dx3 * dy1) / z;
        q.a00 = (c[1].x - c[0].x) + q.a30 * c[1].x;
        q.a01 = (c[3].x - c[0].x) + q.a31 * c[3].x;
        q.a03 = c[0].x;
        q.a10 = (c[1].y - c[0].y) + q.a30 * c[1].y;
        q.a11 = (c[3].y - c[0].y) + q.a31 * c[3].y;
        q.a13 = c[0].y;
    }
}

// Cornerpin based dissolve. Converts input matrices to cornerpin representations
// Then does decomposition and reassembly
Matrix4 TransformInterpolate::dissolveMatricesByCornerpin(Matrix4 mA, Matrix4 mB, float factor)
{
    //Rebuild new matrix from input matrices
    Matrix4 outputMatrix;

    // Logic breakdown

    // Convert both input matrices to cornerpin points
    // result is four xy coordinate pairs for both inputs
    // Use input width and height as original point coordinates
    xyStruct scA[4], scB[4];
    Vector4 ptA[4], ptB[4];

    // Run image corners through input matrices to produce onscreen points
    float sx, sy;
    sx = widthB;
    sy = heightB;
    ptA[0] = mA * Vector4(0.0, 0.0, 1.0, 1.0);
    ptA[1] = mA * Vector4(sx, 0.0, 1.0, 1.0);
    ptA[2] = mA * Vector4(sx, sy, 1.0, 1.0);
    ptA[3] = mA * Vector4(0.0, sy, 1.0, 1.0);

    ptB[0] = mB * Vector4(0.0, 0.0, 1.0, 1.0);
    ptB[1] = mB * Vector4(sx, 0.0, 1.0, 1.0);
    ptB[2] = mB * Vector4(sx, sy, 1.0, 1.0);
    ptB[3] = mB * Vector4(0.0, sy, 1.0, 1.0);

    // Copy these onscreen points to xyStructs
    scA[0].x = ptA[0].x;
    scA[0].y = ptA[0].y;
    scA[1].x = ptA[1].x;
    scA[1].y = ptA[1].y;
    scA[2].x = ptA[2].x;
    scA[2].y = ptA[2].y;
    scA[3].x = ptA[3].x;
    scA[3].y = ptA[3].y;

    scB[0].x = ptB[0].x;
    scB[0].y = ptB[0].y;
    scB[1].x = ptB[1].x;
    scB[1].y = ptB[1].y;
    scB[2].x = ptB[2].x;
    scB[2].y = ptB[2].y;
    scB[3].x = ptB[3].x;
    scB[3].y = ptB[3].y;

    // We need the FROM matrix from cornerpin because otherwise our result is
    // Scaled too big. Multiplying our output with the inverse of "null" gives our final proper result
    Matrix4 nullMatrix;
    xyStruct nullPoints[4];
    nullPoints[0].x = 0.0;
    nullPoints[0].y = 0.0;
    nullPoints[1].x = sx;
    nullPoints[1].y = 0.0;
    nullPoints[2].x = sx;
    nullPoints[2].y = sy;
    nullPoints[3].x = 0.0;
    nullPoints[3].y = sy;

    setCornerPinMatrix(nullPoints, nullMatrix);
    outputMatrix = dissolveCornerpinPointsToMatrix(scA, scB, _dissolve);

    return outputMatrix * nullMatrix.inverse();
}

void TransformInterpolate::setMatrix(Matrix4& matrix)
{
    // First we invert the input of B to remove the effect
    // Then we build new matrix that interpolates from mB to mA
    // And add it as new final matrix
    // NB! it is very important to multiply interpolated matrix with matrix, not vice versa!
    // This works, now only interpolation itself is buggy
    matrix = mB.inverse();
    Matrix4 interpolatedMatrix;

    // This variant uses dissolve based on decomposition and optional corner point compensation
    interpolatedMatrix = dissolveMatricesWithCompensation(mA, mB, _dissolve);

    // This variant dissolves each matrix element to matching element
    if (_cornerpin_components)
        interpolatedMatrix = dissolveMatricesByComponent(mA, mB, _dissolve);

    // Cache the full forward transform for the display knob / export.
    _interpolatedMatrix = interpolatedMatrix;

    // Final result is a product of our new matrix and original B matrix inverted (to undo its effect).
    matrix = interpolatedMatrix * matrix;
}

void TransformInterpolate::matrixAt(const OutputContext& context, Matrix4& matrix)
{
    xyStruct sc[4];
    xyStruct dc[4];
    Hash hash;
    knob("Factor")->store(FloatPtr, &_dissolve, hash, context);

    // We need to re-read the matrices here because otherwise there will be a funky
    // effect where image is still but has motion blur
    // Cast inputs to transforms, if successful, change inputs time context and read matrix
    Transform* trf = dynamic_cast<DD::Image::Transform*>(input(0));
    if (trf)
    {
        // Switch context and read matrix
        trf->gotoContext(context, false);
        mB = trf->concat_matrix();

        // Reset the context of input op
        trf->gotoContext(outputContext(), false);
    }

    trf = dynamic_cast<DD::Image::Transform*>(input(1));
    if (trf)
    {
        // Switch context and read matrix
        trf->gotoContext(context, false);
        mA = trf->concat_matrix();

        // Reset the context of input op
        trf->gotoContext(outputContext(), false);
    }

    setMatrix(matrix);
}

// All ui knobs setup here
void TransformInterpolate::knobs(Knob_Callback f)
{
    Float_knob(f, &_dissolve, "Factor");
    SetFlags(f, DD::Image::Knob::ALWAYS_SAVE | DD::Image::Knob::STARTLINE);
    Bool_knob(f, &_cornerpin_components, "Use Cornerpin Based Interpolation");
    SetFlags(f, DD::Image::Knob::ALWAYS_SAVE | DD::Image::Knob::STARTLINE);
    Float_knob(f, &_compensationFactor, "CP compensation");
    SetFlags(f, DD::Image::Knob::ALWAYS_SAVE | DD::Image::Knob::STARTLINE);
    Transform::knobs(f);

    Divider(f, "");
    Int_knob(f, &_bakeStart, "bake_start", "bake start");
    SetFlags(f, DD::Image::Knob::STARTLINE);
    Int_knob(f, &_bakeEnd, "bake_end", "bake end");
    Button(f, "set_range_from_project", "Set Range from Project");
    Button(f, "bake_to_cornerpin", "Bake to CornerPin");
    SetFlags(f, DD::Image::Knob::STARTLINE);
}


int TransformInterpolate::knob_changed(Knob* k)
{
    if (k->is("bake_to_cornerpin"))
    {
        if (_bakeStart > _bakeEnd)
        {
            script_command("nuke.message('Bake Start must be less than or equal to Bake End.')", true, false);
            script_unlock();
            return 1;
        }
        bake_to_cornerpin();
        return 1;
    }
    if (k->is("set_range_from_project"))
    {
        script_command("nuke.root().firstFrame()", true, true);
        int first = atoi(script_result(true));
        script_unlock();

        script_command("nuke.root().lastFrame()", true, true);
        int last = atoi(script_result(true));
        script_unlock();

        knob("bake_start")->set_value(first);
        knob("bake_end")->set_value(last);
        return 1;
    }
    return Transform::knob_changed(k);
}


void TransformInterpolate::bake_to_cornerpin()
{
    if (!input(0) || !input(1)) return;

    int frameStart = _bakeStart;
    int frameEnd   = _bakeEnd;

    // Create a ProgressTask for user feedback + cancel button
    script_command(
        "_ti_progress = nuke.ProgressTask('Baking to CornerPin')\n"
        "_ti_progress.setMessage('Sampling frames...')\n",
        true, false);
    script_unlock();

    // Collect corner positions for each frame
    // Order: BL(0,0), BR(w,0), TR(w,h), TL(0,h) — Nuke CornerPin convention
    enum CPKEY
    {
        X0, Y0,   // to1
        X1, Y1,   // to2
        X2, Y2,   // to3
        X3, Y3,   // to4
        CPKEY_SIZE
    };

    int frameRange = frameEnd - frameStart + 1;
    std::vector<std::vector<float>> keys(CPKEY_SIZE);
    for (int k = 0; k < CPKEY_SIZE; ++k)
        keys[k].reserve(frameRange);

    bool cancelled = false;

    // Sample the interpolated matrix at every frame.
    // Uses node_input() instead of gotoContext() so the Viewer doesn't
    // re-render on every frame — same approach as TransformCopy.
    OutputContext context = outputContext();
    for (int f = frameStart; f <= frameEnd; ++f)
    {
        // --- Update progress & check for cancellation ---
        {
            int pct = (int)((float)(f - frameStart) / (float)frameRange * 100.0f);
            std::ostringstream prog;
            prog << "_ti_progress.setProgress(" << pct << ")\n"
                 << "_ti_progress.setMessage('Frame " << f
                 << " of " << frameEnd << "')\n";
            script_command(prog.str().c_str(), true, false);
            script_unlock();

            script_command("_ti_progress.isCancelled()", true, true);
            const char* res = script_result(true);
            bool is_cancelled = (res && strcmp(res, "True") == 0);
            script_unlock();

            if (is_cancelled)
            {
                cancelled = true;
                break;
            }
        }

        context.setFrame((double)f);

        // Read dissolve factor at this frame
        Hash hash;
        knob("Factor")->store(FloatPtr, &_dissolve, hash, context);

        // Retrieve input transforms at this frame via node_input() —
        // this does NOT trigger viewer updates (unlike gotoContext).
        Transform* trf0 = dynamic_cast<Transform*>(node_input(0, Op::EXECUTABLE_SKIP, &context));
        if (trf0)
        {
            trf0->validate();
            mB = trf0->concat_matrix();
        }

        Transform* trf1 = dynamic_cast<Transform*>(node_input(1, Op::EXECUTABLE_SKIP, &context));
        if (trf1)
        {
            trf1->validate();
            mA = trf1->concat_matrix();
        }

        // Build the interpolated matrix (caches result in _interpolatedMatrix)
        Matrix4 mat;
        setMatrix(mat);

        // Use the cached forward matrix to transform format corners
        float w = widthB;
        float h = heightB;

        Vector4 v0 = _interpolatedMatrix.transform({0, 0, 1, 1}).divide_w();
        Vector4 v1 = _interpolatedMatrix.transform({w, 0, 1, 1}).divide_w();
        Vector4 v2 = _interpolatedMatrix.transform({w, h, 1, 1}).divide_w();
        Vector4 v3 = _interpolatedMatrix.transform({0, h, 1, 1}).divide_w();

        keys[X0].push_back(v0.x); keys[Y0].push_back(v0.y);
        keys[X1].push_back(v1.x); keys[Y1].push_back(v1.y);
        keys[X2].push_back(v2.x); keys[Y2].push_back(v2.y);
        keys[X3].push_back(v3.x); keys[Y3].push_back(v3.y);
    }

    // Clean up progress bar
    script_command("del _ti_progress\n", true, false);
    script_unlock();

    // If user cancelled, bail without creating the node
    if (cancelled) return;

    // Build TCL script using Nuke {curve R xN ...} syntax
    // The knob value interleaving matches CornerPin2D's to1..to4 knobs
    const char* const knobTokens[CPKEY_SIZE + 1] =
    {
        // to1 {X0 Y0} to2 {X1 Y1} to3 {X2 Y2} to4 {X3 Y3}
        "'to1 {",
        " ",
        " } to2 {",
        " ",
        " } to3 {",
        " ",
        " } to4 {",
        " ",
        " } ",
    };

    int w = (int)widthB;
    int h = (int)heightB;

    std::stringstream script;
    script << "nukescripts.clear_selection_recursive();"
           << "nuke.autoplace(nuke.createNode('CornerPin2D',";

    for (int k = 0; k < CPKEY_SIZE; ++k)
    {
        script << knobTokens[k];
        script << "{curve R x" << frameStart << " ";
        for (int f = 0; f < frameRange; ++f)
            script << keys[k][f] << " ";
        script << "}";
    }

    script << knobTokens[CPKEY_SIZE];
    script << " label {TransformInterpolate \\nframes " << frameStart << " - " << frameEnd << "}";
    script << " from1 {0 0} from2 {" << w << " 0} from3 {" << w << " " << h << "} from4 {0 " << h << "}";
    script << "'));";

    script_command(script.str().c_str(), true, false);
    script_unlock();
}


void TransformInterpolate::_validate(bool for_real)
{
    // Validate all inputs
    for (int i = 0; i < MAX_INPUTS; ++i)
    {
        if (input(i))
            input(i)->validate(for_real);
    }

    mA.makeIdentity();
    mB.makeIdentity();

    widthB = input(0)->format().width();
    heightB = input(0)->format().height();
    widthA = input(1)->format().width();
    heightA = input(1)->format().height();


    // Read concatenated matrices from inputs,
    // these are used directly if we don't need to apply motion blur
    Transform* trf = dynamic_cast<DD::Image::Transform*>(input(0));
    if (trf)
    {
        mB = trf->concat_matrix();
    }

    trf = dynamic_cast<DD::Image::Transform*>(input(1));
    if (trf)
    {
        mA = trf->concat_matrix();
    }

    setMatrix(*matrix());

    Transform::_validate(for_real);
}

// Draws handles and stuff from upstream nodes
void TransformInterpolate::draw_handle(ViewerContext* ctx)
{
    if (ctx->draw_lines())
    {
        const Info& i = concat_input_->info();
        glColor(ctx->node_color());
        gl_rectangle((float)i.x(), (float)i.y(), (float)i.r(), (float)i.t());
    }

    Transform::draw_handle(ctx);
}

static Iop* build(Node* node) { return new TransformInterpolate(node); }
const Iop::Description TransformInterpolate::desc(CLASS, "Transform/TransformInterpolate", build);
