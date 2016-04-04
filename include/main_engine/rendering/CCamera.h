#ifndef _CCAMERA_H
#define _CCAMERA_H

#include "../utils/global.h"

class CCamera
{

    // a camera class based on eigen3

    // By convention with this class:
    // u = 2D coord in image plane
    // c = 3D coord in camera centred coords
    // X = 3D coord in world coords

protected:

    Matrix3d _K;
    Matrix3d _R;
    Vector3d _t;

    // derived parameters
    Matrix3d _Kinv;
    Matrix3d _KR;
    Vector3d _Kt;

    Matrix3d _Rtrans;

    // Image dimensions

    unsigned int _W;
    unsigned int _H;

    // Orthographic camera ?
    bool _IsOrtho;

    void setDerivedParams();

public:

    CCamera(double K[3][3], double camPose[6], unsigned int W,
        unsigned int H);

    CCamera(double K[3][3], double R[3][3], double t[3], unsigned int W,
        unsigned int H);

    ~CCamera();

    Vector3d worldToCamera(vector<double>& X) const
    {
        return _R * Vector3d::Map(&X[0]) + _t;
    }

    Vector2d worldToPixel(vector<double>& X) const
    {
        Vector3d temp = _KR * Vector3d::Map(&X[0]) + _Kt;
        return Vector2d(temp[0]/temp[2], temp[1]/temp[2]);
    }

    const Matrix3d& K() const
    {
        return _K;
    }

    const Matrix3d& R() const
    {
        return _R;
    }

    const Vector3d& t() const
    {
        return _t;
    }

    unsigned int W() const
    {
        return _W;
    }

    unsigned int H() const
    {
        return _H;
    }

    bool IsOrtho() const
    {
        return _IsOrtho;
    }

    const Matrix3d& Kinv() const
    {
        return _Kinv;
    }

    const Matrix3d& Rtrans() const
    {
        return _Rtrans;
    }

    const Matrix3d& KR() const
    {
        return _KR;
    }

    const Vector3d& Kt() const
    {
        return _Kt;
    }

    inline Vector3d OC() const
    {
        return (- _Rtrans * _t );
    }

    inline Vector3d OA() const
    {
        Vector3d zDir;
        zDir << 0, 0, 1;
        return _Rtrans * zDir;
    }

    Matrix4d getProjectionMatrix(double zNear = 0.1, double zFar = 1000.0) const;

    Matrix4d getModelViewMatrix() const;

};

#endif
