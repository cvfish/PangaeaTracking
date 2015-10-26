#include "main_engine/rendering/CCamera.h"

CCamera::CCamera(double K[3][3], double camPose[6], unsigned int W,
                 unsigned int H)
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            _K(i,j) = K[i][j];
            cout << _K(i,j) << endl;
        }
        _t(i) = camPose[i+3];
    }

    _W = W;
    _H = H;

    // get the rotation matrix from axis-angle representation
    // double angle = Vector3d::Map(camPose).norm();
    // Vector3d axis = Vector3d::Map(camPose)/angle;
    Vector3d axis;
    double angle = Map<Vector3d>(camPose).norm();
    if(angle != 0)
    axis = Map<Vector3d>(camPose)/angle;
    else
    axis << 1, 0, 0;
    _R = AngleAxisd(angle, axis).toRotationMatrix();

    setDerivedParams();
}

CCamera::CCamera(double K[3][3], double R[3][3], double t[3],
    unsigned int W, unsigned int H)
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            _K(i,j) = K[i][j];
            _R(i,j) = R[i][j];
        }
        _t(i) = t[i];
    }

    _W = W;
    _H = H;

    setDerivedParams();

}

CCamera::~CCamera()
{
}

void CCamera::setDerivedParams()
{
    _K(0,1) = 0;
    _K(1,0) = 0;
    _K(2,0) = 0;
    _K(2,1) = 0;

    _Kinv.fill(0);

    _Kinv(0,0) = 1 / _K(0,0);
    _Kinv(1,1) = 1 / _K(1,1);
    _Kinv(0,2) = - _K(0,2) / _K(0,0);
    _Kinv(1,2) = - _K(1,2) / _K(1,1);
    _Kinv(2,2) = 1;

    _KR = _K * _R;
    _Kt = _K * _t;

    _Rtrans = _R.transpose();

    if(_K(0,2) == 0)
    _IsOrtho = true;
    else
    _IsOrtho = false;

}

Matrix4d CCamera::getProjectionMatrix(double zNear, double zFar) const
{
    double au = _K(0,0);
    double u0 = _K(0,2);
    double av = _K(1,1);
    double v0 = _K(1,2);

    double factor = 1.0;

    u0 = (_W / 2.0) + factor * (u0 - (_W / 2.0));
    v0 = (_H / 2.0) + factor * (v0 - (_H / 2.0));

    Matrix4d P = MatrixXd::Zero(4,4);

    if(!_IsOrtho)
    {
        P(0,0) = 2 * factor * au / _W;
        P(1,1) = 2 * factor * av / _H;

        P(0,2) = (1.0 - (2 * u0 / _W));
        P(1,2) = -(1.0 - (2 * v0 / _H));

        P(2,2) = (zNear + zFar) / (zNear - zFar);
        P(2,3) = (2 * zNear * zFar) / (zNear - zFar);

        P(3,2) = -1.0;
    }
    else
    {
        P(0,0) = 2/_W;
        P(1,1) = 2/_H;

        P(0,3) = -1;
        P(1,3) = 1;

        zNear = -1000;

        P(2,2) = 2 / (zNear - zFar);
        P(2,3) = (zNear + zFar) / (zNear - zFar);

        P(3,3) = 1;
    }

    return P;

}

Matrix4d CCamera::getModelViewMatrix() const
{
    Matrix3d Flip = MatrixXd::Identity(3,3);

    Flip(1,1) = -1.0;
    Flip(2,2) = -1.0;

    Matrix4d MV = MatrixXd::Identity(4,4);
    MV.block(0,0,3,3) = Flip * _R;
    MV.block(0,3,3,1) = Flip * _t;

    return MV;
}