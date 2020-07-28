#include "kalmanfilter.h"
#include "radarengine_global.h"

static Matrix<double, 4, 2> ZeroMatrix42;
static Matrix<double, 2, 4> ZeroMatrix24;
static Matrix<double, 4> ZeroMatrix4;
static Matrix<double, 2> ZeroMatrix2;

KalmanFilter::KalmanFilter(QObject *parent) :
    QObject(parent)
{
    I = I.Identity();
    Q = ZeroMatrix2;
    R = ZeroMatrix2;

    ResetFilter();
    qDebug()<<Q_FUNC_INFO;
}
void KalmanFilter::ResetFilter() {
    // reset the filter to use  it for a new case
    A = I;

    // transpose of A
    AT = A;

    // Jacobian matrix of partial derivatives dfi / dwj
    W = ZeroMatrix42;
    W(2, 0) = 1.;
    W(3, 1) = 1.;

    // transpose of W
    WT = ZeroMatrix24;
    WT(0, 2) = 1.;
    WT(1, 3) = 1.;

    // Observation matrix, jacobian of observation function h
    // dhi / dvj
    // angle = atan2 (lat,lon) * 2048 / (2 * pi) + v1
    // r = sqrt(x * x + y * y) + v2
    // v is measurement noise
    H = ZeroMatrix24;

    // Transpose of observation matrix
    HT = ZeroMatrix42;

    // Jacobian V, dhi / dvj
    // As V is the identity matrix, it is left out of the calculation of the Kalman gain

    // P estimate error covariance
    // initial values follow
    // P(1, 1) = .0000027 * range * range;   ???
    P = ZeroMatrix4;
    P(0, 0) = 20.;
    P(1, 1) = P(1, 1);
    P(2, 2) = 4.;
    P(3, 3) = 4.;

    // Q Process noise covariance matrix
    Q(0, 0) = NOISE;  // variance in lat speed, (m / sec)2
    Q(1, 1) = NOISE;  // variance in lon speed, (m / sec)2

    // R measurement noise covariance matrix
    R(0, 0) = 100.0;  // variance in the angle 3.0
    R(1, 1) = 25.;    // variance in radius  .5
}

void KalmanFilter::Predict(LocalPosition* xx, double delta_time) {
    Matrix<double, 4, 1> X;
    X(0, 0) = xx->lat;
    X(1, 0) = xx->lon;
    X(2, 0) = xx->dlat_dt;
    X(3, 0) = xx->dlon_dt;
    A(0, 2) = delta_time;  // time in seconds
    A(1, 3) = delta_time;
    /*
  qDebug()<<Q_FUNC_INFO<<"xx lat"<<X(0, 0)<<"xx lon"<<X(1, 0)
         <<"ddlat "<<X(2, 0)<<"dlon"<<X(3, 0)
        <<"A(0, 0)"<<A(0, 0)<<"A(0, 1)"<<A(0, 1)<<"A(0, 2)"<<A(0, 2)
          <<"A(0, 3)"<<A(0, 3);
  */

    AT(2, 0) = delta_time;
    AT(3, 1) = delta_time;

    X = A * X;
    xx->lat = X(0, 0);
    xx->lon = X(1, 0);
    xx->dlat_dt = X(2, 0);
    xx->dlon_dt = X(3, 0);
    xx->sd_speed_m_s = sqrt((P(2, 2) + P(3, 3)) / 2.);
    return;
}

void KalmanFilter::Update_P() {
    // calculate apriori P
    // separated from the predict to prevent the update being done both in pass 1 and pass2

    P = A * P * AT + W * Q * WT;
    return;
}
void KalmanFilter::SetMeasurement(Polar* pol, LocalPosition* x, Polar* expected, int range) {
    // pol measured angular position
    // x expected local position
    // expected, same but in polar coordinates
#define SQUARED(x) ((x) * (x))
    double q_sum = SQUARED(x->lon) + SQUARED(x->lat);

    double c = 2048. / (2. * M_PI);
    H(0, 0) = -c * x->lon / q_sum;
    H(0, 1) = c * x->lat / q_sum;

    q_sum = sqrt(q_sum);
    H(1, 0) = x->lat / q_sum * 512. / (double)range;
    H(1, 1) = x->lon / q_sum * 512. / (double)range;

    HT = H.Transpose();

    Matrix<double, 2, 1> Z;
    Z(0, 0) = (double)(pol->angle - expected->angle);  // Z is  difference between measured and expected
    if (Z(0, 0) > LINES_PER_ROTATION / 2) {
        Z(0, 0) -= LINES_PER_ROTATION;
    }
    if (Z(0, 0) < -LINES_PER_ROTATION / 2) {
        Z(0, 0) += LINES_PER_ROTATION;
    }
    Z(1, 0) = (double)(pol->r - expected->r);

    Matrix<double, 4, 1> X;
    X(0, 0) = x->lat;
    X(1, 0) = x->lon;
    X(2, 0) = x->dlat_dt;
    X(3, 0) = x->dlon_dt;

    // calculate Kalman gain
    K = P * HT * ((H * P * HT + R).Inverse());

    // calculate apostriori expected position
    X = X + K * Z;
    x->lat = X(0, 0);
    x->lon = X(1, 0);
    x->dlat_dt = X(2, 0);
    x->dlon_dt = X(3, 0);

    // update covariance P
    P = (I - K * H) * P;
    x->sd_speed_m_s = sqrt((P(2, 2) + P(3, 3)) / 2.);  // rough approximation of standard dev of speed
    return;
}

