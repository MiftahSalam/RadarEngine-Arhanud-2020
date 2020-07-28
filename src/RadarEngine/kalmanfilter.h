#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QObject>

template <typename Ty, int N, int M = N>

struct Matrix {
  typedef Ty value_type;

  union {
    struct {
      Ty element[N][M];
    };
    struct {
      Ty flatten[N * M];
    };
  };

  Ty& operator()(const int r, const int c) {
    Q_ASSERT(r >= 0 && r < N);
    Q_ASSERT(c >= 0 && c < M);
    return element[r][c];
  }

  const Ty operator()(const int r, const int c) const {
    Q_ASSERT(r >= 0 && r < N);
    Q_ASSERT(c >= 0 && c < M);
    return element[r][c];
  }

  Matrix<Ty, M, N> Transpose() const {
    Matrix<Ty, M, N> result;
    for (int r = 0; r < N; ++r) {
      for (int c = 0; c < M; ++c) {
        result.element[c][r] = element[r][c];
      }
    }
    return result;
  }

  Matrix<Ty, M, N> Init(Ty value) const {
    Matrix<Ty, M, N> result;
    for (int e = 0; e < M * N; ++e) {
      result.flatten[e] = Ty(value);
    }
    return result;
  }

  Matrix<Ty, N, M> Inverse();

  Matrix<Ty, N, N> Identity() {
    Matrix<Ty, N, N> result = Matrix<Ty, N, N>();
    for (int i = 0; i < N * N; ++i) result.flatten[i] = Ty(0);
    for (int i = 0; i < N; ++i) result.element[i][i] = Ty(1);
    return result;
  }
};

namespace detail {

template <typename Ty, int N, int M>
struct inverse;

template <typename Ty>
struct inverse<Ty, 2, 2> {
  Matrix<Ty, 2, 2> operator()(const Matrix<Ty, 2, 2>& a) {
    Matrix<Ty, 2, 2> result;
    Ty det = a.element[0][0] * a.element[1][1] - a.element[0][1] * a.element[1][0];
    Q_ASSERT(det != 0);

    result.element[0][0] = a.element[1][1] / det;
    result.element[1][1] = a.element[0][0] / det;
    result.element[0][1] = -a.element[0][1] / det;
    result.element[1][0] = -a.element[1][0] / det;
    return result;
  }
};

}

template <typename Ty, int N, int M>
Matrix<Ty, N, M> Matrix<Ty, N, M>::Inverse() {
  return detail::inverse<Ty, N, M>()(*this);
}

template <typename Ty, int N, int M, int P>
Matrix<Ty, N, P> operator*(const Matrix<Ty, N, M>& a, const Matrix<Ty, M, P>& b) {
  Matrix<Ty, N, P> result;

  for (int r = 0; r < N; ++r) {
    for (int c = 0; c < P; ++c) {
      Ty accum = Ty(0);
      for (int i = 0; i < M; ++i) {
        accum += a.element[r][i] * b.element[i][c];
      }
      result.element[r][c] = accum;
    }
  }
  return result;
}

template <typename Ty, int N, int M>
Matrix<Ty, N, M> operator-(const Matrix<Ty, N, M>& a) {
  Matrix<Ty, N, M> result;
  for (int e = 0; e < N * M; ++e) result.flatten[e] = -a.flatten[e];
  return result;
}

#define MATRIX_WITH_MATRIX_OPERATOR(op_symbol, op)                                            \
  template <typename Ty, int N, int M>                                                        \
  Matrix<Ty, N, M> operator op_symbol(const Matrix<Ty, N, M>& a, const Matrix<Ty, N, M>& b) { \
    Matrix<Ty, N, M> result;                                                                  \
    for (int e = 0; e < N * M; ++e) result.flatten[e] = a.flatten[e] op b.flatten[e];         \
    return result;                                                                            \
  }

MATRIX_WITH_MATRIX_OPERATOR(+, +)
MATRIX_WITH_MATRIX_OPERATOR(-, -)
#undef MATRIX_WITH_MATRIX_OPERATOR

#define MATRIX_WITH_SCALAR_OPERATOR(op_symbol, op)                              \
  template <typename Ty, int N, int M>                                          \
  Matrix<Ty, N, M> operator op_symbol(const Matrix<Ty, N, M>& a, Ty scalar) {   \
    Matrix<Ty, N, M> result;                                                    \
    for (int e = 0; e < N * M; ++e) result.flatten[e] = a.flatten[e] op scalar; \
    return result;                                                              \
  }

MATRIX_WITH_SCALAR_OPERATOR(+, +)
MATRIX_WITH_SCALAR_OPERATOR(-, -)
MATRIX_WITH_SCALAR_OPERATOR(*, *)
MATRIX_WITH_SCALAR_OPERATOR(/, /)
#undef MATRIX_WITH_SCALAR_OPERATOR

template <typename Ty, int N, int M>
Matrix<Ty, N, M> operator+(Ty scalar, const Matrix<Ty, N, M>& a) {
  return a + scalar;
}

template <typename Ty, int N, int M>
Matrix<Ty, N, M> operator*(Ty scalar, const Matrix<Ty, N, M>& a) {
  return a * scalar;
}

template <typename Ty, int N, int M>
Matrix<Ty, N, M> operator-(Ty scalar, const Matrix<Ty, N, M>& a) {
  return -a + scalar;
}


#define NOISE (0.015)  // Allowed covariance of target speed in lat and lon
                      // critical for the performance of target tracking
                      // lower value makes target go straight
                      // higher values allow target to make curves


class LocalPosition {

public:
  double lat;
  double lon;
  double dlat_dt;  // meters per second
  double dlon_dt;
  double sd_speed_m_s;  // standard deviation of the speed m / sec

};

class Polar {
 public:
  int angle;
  int r;
  quint64 time;  // wxGetUTCTimeMillis
};


class KalmanFilter : public QObject
{
    Q_OBJECT
public:
    explicit KalmanFilter(QObject *parent = 0);
    void SetMeasurement(Polar* pol, LocalPosition* x, Polar* expected, int range);
    void Update_P();
    void Predict(LocalPosition* xx, double delta_time);
    void ResetFilter();

    Matrix<double, 4> A;
    Matrix<double, 4> AT;
    Matrix<double, 4, 2> W;
    Matrix<double, 2, 4> WT;
    Matrix<double, 2, 4> H;
    Matrix<double, 4, 2> HT;
    Matrix<double, 4> P;
    Matrix<double, 2> Q;
    Matrix<double, 2> R;
    Matrix<double, 4, 2> K;
    Matrix<double, 4> I;

};

#endif // KALMANFILTER_H
