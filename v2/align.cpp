#include "pch.h"
#include "BLCalibration.h"
#include "utils.h"

bool jacobi_4x4(double* A, double* D, double* U);

bool align(double M_end[3][3],
    double X0, double Y0, double Z0,
    double X1, double Y1, double Z1,
    double X2, double Y2, double Z2,
    double R[3][3], double T[3])
{
    // Centroids:
    double C_start[3] = {}, C_end[3] = {};
    for (int i = 0; i < 3; i++) C_end[i] = (M_end[0][i] + M_end[1][i] + M_end[2][i]) / 3;
    C_start[0] = (X0 + X1 + X2) / 3;
    C_start[1] = (Y0 + Y1 + Y2) / 3;
    C_start[2] = (Z0 + Z1 + Z2) / 3;

    // Covariance matrix s:
    double s[3 * 3] = {};
    for (int j = 0; j < 3; j++) {
        s[0 * 3 + j] = (X0 * M_end[0][j] + X1 * M_end[1][j] + X2 * M_end[2][j]) / 3 - C_end[j] * C_start[0];
        s[1 * 3 + j] = (Y0 * M_end[0][j] + Y1 * M_end[1][j] + Y2 * M_end[2][j]) / 3 - C_end[j] * C_start[1];
        s[2 * 3 + j] = (Z0 * M_end[0][j] + Z1 * M_end[1][j] + Z2 * M_end[2][j]) / 3 - C_end[j] * C_start[2];
    }

    double Qs[16] = {}, evs[4] = {}, U[16] = {};

    Qs[0 * 4 + 0] = s[0 * 3 + 0] + s[1 * 3 + 1] + s[2 * 3 + 2];
    Qs[1 * 4 + 1] = s[0 * 3 + 0] - s[1 * 3 + 1] - s[2 * 3 + 2];
    Qs[2 * 4 + 2] = s[1 * 3 + 1] - s[2 * 3 + 2] - s[0 * 3 + 0];
    Qs[3 * 4 + 3] = s[2 * 3 + 2] - s[0 * 3 + 0] - s[1 * 3 + 1];

    Qs[1 * 4 + 0] = Qs[0 * 4 + 1] = s[1 * 3 + 2] - s[2 * 3 + 1];
    Qs[2 * 4 + 0] = Qs[0 * 4 + 2] = s[2 * 3 + 0] - s[0 * 3 + 2];
    Qs[3 * 4 + 0] = Qs[0 * 4 + 3] = s[0 * 3 + 1] - s[1 * 3 + 0];
    Qs[2 * 4 + 1] = Qs[1 * 4 + 2] = s[1 * 3 + 0] + s[0 * 3 + 1];
    Qs[3 * 4 + 1] = Qs[1 * 4 + 3] = s[2 * 3 + 0] + s[0 * 3 + 2];
    Qs[3 * 4 + 2] = Qs[2 * 4 + 3] = s[2 * 3 + 1] + s[1 * 3 + 2];

    jacobi_4x4(Qs, evs, U);

    // Looking for the largest eigen value:
    int i_ev = 0;
    double ev_max = evs[i_ev];
    for (int i = 1; i < 4; i++)
        if (evs[i] > ev_max)
            ev_max = evs[i_ev = i];

    // Quaternion:
    double q[4];
    for (int i = 0; i < 4; i++)
        q[i] = U[i * 4 + i_ev];

    double q02 = q[0] * q[0], q12 = q[1] * q[1], q22 = q[2] * q[2], q32 = q[3] * q[3];
    double q0_1 = q[0] * q[1], q0_2 = q[0] * q[2], q0_3 = q[0] * q[3];
    double q1_2 = q[1] * q[2], q1_3 = q[1] * q[3];
    double q2_3 = q[2] * q[3];

    R[0][0] = q02 + q12 - q22 - q32;
    R[0][1] = 2. * (q1_2 - q0_3);
    R[0][2] = 2. * (q1_3 + q0_2);

    R[1][0] = 2. * (q1_2 + q0_3);
    R[1][1] = q02 + q22 - q12 - q32;
    R[1][2] = 2. * (q2_3 - q0_1);

    R[2][0] = 2. * (q1_3 - q0_2);
    R[2][1] = 2. * (q2_3 + q0_1);
    R[2][2] = q02 + q32 - q12 - q22;

    for (int i = 0; i < 3; i++)
        T[i] = C_end[i] - (R[i][0] * C_start[0] + R[i][1] * C_start[1] + R[i][2] * C_start[2]);

    return true;
}


bool jacobi_4x4(double* A, double* D, double* U)
{
    double B[4] = {}, Z[4] = {};
    double Id[16] = { 1., 0., 0., 0.,
                     0., 1., 0., 0.,
                     0., 0., 1., 0.,
                     0., 0., 0., 1. };

    memcpy(U, Id, 16 * sizeof(double));

    B[0] = A[0]; B[1] = A[5]; B[2] = A[10]; B[3] = A[15];
    memcpy(D, B, 4 * sizeof(double));

    for (int iter = 0; iter < 50; iter++) {
        double sum = fabs(A[1]) + fabs(A[2]) + fabs(A[3]) + fabs(A[6]) + fabs(A[7]) + fabs(A[11]);

        if (sum == 0.0)
            return true;

        double tresh = (iter < 3) ? 0.2 * sum / 16. : 0.0;
        for (int i = 0; i < 3; i++) {
            double* pAij = A + 5 * i + 1;
            for (int j = i + 1; j < 4; j++) {
                double Aij = *pAij;
                double eps_machine = 100.0 * fabs(Aij);

                if (iter > 3 && fabs(D[i]) + eps_machine == fabs(D[i]) && fabs(D[j]) + eps_machine == fabs(D[j]))
                    *pAij = 0.0;
                else if (fabs(Aij) > tresh) {
                    double hh = D[j] - D[i], t;
                    if (fabs(hh) + eps_machine == fabs(hh))
                        t = Aij / hh;
                    else {
                        double theta = 0.5 * hh / Aij;
                        t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
                        if (theta < 0.0) t = -t;
                    }

                    hh = t * Aij;
                    Z[i] -= hh;
                    Z[j] += hh;
                    D[i] -= hh;
                    D[j] += hh;
                    *pAij = 0.0;

                    double c = 1.0 / sqrt(1 + t * t);
                    double s = t * c;
                    double tau = s / (1.0 + c);
                    for (int k = 0; k <= i - 1; k++) {
                        double g = A[k * 4 + i], h = A[k * 4 + j];
                        A[k * 4 + i] = g - s * (h + g * tau);
                        A[k * 4 + j] = h + s * (g - h * tau);
                    }
                    for (int k = i + 1; k <= j - 1; k++) {
                        double g = A[i * 4 + k], h = A[k * 4 + j];
                        A[i * 4 + k] = g - s * (h + g * tau);
                        A[k * 4 + j] = h + s * (g - h * tau);
                    }
                    for (int k = j + 1; k < 4; k++) {
                        double g = A[i * 4 + k], h = A[j * 4 + k];
                        A[i * 4 + k] = g - s * (h + g * tau);
                        A[j * 4 + k] = h + s * (g - h * tau);
                    }
                    for (int k = 0; k < 4; k++) {
                        double g = U[k * 4 + i], h = U[k * 4 + j];
                        U[k * 4 + i] = g - s * (h + g * tau);
                        U[k * 4 + j] = h + s * (g - h * tau);
                    }
                }
                pAij++;
            }
        }

        for (int i = 0; i < 4; i++) B[i] += Z[i];
        memcpy(D, B, 4 * sizeof(double));
        memset(Z, 0, 4 * sizeof(double));
    }

    return false;
}
