#include "pch.h"
#include "BLCalibration.h"
#include "utils.h"

int solve_deg2(double a, double b, double c, double& x1, double& x2);
int solve_deg3(double a, double b, double c, double d,
    double& x0, double& x1, double& x2);
int solve_deg4(double a, double b, double c, double d, double e,
    double& x0, double& x1, double& x2, double& x3);

int solveForLengths(double lengths[4][3], double distances[3], double cosines[3])
{
    double p = cosines[0] * 2;
    double q = cosines[1] * 2;
    double r = cosines[2] * 2;

    double inv_d22 = 1. / (distances[2] * distances[2]);
    double a = inv_d22 * (distances[0] * distances[0]);
    double b = inv_d22 * (distances[1] * distances[1]);

    double a2 = a * a, b2 = b * b, p2 = p * p, q2 = q * q, r2 = r * r;
    double pr = p * r, pqr = q * pr;

    // Check reality condition (the four points should not be coplanar)
    if (p2 + q2 + r2 - pqr - 1 == 0)
        return 0;

    double ab = a * b, a_2 = 2 * a;

    double A = -2 * b + b2 + a2 + 1 + ab * (2 - r2) - a_2;

    // Check reality condition
    if (A == 0) return 0;

    double a_4 = 4 * a;

    double B = q * (-2 * (ab + a2 + 1 - b) + r2 * ab + a_4) + pr * (b - b2 + ab);
    double C = q2 + b2 * (r2 + p2 - 2) - b * (p2 + pqr) - ab * (r2 + pqr) + (a2 - a_2) * (2 + q2) + 2;
    double D = pr * (ab - b2 + b) + q * ((p2 - 2) * b + 2 * (ab - a2) + a_4 - 2);
    double E = 1 + 2 * (b - a - ab) + b2 - b * p2 + a2;

    double temp = (p2 * (a - 1 + b) + r2 * (a - 1 - b) + pqr - a * pqr);
    double b0 = b * temp * temp;
    // Check reality condition
    if (b0 == 0)
        return 0;

    double real_roots[4];
    int n = solve_deg4(A, B, C, D, E, real_roots[0], real_roots[1], real_roots[2], real_roots[3]);

    if (n == 0)
        return 0;

    int nb_solutions = 0;
    double r3 = r2 * r, pr2 = p * r2, r3q = r3 * q;
    double inv_b0 = 1. / b0;

    // For each solution of x
    for (int i = 0; i < n; i++) {
        double x = real_roots[i];

        // Check reality condition
        if (x <= 0)
            continue;

        double x2 = x * x;

        double b1 =
            ((1 - a - b) * x2 + (q * a - q) * x + 1 - a + b) *
            (((r3 * (a2 + ab * (2 - r2) - a_2 + b2 - 2 * b + 1)) * x +

                (r3q * (2 * (b - a2) + a_4 + ab * (r2 - 2) - 2) + pr2 * (1 + a2 + 2 * (ab - a - b) + r2 * (b - b2) + b2))) * x2 +

                (r3 * (q2 * (1 - 2 * a + a2) + r2 * (b2 - ab) - a_4 + 2 * (a2 - b2) + 2) + r * p2 * (b2 + 2 * (ab - b - a) + 1 + a2) + pr2 * q * (a_4 + 2 * (b - ab - a2) - 2 - r2 * b)) * x +

                2 * r3q * (a_2 - b - a2 + ab - 1) + pr2 * (q2 - a_4 + 2 * (a2 - b2) + r2 * b + q2 * (a2 - a_2) + 2) +
                p2 * (p * (2 * (ab - a - b) + a2 + b2 + 1) + 2 * q * r * (b + a_2 - a2 - ab - 1)));

        // Check reality condition
        if (b1 <= 0)
            continue;

        double y = inv_b0 * b1;
        double v = x2 + y * y - x * y * r;

        if (v <= 0)
            continue;

        double Z = distances[2] / sqrt(v);
        double X = x * Z;
        double Y = y * Z;

        lengths[nb_solutions][0] = X;
        lengths[nb_solutions][1] = Y;
        lengths[nb_solutions][2] = Z;

        nb_solutions++;
    }

    return nb_solutions;
}


//==================================================================//
//OpenCVÔ´ÂëÂ·¾¶ opencv/modules/calib3d/src/polynom_solver.cpp
int solve_deg2(double a, double b, double c, double& x1, double& x2)
{
    double delta = b * b - 4 * a * c;

    if (delta < 0) return 0;

    double inv_2a = 0.5 / a;

    if (delta == 0) {
        x1 = -b * inv_2a;
        x2 = x1;
        return 1;
    }

    double sqrt_delta = sqrt(delta);
    x1 = (-b + sqrt_delta) * inv_2a;
    x2 = (-b - sqrt_delta) * inv_2a;
    return 2;
}


/// Reference : Eric W. Weisstein. "Cubic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/CubicEquation.html
/// \return Number of real roots found.
int solve_deg3(double a, double b, double c, double d,
    double& x0, double& x1, double& x2)
{
    if (a == 0)
    {
        // Solve second order system
        if (b == 0)
        {
            // Solve first order system
            if (c == 0)
                return 0;

            x0 = -d / c;
            return 1;
        }

        x2 = 0;
        return solve_deg2(b, c, d, x0, x1);
    }

    // Calculate the normalized form x^3 + a2 * x^2 + a1 * x + a0 = 0
    double inv_a = 1. / a;
    double b_a = inv_a * b, b_a2 = b_a * b_a;
    double c_a = inv_a * c;
    double d_a = inv_a * d;

    // Solve the cubic equation
    double Q = (3 * c_a - b_a2) / 9;
    double R = (9 * b_a * c_a - 27 * d_a - 2 * b_a * b_a2) / 54;
    double Q3 = Q * Q * Q;
    double D = Q3 + R * R;
    double b_a_3 = (1. / 3.) * b_a;

    if (Q == 0) {
        if (R == 0) {
            x0 = x1 = x2 = -b_a_3;
            return 3;
        }
        else {
            x0 = pow(2 * R, 1 / 3.0) - b_a_3;
            return 1;
        }
    }

    if (D <= 0) {
        // Three real roots
        double theta = acos(R / sqrt(-Q3));
        double sqrt_Q = sqrt(-Q);
        x0 = 2 * sqrt_Q * cos(theta / 3.0) - b_a_3;
        x1 = 2 * sqrt_Q * cos((theta + 2 * CV_PI) / 3.0) - b_a_3;
        x2 = 2 * sqrt_Q * cos((theta + 4 * CV_PI) / 3.0) - b_a_3;

        return 3;
    }

    // D > 0, only one real root
    double AD = pow(fabs(R) + sqrt(D), 1.0 / 3.0) * (R > 0 ? 1 : (R < 0 ? -1 : 0));
    double BD = (AD == 0) ? 0 : -Q / AD;

    // Calculate the only real root
    x0 = AD + BD - b_a_3;

    return 1;
}

/// Reference : Eric W. Weisstein. "Quartic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/QuarticEquation.html
/// \return Number of real roots found.
int solve_deg4(double a, double b, double c, double d, double e,
    double& x0, double& x1, double& x2, double& x3)
{
    if (a == 0) {
        x3 = 0;
        return solve_deg3(b, c, d, e, x0, x1, x2);
    }

    // Normalize coefficients
    double inv_a = 1. / a;
    b *= inv_a; c *= inv_a; d *= inv_a; e *= inv_a;
    double b2 = b * b, bc = b * c, b3 = b2 * b;

    // Solve resultant cubic
    double r0, r1, r2;
    int n = solve_deg3(1, -c, d * b - 4 * e, 4 * c * e - d * d - b2 * e, r0, r1, r2);
    if (n == 0) return 0;

    // Calculate R^2
    double R2 = 0.25 * b2 - c + r0, R;
    if (R2 < 0)
        return 0;

    R = sqrt(R2);
    double inv_R = 1. / R;

    int nb_real_roots = 0;

    // Calculate D^2 and E^2
    double D2, E2;
    if (R < 10E-12) {
        double temp = r0 * r0 - 4 * e;
        if (temp < 0)
            D2 = E2 = -1;
        else {
            double sqrt_temp = sqrt(temp);
            D2 = 0.75 * b2 - 2 * c + 2 * sqrt_temp;
            E2 = D2 - 4 * sqrt_temp;
        }
    }
    else {
        double u = 0.75 * b2 - 2 * c - R2,
            v = 0.25 * inv_R * (4 * bc - 8 * d - b3);
        D2 = u + v;
        E2 = u - v;
    }

    double b_4 = 0.25 * b, R_2 = 0.5 * R;
    if (D2 >= 0) {
        double D = sqrt(D2);
        nb_real_roots = 2;
        double D_2 = 0.5 * D;
        x0 = R_2 + D_2 - b_4;
        x1 = x0 - D;
    }

    // Calculate E^2
    if (E2 >= 0) {
        double E = sqrt(E2);
        double E_2 = 0.5 * E;
        if (nb_real_roots == 0) {
            x0 = -R_2 + E_2 - b_4;
            x1 = x0 - E;
            nb_real_roots = 2;
        }
        else {
            x2 = -R_2 + E_2 - b_4;
            x3 = x2 - E;
            nb_real_roots = 4;
        }
    }

    return nb_real_roots;
}
