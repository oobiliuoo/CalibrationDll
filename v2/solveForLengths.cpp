#include "pch.h"
#include "BLCalibration.h"
#include "utils.h"


int solveForLengths(double lengths[4][3], double distances[3], double cosines[3])
{
    //吴消元法，数据准备
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

    //A, B, C, D, E 为四次多项式的系数；
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

    //求解四次多项式；
    //int n = solve_deg4(A, B, C, D, E, real_roots[0], real_roots[1], real_roots[2], real_roots[3]);
    cv::Mat coef = (cv::Mat_<double>(5, 1) << E,D,C,B,A);
    cv::Mat roots;
    double n =solvePoly(coef, roots);

    double real_roots[4];
    for (int i = 0; i < 4; i++)
    {
        real_roots[i] = roots.at<double>(i,0);
        //-----------------
        std::cout <<"real_roots: " << real_roots[i] << " \n";
    }


    if (n == 0)
        return 0;

    int nb_solutions = 0;
    double r3 = r2 * r, pr2 = p * r2, r3q = r3 * q;
    double inv_b0 = 1. / b0;

    // For each solution of x
    for (int i = 0; i < roots.rows; i++) {
        double x = real_roots[i];

        // Check reality condition
        if (x <= 0)
            continue;

        double x2 = x * x;
        //对应附录中的b1
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
        
        //-----------------
        std::cout << nb_solutions << " -> x:" << X << " " << Y << " " << Z<<std::endl;

        nb_solutions++;
    }

    return nb_solutions;
}