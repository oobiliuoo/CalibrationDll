#pragma once

void blP3P(bl::vP3F points3D, bl::vP2F points2D, const cv::Mat cameraMatrix, const cv::Mat distCoeffs, cv::Mat& R, cv::Mat& T);

int solveForLengths(double lengths[4][3], double distances[3], double cosines[3]);


