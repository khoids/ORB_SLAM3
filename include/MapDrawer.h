/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    // void DrawMapPoints(const bool bDrawVL, pangolin::OpenGlMatrix &Twc);
    void DrawMapPoints(const bool bDrawVL, const bool bHideGP, pangolin::OpenGlMatrix Twc);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba, const bool bDrawOptCovGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);
    void DrawXYPlane();
    // void DrawVerticalLine(Eigen::Matrix<float, 3, 1> point, Eigen::Matrix<float, 3, 1> gPoint);
    // void SetColor(Eigen::Matrix<float,3,1> point,pangolin::OpenGlMatrix &Twc,const char zeroPlane, const float maxDistanceThreshold);
    void SetColorByDistance(Eigen::Vector3f gPoint, Eigen::Vector3f gCPoint, const float thDistance, Eigen::Vector3f colorNear, Eigen::Vector3f colorFar);
    // void getGroundProjectPoint(Eigen::Matrix<float, 3, 1> &gPoint, float &pHeight, Eigen::Matrix<float, 3, 1> point, Eigen::Matrix<float, 3, 1> cPoint, const float cHeight, const char zeroPlane);
    void getGroundProjectPoint(Eigen::Vector3f &gPoint, float &pHeight, Eigen::Vector3f point, Eigen::Vector3f gCPoint, Eigen::Vector3f upVec);
    // bool isGroundPoint(Eigen::Matrix<float,3,1> gPoint, const float pHeight, const float thHeight);

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
