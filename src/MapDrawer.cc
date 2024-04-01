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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#define HAVE_EIGEN
namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}


// void MapDrawer::getGroundProjectPoint(Eigen::Vector3f &gPoint, float &pHeight, Eigen::Vector3f point, Eigen::Vector3f cPoint, const float cHeight, const char zeroPlane)
void MapDrawer::getGroundProjectPoint(Eigen::Vector3f &gPoint, float &pHeight, Eigen::Vector3f point, Eigen::Vector3f gCPoint, Eigen::Vector3f upVec)
{
    // Perpendicular distance from interest point to ground
    float dist = (point(0)-gCPoint(0))*upVec(0) + (point(1)-gCPoint(1))*upVec(1) + (point(2)-gCPoint(2))*upVec(2);
    pHeight = abs(dist);
    // Shift interest point a distance pHeight along up-vector of camera 
    gPoint = point - upVec*dist;
}

void MapDrawer::SetColorByDistance(Eigen::Vector3f gPoint, Eigen::Vector3f gCPoint, const float thDistance, Eigen::Vector3f colorNear, Eigen::Vector3f colorFar)
{
    // Calculate distant from ground-point to camera
    float distance2 = (pow(gCPoint(0) - gPoint(0), 2) + pow(gCPoint(1) - gPoint(1), 2) + pow(gCPoint(2) - gPoint(2), 2)) * 1.0;
    float percent = distance2 / pow(thDistance, 2);
    // Mix color
    Eigen::Vector3f colorMix = (1.0 - percent) * colorNear + percent * colorFar;
    glColor3f(colorMix[0], colorMix[1], colorMix[2]);
}

void MapDrawer::DrawMapPoints(const bool bDrawVL, const bool bHideGP, Eigen::Matrix4f Twc)
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return;

    const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    // Get current camera pos
    unique_lock<mutex> lock(mMutexCamera);
    Eigen::Matrix4f T_wc = mCameraPose.matrix();
    Eigen::Vector3f camPos( T_wc(12), T_wc(13), T_wc(14));

    const float thDis = 1.0;                // 10m from camera - for gradient color
    const float scale = 1;
    const float thHeight  = 0.025*scale;    // classification point at ground
    const float camHeight = 0.095*scale;    // GPS/IMU height (0.93m KITTI)
    // const float camHeight = 0.093;

    // Get left and forward vector from world-from-camera transformation matrix 
    Eigen::Vector3f lVec( T_wc(0), T_wc(1), T_wc(2));
    Eigen::Vector3f fVec( T_wc(8), T_wc(9), T_wc(10));
    Eigen::Vector3f leftpt = camPos + 0.5*lVec;
    Eigen::Vector3f forwardpt = camPos + 0.5*fVec;

    // Somehow up-vector get from Tcw is wrong. So we'll calculate up-vector from left and forward vector instead
    Eigen::Vector3f uVec = lVec.cross(fVec).normalized();
    Eigen::Vector3f gCamPos = camPos - camHeight*uVec;

    // For debug
    glColor3f(0,0,0.5);
    pangolin::glDrawCross(gCamPos(0), gCamPos(1), gCamPos(2), 0.05);
    glColor3f(1,0,0);
    pangolin::glDrawLine(camPos(0), camPos(1), camPos(2), gCamPos(0), gCamPos(1), gCamPos(2));
    pangolin::glDrawLine(camPos(0), camPos(1), camPos(2), leftpt(0), leftpt(1), leftpt(2));
    pangolin::glDrawLine(camPos(0), camPos(1), camPos(2), forwardpt(0), forwardpt(1), forwardpt(2));
    

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
        if (bHideGP)
        {
            Eigen::Vector3f gpPos;
            float gpHeight;
            getGroundProjectPoint(gpPos, gpHeight, pos, gCamPos, uVec);
            if (gpHeight <= thHeight)
                continue;
            glVertex3f(pos(0), pos(1), pos(2));
        }
        else
            glVertex3f(pos(0), pos(1), pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    // glColor3f(1.0,0.0,0.0);
    // std::cout << "==================== " <<endl;
    for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
    {
        if ((*sit)->isBad())
            continue;
        Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
        Eigen::Vector3f gpPos;
        float gpHeight;
        getGroundProjectPoint(gpPos, gpHeight, pos, gCamPos, uVec);

        // if (sit == spRefMPs.begin())
        // {
        //     std::cout << "point  " << pos(1) << endl;
        //     std::cout << "G_pt   " << gpPos(1) << endl;
        //     std::cout << "height " << gpHeight << endl;

        //     std::cout << "cam_pt " << camPos(1) << endl;
        //     std::cout << "Gc_pt  " << gCamPos(1) << endl;
        //     std::cout << "cH     " << camHeight << endl << endl;
        // }
        


        if (gpHeight <= thHeight) // point at ground
        {
            if (bHideGP)
                continue;
            else
            {
                glColor3f(0.0, 1.0, 0.0);
                glVertex3f(pos(0), pos(1), pos(2));
            }
        }
        else
        {
            Eigen::Vector3f colorNear(1.0, 0.0, 0.0); // Red for near-point
            Eigen::Vector3f colorFar(1.0, 0.0, 1.0);  // Purple for far-point
            SetColorByDistance(gpPos, gCamPos, 2.0, colorNear, colorFar);
            glVertex3f(pos(0), pos(1), pos(2));
        }
    }
    glEnd();

    glBegin(GL_LINES);
    if (bDrawVL)
    {
        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
        {
            if ((*sit)->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
            Eigen::Vector3f gpPos;
            float gpHeight;
            getGroundProjectPoint(gpPos, gpHeight, pos, gCamPos, uVec);
            if (gpHeight <= thHeight) // point at ground
            {
                if (bHideGP)
                    continue;
                else
                {
                    glColor3f(0.0, 1.0, 0.0);
                    glVertex3f(pos(0), pos(1), pos(2));
                    glVertex3f(gpPos(0), gpPos(1), gpPos(2));
                }
            }
            else
            {
                Eigen::Vector3f colorNear(1.0, 0.0, 0.0); // Red for near-point
                Eigen::Vector3f colorFar(1.0, 0.0, 1.0);  // Purple for far-point
                SetColorByDistance(gpPos, gCamPos, thDis, colorNear, colorFar);
                glVertex3f(pos(0), pos(1), pos(2));
                glVertex3f(gpPos(0), gpPos(1), gpPos(2));
            }
        }
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba, const bool bDrawOptCovGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty() && bDrawOptCovGraph)
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);

    // glVertex3f(0,0,0);
    // glVertex3f(0,0,0.5);
    // glVertex3f(0,0,0);
    // glVertex3f(0,0.5,0);
    // glVertex3f(0,0,0);
    // glVertex3f(0.5,0,0);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}

void MapDrawer::DrawXYPlane()
{
    glColor3f(0.5f, 0.5f, 0.5f);
    //pangolin::glDraw_z0(0.1f,100);
    pangolin::glDraw_y0(0.1f,100);
    return;
}
}//namespace ORB_SLAM