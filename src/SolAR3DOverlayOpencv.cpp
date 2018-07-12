/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolAR3DOverlayOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ComponentFactory.h"


#include <map>
#include <random>

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR3DOverlayOpencv)

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
    namespace OPENCV {

    SolAR3DOverlayOpencv::SolAR3DOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolAR3DOverlayOpencv>())
    {
        addInterface<api::display::I3DOverlay>(this);

        m_camMatrix.create(3, 3, CV_32FC1);
        m_camDistorsion.create(5, 1, CV_32FC1);
    m_parallelepiped.create(8, 3, CV_32FC1);

    m_cameraDistorsion.resize(5);
    m_cameraMatrix.resize(9);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloatVector("cameraDistorsion",m_cameraDistorsion);
    params->wrapFloatVector("cameraMatrix",m_cameraMatrix);


   LOG_DEBUG(" SolAR3DOverlayOpencv constructor");

}

void SolAR3DOverlayOpencv::drawBox (Transform3Df & pose, const float X_world, const float Y_world, const float Z_world, const Transform3Df affineTransform, SRef<Image> displayImage)
{

    // image where parallelepiped will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    // set position of 8 corners of parallelepiped
    setParallelepipedPosition(X_world,Y_world,Z_world);

    moveParalleliped(affineTransform); // apply move to the parallelepiped

    // where to store image points of parallelpiped with pose applied
    std::vector<cv::Point2f > imagePoints;

    // Rotation and Translation from input pose
    cv::Mat Rvec;   Rvec.create(3, 3, CV_32FC1);
    cv::Mat Tvec;   Tvec.create(3, 1, CV_32FC1);


    Rvec.at<float>(0,0) = pose(0,0);
    Rvec.at<float>(0,1) = pose(0,1);
    Rvec.at<float>(0,2) = pose(0,2);

    Rvec.at<float>(1,0) = pose(1,0);
    Rvec.at<float>(1,1) = pose(1,1);
    Rvec.at<float>(1,2) = pose(1,2);

    Rvec.at<float>(2,0) = pose(2,0);
    Rvec.at<float>(2,1) = pose(2,1);
    Rvec.at<float>(2,2) = pose(2,2);

    Tvec.at<float>(0,0) = pose(0,3);
    Tvec.at<float>(1,0) = pose(1,3);
    Tvec.at<float>(2,0) = pose(2,3);

    cv::Mat rodrig;
    cv::Rodrigues(Rvec,rodrig);

    //compute the projection of the points of the cube
    cv::projectPoints(m_parallelepiped, rodrig, Tvec, m_camMatrix, m_camDistorsion, imagePoints);

   // draw parallelepiped
    // circle around corners
    for(auto & element : imagePoints ){
        if (element.x >=0 && element.x < displayedImage.cols && element.y >= 0 && element.y < displayedImage.rows)
            circle(displayedImage, element, 8, cv::Scalar(128, 0, 128), -1);
   }

    // finally draw cube
    for (int i = 0; i < 4; i++)
    {
        SolAROpenCVHelper::drawCVLine(displayedImage, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0,0,255), 4);
        SolAROpenCVHelper::drawCVLine(displayedImage, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], cv::Scalar(0,255,0), 4);
        SolAROpenCVHelper::drawCVLine(displayedImage, imagePoints[i], imagePoints[i + 4], cv::Scalar(255,0,0), 4);
    }
}

void SolAR3DOverlayOpencv::setParallelepipedPosition(const float X_world, const float Y_world, const float Z_world)
{

    float half_X=X_world*0.5f;
    float half_Y=Y_world*0.5f;
    float Z=Z_world;

    // 4 corners at z=0
    // 1st corner
    m_parallelepiped.at< float >(0, 0) = -half_X;
    m_parallelepiped.at< float >(0, 1) = -half_Y;
    m_parallelepiped.at< float >(0, 2) = 0;

    // 2nd corner
    m_parallelepiped.at< float >(1, 0) = half_X;
    m_parallelepiped.at< float >(1, 1) = -half_Y;
    m_parallelepiped.at< float >(1, 2) = 0;

    // 3rd corner
    m_parallelepiped.at< float >(2, 0) = half_X;
    m_parallelepiped.at< float >(2, 1) = half_Y;
    m_parallelepiped.at< float >(2, 2) = 0;

    // 4th corner
    m_parallelepiped.at< float >(3, 0) = -half_X;
    m_parallelepiped.at< float >(3, 1) = half_Y;
    m_parallelepiped.at< float >(3, 2) = 0;


    // 4 parallel corners to corners above, at z=-Z
    // 5th corner
    m_parallelepiped.at< float >(4, 0) = -half_X;
    m_parallelepiped.at< float >(4, 1) = -half_Y;
    m_parallelepiped.at< float >(4, 2) = -Z;

    // 6th corner
    m_parallelepiped.at< float >(5, 0) = +half_X;
    m_parallelepiped.at< float >(5, 1) = -half_Y;
    m_parallelepiped.at< float >(5, 2) = -Z;

    // 7th corner
    m_parallelepiped.at< float >(6, 0) = half_X;
    m_parallelepiped.at< float >(6, 1) = half_Y;
    m_parallelepiped.at< float >(6, 2) = -Z;

    // 8th corner
    m_parallelepiped.at< float >(7, 0) = -half_X;
    m_parallelepiped.at< float >(7, 1) = half_Y;
    m_parallelepiped.at< float >(7, 2) = -Z;

}

void SolAR3DOverlayOpencv::moveParalleliped(const Transform3Df transformation)
{
    std::vector<SRef<Point3Df>> point3D_vec;
    std::vector<SRef<Point3Df>> output_point3D_vec;

    for (int i = 0;i<8;++i)
    {
        Point3Df point3D(m_parallelepiped.at< float >(i, 0),m_parallelepiped.at< float >(i, 1),m_parallelepiped.at< float >(i, 2));
        point3D_vec.push_back(xpcf::utils::make_shared<Point3Df>(point3D));
    }

    transform3D(point3D_vec,transformation,output_point3D_vec);

    for (int i = 0;i<8;++i)
    {
        Point3Df point3D = *(output_point3D_vec.at(i));
        m_parallelepiped.at< float >(i, 0)=point3D.getX();
        m_parallelepiped.at< float >(i, 1)=point3D.getY();
        m_parallelepiped.at< float >(i, 2)=point3D.getZ();
    }

}

FrameworkReturnCode SolAR3DOverlayOpencv::transform3D(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation, std::vector<SRef<Point3Df>> & outputPoints)
{
    Point3Df outputPoint3D;
    Vector4f outputVector4f;

    for (int i = 0;i<inputPoints.size();++i){

        Point3Df inputPoint3D = *(inputPoints.at(i));
        Vector4f inputVector4f(inputPoint3D.getX(),inputPoint3D.getY(), inputPoint3D.getZ(), 1);
        outputVector4f=transformation*inputVector4f;
        if (outputVector4f[3]!=0) {
            outputPoint3D.setX(outputVector4f[0]/outputVector4f[3]);
            outputPoint3D.setY(outputVector4f[1]/outputVector4f[3]);
            outputPoint3D.setZ(outputVector4f[2]/outputVector4f[3]);
        } else {
            outputPoint3D.setX(0);
            outputPoint3D.setY(0);
            outputPoint3D.setZ(0);
        }
        outputPoints.push_back(xpcf::utils::make_shared<Point3Df>(outputPoint3D));


    }

    return FrameworkReturnCode::_SUCCESS;
}


void SolAR3DOverlayOpencv::setCameraParameters(const CamCalibration & intrinsic_param, const CamDistortion & distorsion_param){

    m_intrinsic_parameters=intrinsic_param;
    m_distorsion_parameters=distorsion_param;

    m_camDistorsion.at<float>(0, 0)  = distorsion_param(0);
    m_camDistorsion.at<float>(1, 0)  = distorsion_param(1);
    m_camDistorsion.at<float>(2, 0)  = distorsion_param(2);
    m_camDistorsion.at<float>(3, 0)  = distorsion_param(3);
    m_camDistorsion.at<float>(4, 0)  = distorsion_param(4);

    m_camMatrix.at<float>(0, 0) = intrinsic_param(0,0);
    m_camMatrix.at<float>(0, 1) = intrinsic_param(0,1);
    m_camMatrix.at<float>(0, 2) = intrinsic_param(0,2);
    m_camMatrix.at<float>(1, 0) = intrinsic_param(1,0);
    m_camMatrix.at<float>(1, 1) = intrinsic_param(1,1);
    m_camMatrix.at<float>(1, 2) = intrinsic_param(1,2);
    m_camMatrix.at<float>(2, 0) = intrinsic_param(2,0);
    m_camMatrix.at<float>(2, 1) = intrinsic_param(2,1);
    m_camMatrix.at<float>(2, 2) = intrinsic_param(2,2);


}



void SolAR3DOverlayOpencv::initCameraParametersFromConfigFile()
{

    m_camDistorsion.at<float>(0, 0)  = m_cameraDistorsion.at(0);
    m_camDistorsion.at<float>(1, 0)  = m_cameraDistorsion.at(1);
    m_camDistorsion.at<float>(2, 0)  = m_cameraDistorsion.at(2);
    m_camDistorsion.at<float>(3, 0)  = m_cameraDistorsion.at(3);
    m_camDistorsion.at<float>(4, 0)  = m_cameraDistorsion.at(4);

    m_camMatrix.at<float>(0, 0) = m_cameraMatrix.at(0);
    m_camMatrix.at<float>(0, 1) = m_cameraMatrix.at(1);
    m_camMatrix.at<float>(0, 2) = m_cameraMatrix.at(2);
    m_camMatrix.at<float>(1, 0) = m_cameraMatrix.at(3);
    m_camMatrix.at<float>(1, 1) = m_cameraMatrix.at(4);
    m_camMatrix.at<float>(1, 2) = m_cameraMatrix.at(5);
    m_camMatrix.at<float>(2, 0) = m_cameraMatrix.at(6);
    m_camMatrix.at<float>(2, 1) = m_cameraMatrix.at(7);
    m_camMatrix.at<float>(2, 2) = m_cameraMatrix.at(8);
}


}
}
}
