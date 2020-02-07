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

#include "SolAR3DOverlayBoxOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR3DOverlayBoxOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolAR3DOverlayBoxOpencv::SolAR3DOverlayBoxOpencv():ConfigurableBase(xpcf::toUUID<SolAR3DOverlayBoxOpencv>())
{
    declareInterface<api::display::I3DOverlay>(this);

    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);
    m_parallelepiped.create(8, 3, CV_32FC1);

    declarePropertySequence("orientation", m_orientation);
    declarePropertySequence("position", m_position);
    declarePropertySequence("size", m_size);

   LOG_DEBUG(" SolAR3DOverlayBoxOpencv constructor");

}

xpcf::XPCFErrorCode SolAR3DOverlayBoxOpencv::onConfigured()
{
    LOG_DEBUG(" SolAR3DOverlayBoxOpencv onConfigured");
    float half_X=m_size[0]*0.5f;
    float half_Y=m_size[1]*0.5f;
    float Z=m_size[2];

    RotationMatrixf rotation;
    rotation = Maths::AngleAxisf(m_orientation[0] * SOLAR_DEG2RAD, Vector3f::UnitX())
                             * Maths::AngleAxisf(m_orientation[1] * SOLAR_DEG2RAD, Vector3f::UnitY())
                             * Maths::AngleAxisf(m_orientation[2] * SOLAR_DEG2RAD, Vector3f::UnitZ());
    Vector3f translation;
    translation(0) = m_position[0];
    translation(1) = m_position[1];
    translation(2) = m_position[2];


    Transform3Df transform;
    transform.setIdentity();
    transform.translate(translation);
    transform.rotate(rotation);

    std::vector<Vector4f> parallelepiped;

    parallelepiped.push_back(transform * Vector4f(-half_X, -half_Y, 0.0f, 1.0f));
    parallelepiped.push_back(transform * Vector4f(half_X, -half_Y, 0.0f, 1.0f));
    parallelepiped.push_back(transform * Vector4f(half_X, half_Y, 0.0f, 1.0f));
    parallelepiped.push_back(transform * Vector4f(-half_X, half_Y, 0.0f, 1.0f));
    parallelepiped.push_back(transform * Vector4f(-half_X, -half_Y, -Z, 1.0f));
    parallelepiped.push_back(transform * Vector4f(half_X, -half_Y, -Z, 1.0f));
    parallelepiped.push_back(transform * Vector4f(half_X, half_Y, -Z, 1.0f));
    parallelepiped.push_back(transform * Vector4f(-half_X, half_Y, -Z, 1.0f));

    for (int i = 0; i < parallelepiped.size(); i++)
         for (int j = 0; j < 3; j++)
            m_parallelepiped.at< float >(i, j) = parallelepiped[i](j);

    return xpcf::_SUCCESS;
}

void SolAR3DOverlayBoxOpencv::draw (const Transform3Df & pose, SRef<Image> displayImage)
{

    Transform3Df poseInverse =pose.inverse();

    // image where parallelepiped will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    // where to store image points of parallelpiped with pose applied
    std::vector<cv::Point2f > imagePoints;

    // Rotation and Translation from input pose
    cv::Mat Rvec;   Rvec.create(3, 3, CV_32FC1);
    cv::Mat Tvec;   Tvec.create(3, 1, CV_32FC1);


    Rvec.at<float>(0,0) = poseInverse(0,0);
    Rvec.at<float>(0,1) = poseInverse(0,1);
    Rvec.at<float>(0,2) = poseInverse(0,2);

    Rvec.at<float>(1,0) = poseInverse(1,0);
    Rvec.at<float>(1,1) = poseInverse(1,1);
    Rvec.at<float>(1,2) = poseInverse(1,2);

    Rvec.at<float>(2,0) = poseInverse(2,0);
    Rvec.at<float>(2,1) = poseInverse(2,1);
    Rvec.at<float>(2,2) = poseInverse(2,2);

    Tvec.at<float>(0,0) = poseInverse(0,3);
    Tvec.at<float>(1,0) = poseInverse(1,3);
    Tvec.at<float>(2,0) = poseInverse(2,3);

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

void SolAR3DOverlayBoxOpencv::setCameraParameters(const CamCalibration & intrinsic_param, const CamDistortion & distorsion_param){

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

}
}
}
