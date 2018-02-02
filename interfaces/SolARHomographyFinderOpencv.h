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

#ifndef SolARHomographyFinderOpencv_H
#define SolARHomographyFinderOpencv_H
#include <vector>

#include "api/devices/input/ISolARCamera.h"
#include "ISolARPoseHomography.h"

#include "ComponentBase.h"

#include "opencv2/core.hpp"

#include "datastructure/SolARPose.h"
#include "datastructure/SolARKeypoint.h"

namespace SolAR {

class SolARHomographyFinderOpencv : public org::bcom::xpcf::ComponentBase,
    public ISolARPoseHomography
{
public:
    SolARHomographyFinderOpencv();


    SolARPose findPose (std::vector<SolAR::DescriptorMatcher::Match>& matches,
                                      const std::vector< sptrnms::shared_ptr<SolAR::SolARKeypoint> >& keypoints) override;

    int findHomography(std::vector<SolAR::DescriptorMatcher::Match>& matches,
                  const std::vector< sptrnms::shared_ptr<SolAR::SolARKeypoint> >& keypoints1,
                  const std::vector< sptrnms::shared_ptr<SolAR::SolARKeypoint> >& keypoints2,
                  SolAR2DTransformf & homography) override;


    int computeRefinedPose( const SolAR2DTransformf & homography, SolARPose & pose) override;

    int computePose( const SolAR2DTransformf & homography, sptrnms::shared_ptr<ISolARCamera>& cam, SolARPoseMatrix & pose) override;
    void overlay(const SolAR2DTransformf & homography,const SRef<SolAR::SolARImage> refImage, SRef<SolAR::SolARImage> displayImage) override;
    void getGoodMatches(std::vector<SolAR::DescriptorMatcher::Match>& good_matches);

    void overlay3DProjectedPoint( SolAR::SolARPose & pose, sptrnms::shared_ptr<ISolARCamera>& cam , const SRef<SolAR::SolARImage> refImage, SRef<SolAR::SolARImage> displayImage) override;

    void setRefDimensions(int refWidth,int refHeight);
    void setCameraParameters(SolARCamCalibration intrinsic_parameters, SolARCamDistortion distorsion_parameters);
    void setReferenceKeypoints(const std::vector< sptrnms::shared_ptr<SolAR::SolARKeypoint> >& refKeypoints);

    void unloadComponent () override final;

    XPCF_DECLARE_UUID("A8C27A2A-9E31-4B8D-8F24-46A4F47CB930");

private:

    bool isHValid(const SolAR2DTransformf& H);

    void estimateCubeDrawing(const unsigned int width, const unsigned int height);

    void initObjCorner(const unsigned int width, const unsigned int height );
    bool validPoint(const cv::Point2f & point, const unsigned int width, const unsigned int height );


    std::vector<cv::DMatch> good_matches;

    std::vector<cv::Point2f> obj_corners;
    std::vector<cv::Point2f> scene_corners;


    std::vector< sptrnms::shared_ptr<SolAR::SolARKeypoint> > m_refKeypoints;


    int refWidth;
    int refHeight;

    cv::Mat m_camMatrix;
    cv::Mat m_camDistortion;

    cv::Mat m_cube;//to display projected point

};

}

#endif
