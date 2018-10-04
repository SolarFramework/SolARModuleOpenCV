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

#include "SolARPerspectiveControllerOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/opencv.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARPerspectiveControllerOpencv::SolARPerspectiveControllerOpencv():ConfigurableBase(xpcf::toUUID<SolARPerspectiveControllerOpencv>())
    {
        addInterface<api::image::IPerspectiveController>(this);
        SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
        params->wrapInteger("outputImageWidth", m_outputImageWidth);
        params->wrapInteger("outputImageHeight", m_outputImageHeight);
    }

    FrameworkReturnCode SolARPerspectiveControllerOpencv::correct(const SRef<Image> inputImg, SRef<Contour2Df> & contour, SRef<Image> & outputImage)
    {
        std::vector<cv::Point2f> points;
        cv::Size patches_size(m_outputImageWidth, m_outputImageHeight);
        std::vector<cv::Point2f> markerCorners2D;
        markerCorners2D.push_back(cv::Point2f(0, 0));
        markerCorners2D.push_back(cv::Point2f(m_outputImageWidth - 1, 0));
        markerCorners2D.push_back(cv::Point2f(m_outputImageWidth - 1, m_outputImageHeight - 1));
        markerCorners2D.push_back(cv::Point2f(0, m_outputImageHeight - 1));

        cv::Mat cv_inputImg = SolAROpenCVHelper::mapToOpenCV(inputImg);
          // For each contour, extract the patch
        if (contour->size()>=4)

        {
            for(unsigned int j =0; j < 4; ++j)
            {
               points.push_back(cv::Point2f((*contour)[j][0],(*contour)[j][1]));
            }
                // Find the perspective transformation that brings current marker to rectangular form
            cv::Mat markerTransform = cv::getPerspectiveTransform(points, markerCorners2D);
                // Transform image to get a canonical marker image
            cv::Mat cv_patch;
            cv::warpPerspective(cv_inputImg, cv_patch, markerTransform, patches_size);
            SolAROpenCVHelper::convertToSolar(cv_patch, outputImage);

            return FrameworkReturnCode::_SUCCESS;
        }
        else
        {
            outputImage = nullptr;
            return FrameworkReturnCode::_ERROR_;
        }
    }

    FrameworkReturnCode SolARPerspectiveControllerOpencv::correct(const SRef<Image> inputImg, std::vector<SRef<Contour2Df>> & contours, std::vector<SRef<Image>> & patches)
    {
        if (inputImg == nullptr)
        {
            LOG_ERROR("The input image for PerspectiveControllerOpenCV is null");
            return FrameworkReturnCode::_ERROR_;
        }
        if (m_outputImageWidth <=0 || m_outputImageHeight <=0)
        {
            LOG_ERROR("The width or height of the output image for PerspectiveControllerOpenCV is null or negative");
            return FrameworkReturnCode::_ERROR_;
        }
        std::vector<cv::Point2f> points;
        cv::Size patches_size(m_outputImageWidth, m_outputImageHeight);
        std::vector<cv::Point2f> markerCorners2D;
        markerCorners2D.push_back(cv::Point2f(0, 0));
        markerCorners2D.push_back(cv::Point2f(m_outputImageWidth - 1, 0));
        markerCorners2D.push_back(cv::Point2f(m_outputImageWidth - 1, m_outputImageHeight - 1));
        markerCorners2D.push_back(cv::Point2f(0, m_outputImageHeight - 1));

        cv::Mat cv_inputImg = SolAROpenCVHelper::mapToOpenCV(inputImg);
        patches.clear();
          // For each contour, extract the patch
        for (size_t i = 0; i<contours.size(); i++)
        {
            points.clear();
            // If the contour contains at least 4 points
            if (contours[i]->size() >= 4)
            {
                for(unsigned int j =0; j < 4; ++j)
                {
                   points.push_back(cv::Point2f((*(contours[i]))[j][0],(*(contours[i]))[j][1]));
                }
                    // Find the perspective transformation that brings current marker to rectangular form
                cv::Mat markerTransform = cv::getPerspectiveTransform(points, markerCorners2D);
                    // Transform image to get a canonical marker image
                cv::Mat cv_patch;
                cv::warpPerspective(cv_inputImg, cv_patch, markerTransform, patches_size);
                SRef<Image> patch;
                SolAROpenCVHelper::convertToSolar(cv_patch, patch);
                patches.push_back(patch);
            }
            // else add a empty image to ensure the order of contours and output corrected images.
            else
                patches.push_back(nullptr);
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
