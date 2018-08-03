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

#include "SolARContoursExtractorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#include "xpcf/component/ComponentBase.h"
#include "xpcf/properties/IPropertyMap.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARContoursExtractorOpencv::SolARContoursExtractorOpencv():ConfigurableBase(xpcf::toUUID<SolARContoursExtractorOpencv>())
    {
        addInterface<api::features::IContoursExtractor>(this);
        SRef<xpcf::IPropertyMap> properties;
        properties = xpcf::getPropertyMapInstance();
        // properties = getPropertyRootNode();
        properties->wrapInteger("minContourEdges",m_minContourEdges);
    }

    FrameworkReturnCode SolARContoursExtractorOpencv::extract(const SRef<Image> inputImg, std::vector<SRef<Contour2Df>> & contours)
    {
        if (inputImg->getImageLayout() != Image::LAYOUT_GREY)
        {
            LOG_ERROR("SolARContoursExtractorOpencv::extract takes only binary image as input")
            return FrameworkReturnCode::_ERROR_;
        }

        cv::Mat thresholdImg;
        SolAROpenCVHelper::mapToOpenCV(inputImg, thresholdImg);
        if(!thresholdImg.empty())
        {
            std::vector<std::vector<cv::Point>> ocv_contours;
            cv::findContours(thresholdImg, ocv_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
            contours.clear();
            for (size_t i = 0; i<ocv_contours.size(); i++)
            {
                size_t contourSize = ocv_contours[i].size();
                if (contourSize > m_minContourEdges)
                {
                    SRef<Contour2Df> contour = xpcf::utils::make_shared<Contour2Df>();
                    for (size_t j = 0; j < contourSize; j++)
                    {
                        Point2Df point;
                        point[0] = ocv_contours[i][j].x;
                        point[1] = ocv_contours[i][j].y;
                        contour->push_back(point);
                    }
                    contours.push_back(contour);
                }
            }
        }
        else
        {
            LOG_ERROR("SolARContoursExtractorOpencv::extract get an empty image as input")
            return FrameworkReturnCode::_ERROR_;
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
