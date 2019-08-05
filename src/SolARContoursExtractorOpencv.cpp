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
#include "core/Log.h"

#include "opencv2/opencv.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARContoursExtractorOpencv::SolARContoursExtractorOpencv():ConfigurableBase(xpcf::toUUID<SolARContoursExtractorOpencv>())
    {
        declareInterface<api::features::IContoursExtractor>(this);
        declareProperty("minContourEdges",m_minContourEdges);
    }

    FrameworkReturnCode SolARContoursExtractorOpencv::extract(const SRef<Image> inputImg, std::vector<Contour2Df> & contours)
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
            for (auto & ocv_contour : ocv_contours)
            {
                size_t contourSize = ocv_contour.size();
                if (contourSize > m_minContourEdges)
                {
                    Contour2Df contour;
                    contour.reserve(contourSize);
                    for (size_t j = 0; j < contourSize; j++)
                    {
                        contour.emplace_back(ocv_contour[j].x, ocv_contour[j].y);
                    }
                    contours.emplace_back(contour);
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
