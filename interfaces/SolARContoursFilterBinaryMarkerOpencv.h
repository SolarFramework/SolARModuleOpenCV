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

#ifndef SOLARCONTOURSFILTERBINARYMARKEROPENCV_H
#define SOLARCONTOURSFILTERBINARYMARKEROPENCV_H

#include "api/features/IContoursFilter.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARContoursFilterBinaryMarkerOpencv
 * @brief <B>Filters contours to select only the contours of squared binary markers.</B>
 * <TT>UUID: 4309dcc6-cc73-11e7-abc4-cec278b6b50a</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ minContourLength,
 *                          the minimum length of an edge of a contour in pixels after simplification.<br>
 *                            Any simplified contour which will have at least one edge that will
 *                            have a length in pixel less than this value will be removed from the
 *                            input filters,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 20.f }}
 * @SolARComponentProperty{ epsilon,
 *                          the maximum distance between the original curve and its approximation.<br>
 *                            This filter first simplifies the contour if its curve is low.
 *                            The simplified contour will not be more than epsilon pixels away
 *                            from the original contour,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.05f }}
 * @SolARComponentProperty{ minDistanceBetweenContourCorners,
 *                          the minimum number of edges of a contour to extract. If negative value\, extract all contours,
 *                          @SolARComponentPropertyDescNum{ float, [-1..MAX FLOAT], 0.05f }}
 * @SolARComponentPropertiesEnd
 *
 */

class SOLAROPENCV_EXPORT_API SolARContoursFilterBinaryMarkerOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IContoursFilter {
public:
    SolARContoursFilterBinaryMarkerOpencv();
    ~SolARContoursFilterBinaryMarkerOpencv() = default;
    /// @brief Filters an ensemble of contours according to a given parameters (minimum of contour length,  minimum contours corners..).
    /// [in] input_contours: original extracted contours.
    ///[out] output_contours: final filtred contours.
    FrameworkReturnCode filter(const std::vector<datastructure::Contour2Df> & input_contours, std::vector<datastructure::Contour2Df> & output_contours) override;
    void unloadComponent () override final; 

private:    
    /// @brief The maximum distance between the original curve and its approximation.
    /// This filter first simplifies the contour if its curve is low. The simplified contour will not be more than epsilon pixels away from the original contour.
    float m_epsilon = 0.05f;

    /// @brief The minimum length of an edge of a contour in pixels after simplification.
    /// Any simplified contour which will have at least one edge that will have a length in pixel less than this value will be removed from the input filters
    float m_minContourLength = 20.0f;

    /// @brief The minimum average distance in pixels between corners of a contour and the same corners of another contour
    /// If the corners are too close from the same corners of another contour, the contour with the lower perimeter is removed
    float m_minDistanceBetweenContourCorners = 10.0f;
};

}
}
}  // end of namespace Solar

#endif // SOLARCONTOURSFILTERBINARYMARKEROPENCV_H
