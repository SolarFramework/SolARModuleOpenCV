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

#ifndef SolARSIDEBYSIDEOVERLAYOPENCV_H
#define SolARSIDEBYSIDEOVERLAYOPENCV_H
#include <vector>

#include "opencv2/core.hpp"

#include "api/display/ISideBySideOverlay.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARSideBySideOverlayOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::ISideBySideOverlay
{
public:
    SolARSideBySideOverlayOpencv();

    void drawMatchesLines(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <SRef<Point2Df>> & points_image1, const std::vector <SRef<Point2Df>> & points_image2) override;

    void unloadComponent () override final;

private:
    /// @brief The color of the linse displaying the matches between the two images
    std::vector<unsigned int> m_color = {0,255,0};

    /// @brief if not null, the color will be randomized for each line.
    unsigned int m_randomColor = 0;

    /// @brief the thickness of the lines displaying the matches between the two images
    unsigned int m_thickness = 1;

    /// @brief the maximum number of matches to display. If negative, all matches are displayed
    int m_maxMatches = -1;
};

}
}
}

#endif // SolARSIDEBYSIDEOVERLAYOPENCV_H
