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

#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARSideBySideOverlayOpencv : public org::bcom::xpcf::ComponentBase,
    public api::display::ISideBySideOverlay
{
public:
    SolARSideBySideOverlayOpencv();

    void drawMatchesLines(SRef<Image> & image1, SRef<Image> & image2, SRef<Image> & outImage, std::vector <SRef<Point2Df>> & points_image1, std::vector <SRef<Point2Df>> & points_image2, int matches_number = -1) override;

    void unloadComponent () override final;

};

}
}
}

#endif // SolARSIDEBYSIDEOVERLAYOPENCV_H
