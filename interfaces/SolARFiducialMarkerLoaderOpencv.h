/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
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

#ifndef SOLARFIDUCIALMARKERLOADEROPENCV_H
#define SOLARFIDUCIALMARKERLOADEROPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

#include "api/input/files/ITrackableLoader.h"
#include "datastructure/Trackable.h"
#include "datastructure/FiducialMarker.h"

#include "opencv2/opencv.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARFiducialMarkerLoaderOpencv
 * @brief <B>Loads a fiducial marker from a description file.</B>
 * <TT>UUID: d0116ed2-45d7-455d-8011-57959da1b0fa</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARFiducialMarkerLoaderOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::files::ITrackableLoader {

    public:

        SolARFiducialMarkerLoaderOpencv();
        ~SolARFiducialMarkerLoaderOpencv() override;

        void unloadComponent () override;

        /// @brief Loads a specific trackable object and its features.
        /// @return SRef<Trackable> : the trackable object created from the description file
        /// or 0 if an error occurs
        virtual SRef<Trackable> loadTrackable() override;

     private:
        /// @brief the path to the file describing the fiducial marker
        std::string m_filePath = "";
};

}
}
}  // end of namespace Solar

#endif // SOLARFIDUCIALMARKERLOADEROPENCV_H
