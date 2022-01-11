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

#ifndef SOLARQRCODELOADEROPENCV_H
#define SOLARQRCODELOADEROPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/input/files/ITrackableLoader.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARQRCodeLoaderOpencv
 * @brief <B>Load a QR code from file.</B>
 * <TT>UUID: 435242ab-e2fe-4477-9ec0-44af2dfa2386</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ filePath,
 *                          the path to the file describing the QR code,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARQRCodeLoaderOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::files::ITrackableLoader {

    public:

        SolARQRCodeLoaderOpencv();
        ~SolARQRCodeLoaderOpencv() override;

        void unloadComponent () override;

        /// @brief Loads a specific trackable object and its features.
        /// @param [in,out] trackable: the loaded trackable loaded
        /// @return FrameworkReturnCode::_SUCCESS if load succeed, else FrameworkReturnCode::_ERROR_
        virtual FrameworkReturnCode loadTrackable(SRef<datastructure::Trackable>& trackable) override;

     private:
        /// @brief the path to the file describing the QR code
        std::string m_filePath = "";
};

}
}
}  // end of namespace Solar

#endif // SOLARQRCODELOADEROPENCV_H
