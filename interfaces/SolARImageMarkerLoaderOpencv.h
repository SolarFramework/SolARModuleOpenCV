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

#ifndef SOLARIMAGEMARKERLOADEROPENCV_H
#define SOLARIMAGEMARKERLOADEROPENCV_H

#include <api/input/files/ITrackableLoader.h>
#include <xpcf/component/ConfigurableBase.h>
#include <SolAROpencvAPI.h>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageMarkerLoaderOpencv
 * @brief <B>Loads a 2D natural image marker from a file.</B>
 * <TT>UUID: aae41002-8e5b-11eb-8dcd-0242ac130003</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ filePath,
 *                          the path to the file describing the 2D natural image marker,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARImageMarkerLoaderOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::files::ITrackableLoader {
public:
    SolARImageMarkerLoaderOpencv();

    ~SolARImageMarkerLoaderOpencv() override = default;
    void unloadComponent () override final;

    /// @brief Loads a specific trackable object and its features.
    /// @param [in,out] trackable: the loaded trackable loaded
    /// @return FrameworkReturnCode::_SUCCESS if load succeed, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode loadTrackable(SRef<datastructure::Trackable>& trackable) override;

private:
    /// @brief the path to the file describing the 2D natural image marker
    std::string m_filePath ="";
};

}
}
}

#endif
