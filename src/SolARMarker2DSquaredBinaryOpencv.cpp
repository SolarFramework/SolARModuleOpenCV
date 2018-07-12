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

#include "SolARMarker2DSquaredBinaryOpencv.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARMarker2DSquaredBinaryOpencv::SolARMarker2DSquaredBinaryOpencv():ComponentBase(xpcf::toUUID<SolARMarker2DSquaredBinaryOpencv>())
    {
        addInterface<api::input::files::IMarker2DSquaredBinary>(this);
        LOG_DEBUG("SolARMarker2DSquaredBinaryOpencv constructor")
        m_size.width = 0;
        m_size.height = 0;
    }

    SolARMarker2DSquaredBinaryOpencv::~SolARMarker2DSquaredBinaryOpencv()
    {
    LOG_DEBUG(" SolARMarker2DSquaredBinaryOpencv")
    }

    FrameworkReturnCode SolARMarker2DSquaredBinaryOpencv::loadMarker(const std::string & filename)
    {
        cv::Mat cv_pattern;

        cv::FileStorage fs(filename, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            LOG_ERROR("Binary Marker file {} cannot be loaded", filename)
            return FrameworkReturnCode::_ERROR_;
        }

        fs["MarkerWidth"] >> m_size.width;
        fs["MarkerHeight"] >> m_size.height;
        fs["Pattern"] >> cv_pattern;
        fs.release();

        int nbRows = cv_pattern.rows;
        int nbCols = cv_pattern.cols;

        if (nbRows== 0 || nbCols ==0)
        {
            LOG_ERROR("In Binary Marker file {}, the pattern matrix is empty", filename)
            return FrameworkReturnCode::_ERROR_;
        }

        SquaredBinaryPatternMatrix sfpm;
        sfpm.resize(nbRows, nbCols);

        for (int i = 0; i < nbRows; i++)
            for (int j = 0; j < nbCols; j++)
                 sfpm(i,j) =(bool)cv_pattern.at<bool>(i,j);

        m_pattern.setPatternMatrix(sfpm);
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}
