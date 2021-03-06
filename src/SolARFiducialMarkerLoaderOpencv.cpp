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

#include "SolARFiducialMarkerLoaderOpencv.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARFiducialMarkerLoaderOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARFiducialMarkerLoaderOpencv::SolARFiducialMarkerLoaderOpencv():ConfigurableBase(xpcf::toUUID<SolARFiducialMarkerLoaderOpencv>())
    {
        LOG_DEBUG("SolARFiducialMarkerLoaderOpencv constructor")
        declareInterface<api::input::files::ITrackableLoader>(this);
        declareProperty("filePath", m_filePath);
    }

    SolARFiducialMarkerLoaderOpencv::~SolARFiducialMarkerLoaderOpencv()
    {
        LOG_DEBUG(" SolARFiducialMarkerLoaderOpencv destructor")
    }

    SRef<Trackable> SolARFiducialMarkerLoaderOpencv::loadTrackable()
    {
        SRef<FiducialMarker> fiducialMarker = 0; // Fiducial Marker instance returned
        SquaredBinaryPattern binaryPattern; // Binary pattern of the fiducial marker
        Sizef markerSize; // Size of the fiducial image
        cv::Mat cv_pattern;

        LOG_DEBUG("SolARFiducialMarkerLoaderOpencv::loadTrackable");

        if (m_filePath.empty())
        {
            LOG_ERROR("Fiducial marker definition file path has not been defined");
            return fiducialMarker;
        }

        LOG_DEBUG("Load fiducial configuration file: {}", m_filePath);

        cv::FileStorage fs(m_filePath, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            LOG_ERROR("Binary Fiducial Marker file {} cannot be loaded", m_filePath)
            return fiducialMarker;
        }

        LOG_DEBUG("Extract data from configuration file");

        fs["MarkerWidth"] >> markerSize.width;
        fs["MarkerHeight"] >> markerSize.height;
        fs["Pattern"] >> cv_pattern;
        fs.release();

        int nbRows = cv_pattern.rows;
        int nbCols = cv_pattern.cols;

        if (nbRows== 0 || nbCols ==0)
        {
            LOG_ERROR("In Binary Fiducial Marker file {}, the pattern matrix is empty", m_filePath)
            return fiducialMarker;
        }

        // Reconstruct the binary pattern
        SquaredBinaryPatternMatrix sfpm;
        sfpm.resize(nbRows, nbCols);

        for (int i = 0; i < nbRows; i++)
            for (int j = 0; j < nbCols; j++)
                 sfpm(i,j) =(bool)cv_pattern.at<bool>(i,j);

        // Set the binary pattern
        binaryPattern.setPatternMatrix(sfpm);

        // Create the new FiducialMarker object (kinf of Trackable object)
        fiducialMarker = SRef<FiducialMarker>(new FiducialMarker(m_filePath, markerSize, binaryPattern));

        LOG_DEBUG("Fiducial marker url / width / height / pattern size = {} / {} / {} / {}",
                  fiducialMarker->getURL(), fiducialMarker->getWidth(), fiducialMarker->getHeight(), fiducialMarker->getPattern().getSize());

        return fiducialMarker;
    }

}
}
}
