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

#include "SolARHomographyEstimationOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "ComponentFactory.h"

#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARHomographyEstimationOpencv::SolARHomographyEstimationOpencv()
{
    setUUID(SolARHomographyEstimationOpencv::UUID);
    addInterface<api::solver::pose::IHomographyEstimation>(this,api::solver::pose::IHomographyEstimation::UUID, "interface api::solver::pose::IHomographyEstimation");
    LOG_DEBUG("SolARHomographyEstimationOpencv constructor")
}

api::solver::pose::HomographyEstimation::RetCode SolARHomographyEstimationOpencv::findHomography(const std::vector< SRef<Point2Df> >& srcPoints,
                                          const std::vector< SRef<Point2Df> >& dstPoints,
                                          Transform2Df & homography)
{

    cv::Mat H;
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    cv::Point2f point;


    for( int i = 0; i < srcPoints.size(); i++ ){
        point.x=srcPoints.at(i)->getX();
        point.y=srcPoints.at(i)->getY();
        obj.push_back( point );

        point.x=dstPoints.at(i)->getX();
        point.y=dstPoints.at(i)->getY();
        scene.push_back( point);
    }

    H = cv::findHomography( obj, scene, CV_RANSAC, 8 );
	if (!H.data) {
		LOG_DEBUG("Homography matrix is empty")
		return api::solver::pose::HomographyEstimation::HOMOGRAPHY_EMPTY;
	}

    H.convertTo(H,CV_32F);

    SolAROpenCVHelper::convertCVMatToSolar(H,homography);

    return api::solver::pose::HomographyEstimation::HOMOGRAPHY_ESTIMATION_OK;
}

}
}
}
