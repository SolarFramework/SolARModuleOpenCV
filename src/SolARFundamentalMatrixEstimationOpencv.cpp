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

#include "SolARFundamentalMatrixEstimationOpencv.h"
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

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARFundamentalMatrixEstimationOpencv::SolARFundamentalMatrixEstimationOpencv()
{
    setUUID(SolARFundamentalMatrixEstimationOpencv::UUID);
    addInterface<api::solver::pose::IFundamentalMatrixEstimation>(this,api::solver::pose::IFundamentalMatrixEstimation::UUID, "interface api::solver::pose::IFundamentalMatrixEstimation");
    LOG_DEBUG("SolARFundamentalMatrixEstimationOpencv constructor")
}

api::solver::pose::FundamentalMatrixEstimation::RetCode SolARFundamentalMatrixEstimationOpencv::findFundamental(const std::vector< SRef<Point2Df> >& srcPoints,
                                          const std::vector< SRef<Point2Df> >& dstPoints,
                                          Transform2Df &fundamental)
{

    cv::Mat F;
    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;

    points_view1.resize(srcPoints.size());
    points_view2.resize(dstPoints.size());

    double minVal, maxVal;


    std::vector<uchar>status(points_view1.size());

    for( int i = 0; i < srcPoints.size(); i++ ){
        points_view1[i].x=srcPoints.at(i)->getX();
        points_view1[i].y=srcPoints.at(i)->getY();

        points_view2[i].x=dstPoints.at(i)->getX();
        points_view2[i].y=dstPoints.at(i)->getY();
    }
    cv::minMaxIdx(points_view1, &minVal, &maxVal);
    F = cv::findFundamentalMat(points_view1, points_view2, cv::FM_RANSAC, 0.006 * maxVal, 0.99, status);
    if(!F.data){
        LOG_DEBUG("Fundamental matrix is empty")
        return api::solver::pose::FundamentalMatrixEstimation::FUNDAMENTALMATRIX_EMPTY;
	}

    F.convertTo(F,CV_32F);

    SolAROpenCVHelper::convertCVMatToSolar(F,fundamental);
    return api::solver::pose::FundamentalMatrixEstimation::FUNDAMENTALMATRIX_ESTIMATION_OK;
}

}
}
}
