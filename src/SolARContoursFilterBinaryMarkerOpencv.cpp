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

#include "SolARContoursFilterBinaryMarkerOpencv.h"
#include "SolAROpenCVHelper.h"

#include "opencv2/opencv.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARContoursFilterBinaryMarkerOpencv::SolARContoursFilterBinaryMarkerOpencv():ConfigurableBase(xpcf::toUUID<SolARContoursFilterBinaryMarkerOpencv>())
    {
        declareInterface<api::features::IContoursFilter>(this);
        declareProperty("minContourLength",m_minContourLength);
        declareProperty("espilon",m_epsilon);
        declareProperty("minDistanceBetweenContourCorners",m_minDistanceBetweenContourCorners);
    }

    // Compute the perimeter of a contour
    float computePerimeter(const Contour2Df & contour)
    {
        float sum = 0;
        for (size_t i = 0; i<contour.size(); i++)
        {
            size_t i2 = (i + 1) % contour.size();
            sum += ((contour[i])-(contour[i2])).norm();
        }
        return sum;
    }

    FrameworkReturnCode SolARContoursFilterBinaryMarkerOpencv::filter(const std::vector<Contour2Df> & input_contours, std::vector<Contour2Df> & output_contours)
    {
        std::vector<cv::Point2i> approxCurve;
        std::vector<Contour2Df> possibleMarkers;
        // For each contour, analyze if it is a parallelepiped likely to be the marker
        for (const Contour2Df &input_contour: input_contours)
        {
            // Approximate to a polygon
            double eps = input_contour.size() * m_epsilon;
            cv::approxPolyDP(SolAROpenCVHelper::convertToOpenCV(input_contour), approxCurve, eps, true);
            // We interested only in polygons that contains only four points and that are convex
            if ((approxCurve.size() == 4) && (cv::isContourConvex(approxCurve)))
            {
                // Ensure that the distance between consecutive points is large enough
                float minDist = std::numeric_limits<float>::max();
                for (int i = 0; i < 4; i++){
                    cv::Point side = approxCurve[i] - approxCurve[(i + 1) % 4];
                    float squaredSideLength = side.dot(side);
                    minDist = std::min(minDist, squaredSideLength);
                }
                // Check that distance is not very small
                if (minDist > m_minContourLength)
                {
                    Contour2Df contour;
                    for (int i = 0; i<4; i++)
                    {
                        contour.push_back(Point2Df(approxCurve[i].x, approxCurve[i].y));
                     }
                    Point2Df v1 = (contour[1]) - (contour[0]);
                    Point2Df v2 = (contour[2]) - (contour[0]);

                    double o = (v1[0] * v2[1]) - (v1[1] * v2[0]);
                    if (o < 0.0){		 //if the third point is in the left side, then sort in anti-clockwise order
                        Point2Df temp_point = contour[3];
                        contour[3] = contour[1];
                        contour[1] = temp_point;
                    }
                    possibleMarkers.push_back(contour);
                }
            }
        }

        // Remove candidates for which a corner is close to the same corner of another contour
        std::vector<std::pair<int, int>> tooNearCandidates;
        float minSquaredDistance = m_minDistanceBetweenContourCorners * m_minDistanceBetweenContourCorners;
        for (size_t i = 0; i<possibleMarkers.size(); i++)
        {
            //calculate the average distance of each corner to the nearest corner of the other marker candidate
            for (size_t j = i + 1; j<possibleMarkers.size(); j++)
            {
                float distSquared = 0;
                for (int c = 0; c < 4; c++)
                {
                    Point2Df v = (possibleMarkers[i])[c] - (possibleMarkers[j][c]);
                    distSquared += v.dot(v);
                }
                distSquared /= 4;
                if (distSquared < minSquaredDistance)
                {
                    tooNearCandidates.push_back(std::pair<int, int>(i, j));
                }
            }
        }

        // If two contours have the same corners that are too close, keep the contour with the bigger perimeter
        std::vector<bool> removalMask(possibleMarkers.size(), false);
        for (const std::pair<int, int> &tooNearCandidate: tooNearCandidates)
        {
            float p1 = computePerimeter(possibleMarkers[tooNearCandidate.first]);
            float p2 = computePerimeter(possibleMarkers[tooNearCandidate.second]);
            size_t removalIndex;
            if (p1 > p2)
                removalIndex = tooNearCandidate.second;
            else
                removalIndex = tooNearCandidate.first;

            removalMask[removalIndex] = true;
        }

        // Return candidates
        output_contours.clear();
        for (size_t i = 0; i<possibleMarkers.size(); i++)
        {
            if (!removalMask[i])
                output_contours.push_back(possibleMarkers[i]);
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
