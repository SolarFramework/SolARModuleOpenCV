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

#ifndef SolARMATCHESOVERLAYOPENCV_H
#define SolARMATCHESOVERLAYOPENCV_H
#include <vector>

#include "opencv2/core.hpp"

#include "api/display/IMatchesOverlay.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARMatchesOverlayOpencv
 * @brief <B>Displays matching keypoints between two images.</B>
 * <TT>UUID: e95302be-3fe1-44e0-97bf-a98380464af9</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARMatchesOverlayOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::IMatchesOverlay
{
public:
    SolARMatchesOverlayOpencv();

    /// @brief draw Match Lines.
    /// Draw all the lines joining the keypoints that match between two images
    /// @param[in] image1:  The first image on which have been extracted the first vector keypoints
    /// @param[in] image2: The second image on which have been extracted the second vector of keypoints
    /// @param[out] outImage: The resulting image merging image1 and image2 in side-by-side. If outImage has not been initialized, it will be done by this method. If it size is not the good one, it will be resized by this method.
    /// @param[in] points_image1: The keypoints of image1 that match with keypoints of image2. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image2.
    /// @param[in] points_image2: The keypoints of image2 that match with keypoints of image1. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image1.
    /// @param[in|out] matches, a vector of matches between the first and second image that will be displayed. If this vector is empty, we consider that the ith point of points_image1 matches with the ith point of points_image2.
    void draw(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <Point2Df> & points_image1, const std::vector <Point2Df> & points_image2, const std::vector<DescriptorMatch> matches = std::vector<DescriptorMatch>()) override;

    /// @brief draw Match Lines.
    /// Draw all the lines joining the keypoints that match between two images
    /// @param[in] image1:  The first image on which have been extracted the first vector keypoints
    /// @param[in] image2: The second image on which have been extracted the second vector of keypoints
    /// @param[out] outImage: The resulting image merging image1 and image2 in side-by-side. If outImage has not been initialized, it will be done by this method. If it size is not the good one, it will be resized by this method.
    /// @param[in] keypoints_image1: The keypoints of image1 that match with keypoints of image2. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image2.
    /// @param[in] keypoints_image2: The keypoints of image2 that match with keypoints of image1. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image1.
    /// @param[in|out] matches, a vector of matches between the first and second image that will be displayed. If this vector is empty, we consider that the ith point of points_image1 matches with the ith point of points_image2.
    void draw(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <Keypoint> & keypoints_image1, const std::vector <Keypoint> & keypoints_image2, const std::vector<DescriptorMatch> matches = std::vector<DescriptorMatch>()) override;

	/// @brief draw Match Lines.
    /// Draw all the lines joining the keylines that match between two images
    /// @param[in] image1:  The first image on which have been extracted the first vector keylines
    /// @param[in] image2: The second image on which have been extracted the second vector of keylines
    /// @param[out] outImage: The resulting image merging image1 and image2 in side-by-side. If outImage has not been initialized, it will be done by this method. If it size is not the good one, it will be resized by this method.
    /// @param[in] keylines_image1: The keylines of image1 that match with keylines of image2. The Nth keyline of this vector match with the Nth keyline of the vector points_image2.
    /// @param[in] keylines_image2: The keylines of image2 that match with keylines of image1. The Nth keyline of this vector match with the Nth keyline of the vector points_image1.
    /// @param[in|out] matches, a vector of matches between the first and second image that will be displayed. If this vector is empty, we consider that the ith line of keylines_image1 matches with the ith line of keylines_image2.
	void draw(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <Keyline> & keylines_image1, const std::vector <Keyline> & keylines_image2, const std::vector<DescriptorMatch> matches = std::vector<DescriptorMatch>()) override;
    
	/// @brief draw Match Lines.
    /// Draw all the lines joining the keypoints that match between two images
    /// @param[in] image: The image to display for drawing in overlay the matches
    /// @param[out] outImage: The resulting image diaplying matches in overlay. If outImage has not been initialized, it will be done by this method. If it size is not the good one, it will be resized by this method.
    /// @param[in] points_image1: The keypoints of image1 that match with keypoints of image2. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image2.
    /// @param[in] points_image2: The keypoints of image2 that match with keypoints of image1. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image1.
    /// @param[in|out] matches, a vector of matches between the first and second image that will be displayed. If this vector is empty, we consider that the ith point of points_image1 matches with the ith point of points_image2.
    void draw(const SRef<Image> image, SRef<Image> & outImage, const std::vector <Point2Df> & points_image1, const std::vector <Point2Df> & points_image2, const std::vector<DescriptorMatch> matches = std::vector<DescriptorMatch>()) override;

    /// @brief dra Match Lines.
    /// Draw all the lines joining the keypoints that match between two images
    /// @param[in] image: The image to display for drawing in overlay the matches
    /// @param[out] outImage: The resulting image diaplying matches in overlay. If outImage has not been initialized, it will be done by this method. If it size is not the good one, it will be resized by this method.
    /// @param[in] keypoints_image1: The keypoints of image1 that match with keypoints of image2. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image2.
    /// @param[in] keypoints_image2: The keypoints of image2 that match with keypoints of image1. The Nth keypoint of this vector match with the Nth keypoint of the vector points_image1.
    /// @param[in|out] matches, a vector of matches between the first and second image that will be displayed. If this vector is empty, we consider that the ith point of points_image1 matches with the ith point of points_image2.
    void draw(const SRef<Image> image, SRef<Image> & outImage, const std::vector <Keypoint> & keypoints_image1, const std::vector <Keypoint> & keypoints_image2, const std::vector<DescriptorMatch> matches = std::vector<DescriptorMatch>())override;

    void unloadComponent () override final;

private:
    /// @brief The color of the linse displaying the matches between the two images
    std::vector<unsigned int> m_color = {0,255,0};

    /// @brief if not null, the color will be randomized for each line.
    std::string m_mode = "COLOR";

    /// @brief the thickness of the lines displaying the matches between the two images
    unsigned int m_thickness = 1;

    /// @brief the maximum number of matches to display. If negative, all matches are displayed
    int m_maxMatches = -1; 

    /// @brief the minimal distance ratio compared to the image width for which the line will be green. Otherwise, the color will be red if the distance ratio of a matche is null, and will fade to yellow and then to green green when the distance ratio of a match will be equal to this value (used only for FADING mode).
    float m_minDistanceRatioGreen = 0.15f;
};

}
}
}

#endif // SolARMATCHESOVERLAYOPENCV_H
