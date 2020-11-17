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

#include "SolARDescriptorsExtractorSBPatternOpencv.h"
#include "SolAROpenCVHelper.h"

namespace xpcf = org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARDescriptorsExtractorSBPatternOpencv::SolARDescriptorsExtractorSBPatternOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorsExtractorSBPatternOpencv>())
    {
        declareInterface<api::features::IDescriptorsExtractorSBPattern>(this);

        declareProperty("patternSize", m_patternSize);
    }

    FrameworkReturnCode SolARDescriptorsExtractorSBPatternOpencv::extract(const SquaredBinaryPattern & pattern, SRef<DescriptorBuffer> & descriptor)
    {
        SquaredBinaryPatternMatrix matrix = pattern.getPatternMatrix();

        descriptor = xpcf::utils::make_shared<DescriptorBuffer>(DescriptorType::SBPATTERN, DescriptorDataType::TYPE_8U, matrix.rows() * matrix.cols(), 1);
        unsigned char* descriptorData = (unsigned char*)descriptor->data();

        int nbCols = matrix.cols();
        for (int i = 0; i < matrix.rows(); i++)
            for (int j = 0; j < nbCols; j++)
            {
              descriptorData[i * nbCols + j] = matrix(i,j);
            }
        return FrameworkReturnCode::_SUCCESS;
    }


    FrameworkReturnCode SolARDescriptorsExtractorSBPatternOpencv::extract(const std::vector<SRef<Image>> & inputImages,
                                                                          const std::vector<Contour2Df> & contours,
                                                                          SRef<DescriptorBuffer> & pattern_descriptors,
                                                                          std::vector<Contour2Df> & recognized_contours)
    {
        recognized_contours.clear();
        std::vector<size_t> recognizedPatterns;
        size_t i;
        for (i = 0; i < inputImages.size(); i++)
        {
            if (isPattern(inputImages[i]))
                recognizedPatterns.push_back(i);
        }
        if (recognizedPatterns.size()== 0)
        {
            pattern_descriptors = xpcf::utils::make_shared<DescriptorBuffer>(DescriptorType::SBPATTERN, DescriptorDataType::TYPE_8U, m_patternSize*m_patternSize, 0);
            return FrameworkReturnCode::_SUCCESS;
        }

        pattern_descriptors = xpcf::utils::make_shared<DescriptorBuffer>(DescriptorType::SBPATTERN, DescriptorDataType::TYPE_8U, m_patternSize*m_patternSize, static_cast<uint32_t>(recognizedPatterns.size()*4));
        int descriptor_size = pattern_descriptors->getDescriptorByteSize();

        for (i = 0; i < recognizedPatterns.size(); i++)
        {
            unsigned char* data = (unsigned char*)pattern_descriptors->data() + (i*4*descriptor_size);
            if (getPatternDescriptorFromImage(inputImages[recognizedPatterns[i]], data) == FrameworkReturnCode::_SUCCESS)
            {
                Contour2Df recognizedContour = contours[recognizedPatterns[i]];
                recognized_contours.push_back(recognizedContour);
                unsigned char* descriptorDataTemp = data;
                Contour2Df rotatedContourTemp = recognizedContour;
                //check all possible rotations
                for (int j = 0; j < 3; j++)
                {
                    // An array to store the descriptor buffer
                     unsigned char* descriptorData = descriptorDataTemp + descriptor_size;

                    // Rotate the current pattern 90° counter-clockwise to set the new rotated pattern
                     for (int row = 0; row < m_patternSize; row++)
                         for (int col = 0; col < m_patternSize; col++)
                         {
                             descriptorData[row * m_patternSize + col] = descriptorDataTemp[(col*m_patternSize) + m_patternSize - row - 1];
                         }
                     // Rotate the curent contour counter-clockwise to set the new rotated contour
                    Contour2Df rotatedContour;
                    for (int num_point = 0; num_point <4; num_point++)
                        rotatedContour.push_back(rotatedContourTemp[(num_point+1)%4]);

                    recognized_contours.push_back(rotatedContour);

                    descriptorDataTemp = descriptorData;
                    rotatedContourTemp = rotatedContour;
                 }
            }
        }

        return FrameworkReturnCode::_SUCCESS;
    }

    bool SolARDescriptorsExtractorSBPatternOpencv::isPattern(const SRef<Image> image)
    {
        cv::Mat cv_image = SolAROpenCVHelper::mapToOpenCV(image);

        //Markers are divided in nxm regions, of which the inner n-2xm-2 belongs to pattern info
        //We check here if the external border is entirely black

        int cellWidth = image->getWidth() / (m_patternSize+2);
        int cellHeight = image->getHeight() / (m_patternSize+2);

        for (int y = 0; y < m_patternSize+2; y++)
        {
            int inc = m_patternSize+1;

            if (y == 0 || y == m_patternSize+1) inc = 1; //for first and last row, check the whole border

            for (int x = 0; x<m_patternSize+2; x += inc)
            {
                int cellX = x * cellWidth;
                int cellY = y * cellHeight;
                cv::Mat cell = cv_image(cv::Rect(cellX, cellY, cellWidth, cellHeight));

                int nZ = cv::countNonZero(cell);

                if (nZ >(cellWidth*cellHeight) / 2)
                {
                    return false; //can not be a marker because the border element is not black!
                }
            }
        }
        return true;
    }

    FrameworkReturnCode SolARDescriptorsExtractorSBPatternOpencv::getPatternDescriptorFromImage (SRef<Image> image, unsigned char* data)
    {
        cv::Mat cv_image = SolAROpenCVHelper::mapToOpenCV(image);
        //cv::imshow("Recognized Pattern", grey);

        //Markers are divided in nxm regions, of which the inner n-2xm-2 belongs to pattern info
        //We check here if the external border is entirely black

        int cellWidth = image->getWidth() / (m_patternSize+2);
        int cellHeight = image->getHeight() / (m_patternSize+2);

        //get information(for each inner square, determine if it is  black or white)
        for (int y = 0; y<m_patternSize; y++)
        {
            for (int x = 0; x<m_patternSize; x++)
            {
                uint32_t cellX = (x + 1)*cellWidth;
                uint32_t cellY = (y + 1)*cellHeight;
                cv::Mat cell = cv_image(cv::Rect(cellX, cellY, cellWidth, cellHeight));

                // Check the number of white pixels in the cell.
                int nZ = cv::countNonZero(cell);
                //If there is a majority of white pixels, set the matrix value to 1, else to 0
                if (nZ>(cellWidth*cellHeight) / 2)
                    data[m_patternSize*y+x] = (unsigned char)1;
                else
                    data[m_patternSize*y+x] = (unsigned char)0;
            }
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
