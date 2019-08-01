/**
 * @file AKAZEConfig.h
 * @brief AKAZE configuration file
 * @date Feb 23, 2014
 * @author Pablo F. Alcantarilla, Jesus Nuevo
 */

#ifndef __OPENCV_FEATURES_2D_AKAZE_CONFIG_H__
#define __OPENCV_FEATURES_2D_AKAZE_CONFIG_H__

#include <opencv2/features2d.hpp>

namespace cv
{
/* ************************************************************************* */
/// AKAZE configuration options structure
struct AKAZEOptionsV2 {

    AKAZEOptionsV2() = default;

    int omax{4};                       ///< Maximum octave evolution of the image 2^sigma (coarsest scale sigma units)
    int nsublevels{4};                 ///< Default number of sublevels per scale level
    int img_width{0};                  ///< Width of the input image
    int img_height{0};                 ///< Height of the input image
    float soffset{1.6f};                  ///< Base scale offset (sigma units)
    float derivative_factor{1.5f};        ///< Factor for the multiscale derivatives
    float sderivatives{1.0};             ///< Smoothing factor for the derivatives
    int diffusivity{KAZE::DIFF_PM_G2};   ///< Diffusivity type

    float dthreshold{0.001f};               ///< Detector response threshold to accept point
    float min_dthreshold{0.00001f};           ///< Minimum detector threshold to accept a point

    int descriptor{AKAZE::DESCRIPTOR_MLDB};     ///< Type of descriptor
    int descriptor_size{0};            ///< Size of the descriptor in bits. 0->Full size
    int descriptor_channels{3};        ///< Number of channels in the descriptor (1, 2, 3)
    int descriptor_pattern_size{10};    ///< Actual patch size is 2*pattern_size*point.scale

    float kcontrast_percentile{0.7f};     ///< Percentile level for the contrast factor
    int kcontrast_nbins{300};            ///< Number of bins for the contrast factor histogram
};

}

#endif
