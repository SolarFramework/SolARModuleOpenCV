#include <iostream>
#include <string>

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "SolARImageLoaderOpencv.h"
#include "SolARImageViewerOpencv.h"
#include <SolARImageConvertorOpencv.h>
#include <SolarImageFilterOpencv.h>

#include <iostream>
#include <map>

using namespace std;
using namespace SolAR;


using namespace SolAR;

namespace xpcf  = org::bcom::xpcf;


void run(){
   // UUID GENERATOR
    boost::uuids::string_generator gen;

     sptrnms::shared_ptr<ISolARImageLoader>     imageLoader;
    sptrnms::shared_ptr<ISolARImageViewer>     imageViewer;
    sptrnms::shared_ptr<ISolARImageFilter>     imageFilter;
    sptrnms::shared_ptr<ISolARImageConvertor>  imageConvertor;
    SRef<SolARImage> inputImage;

    //SPECIFIC IMPLEMENTATION (OPENCV) LOADING
    xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(ISolARImageLoader::UUID ),       imageLoader);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(ISolARImageViewer::UUID ),       imageViewer);
    xpcf::ComponentFactory::createComponent<SolARImageConvertorOpencv>(gen(ISolARImageConvertor::UUID ),imageConvertor);
    xpcf::ComponentFactory::createComponent<SolARImageFilterOpencv>(gen(ISolARImageFilter::UUID ),   imageFilter);


    std::string  filename= "D:/cartman_2.png";
    imageLoader->loadImage(filename,inputImage);

    SRef<SolARImage> grayImage  = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_GREY,
                                     SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());


    SRef<SolARImage> binaryImage  = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_GREY,
                                     SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage> bluredMedianImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                          SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage> bluredGaussianImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                           SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage> bluredBilateralImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                             SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage> bluredImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                             SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage>  erodedImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                     SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());

    SRef<SolARImage>  dilatedImage = xpcf::utils::make_shared<SolARImage>(SolARImage::ImageLayout::LAYOUT_RGB,
                                     SolARImage::PixelOrder::INTERLEAVED,inputImage->getDataType());
    int kwidth = 21;
    int kheight = 21;

    // SOLAR IMAGE CONVERSION
   imageConvertor->convert(inputImage,grayImage);
   // SOLAR IMAGE FILTERING BINARIZATION
   imageFilter->binarize(grayImage, binaryImage,100, 255);
   // SOLAR IAGE FILTERING BLURRING
   imageFilter->blur(inputImage, bluredImage,1,kwidth,kheight,0);
   imageFilter->blur(inputImage, bluredGaussianImage,1,kwidth,kheight,1);
   imageFilter->blur(inputImage, bluredMedianImage,1,kwidth,kheight,2);
   imageFilter->blur(inputImage, bluredBilateralImage,1,kwidth,kheight,3);
   // SOLAR IMAGE FILTERING MORPHO
   imageFilter->erode(inputImage, erodedImage,2,10);
   imageFilter->dilate(inputImage, dilatedImage,2,10);
   // SOLAR IMAGE VIEW
   imageViewer->display("original image",inputImage);
   imageViewer->display("gray image",grayImage);
   imageViewer->display("blur image",bluredImage);
   imageViewer->display("blurGaussian image",bluredGaussianImage);
   imageViewer->display("blurMedian image",bluredMedianImage);
   imageViewer->display("blurBilateral image",bluredBilateralImage);
   imageViewer->display("eroded image",erodedImage);
   imageViewer->display("dilated image",dilatedImage);

    cv::waitKey(0);
}

int main(){
    run();
    return 0;
}

