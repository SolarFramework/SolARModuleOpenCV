#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolAROpenCVHelper.h"
#include <set>


using namespace org::bcom::xpcf;
namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARGeometricMatchesFilterOpencv::SolARGeometricMatchesFilterOpencv():ConfigurableBase(xpcf::toUUID<SolARGeometricMatchesFilterOpencv>())
{ 
    LOG_DEBUG("SolARGeometricMatchesFilterOpencv constructor")
    addInterface<api::features::IMatchesFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapDouble("confidenceLevel", m_confidenceLevel);
    params->wrapDouble("outlierDistanceRatio", m_outlierDistanceRatio);

}


SolARGeometricMatchesFilterOpencv::~SolARGeometricMatchesFilterOpencv(){

}



void SolARGeometricMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>&inputMatches,
                                               std::vector<DescriptorMatch>&outputMatches,
                                               const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                                               const std::vector<SRef<Keypoint>>&inputKeyPointsB){

    std::vector<DescriptorMatch>tempMatches;
    std::vector<uchar> status(inputKeyPointsA.size());
    std::vector<cv::Point2f> pts1, pts2;

    if(inputMatches.size()){

        // get Align matches
        for (unsigned int i = 0; i<inputMatches.size(); i++) {
            assert(inputMatches[i].getIndexInDescriptorA() < inputKeyPointsA.size());
            pts1.push_back(cv::Point2f(inputKeyPointsA[inputMatches[i].getIndexInDescriptorA()]->getX(),
                                       inputKeyPointsA[inputMatches[i].getIndexInDescriptorA()]->getY()));

            assert(inputMatches[i].getIndexInDescriptorB() < inputKeyPointsB.size());
            pts2.push_back(cv::Point2f(inputKeyPointsB[inputMatches[i].getIndexInDescriptorB()]->getX(),
                                       inputKeyPointsB[inputMatches[i].getIndexInDescriptorB()]->getY()));

        }


        cv::Mat F;
        {
            double minVal, maxVal;
            cv::minMaxIdx(pts1, &minVal, &maxVal);
            F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, m_outlierDistanceRatio * maxVal, m_confidenceLevel, status);
        }

        for (unsigned int i = 0; i<status.size(); i++) {
            if (status[i]) {
                   tempMatches.push_back(inputMatches[i]);
            }
        }
    }
    outputMatches=tempMatches;
    return;
}

}
}
}
