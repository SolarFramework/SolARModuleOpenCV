#include "SolARBasicMatchesFilterOpencv.h"
#include "ComponentFactory.h"
#include "SolAROpenCVHelper.h"
#include <set>


using namespace org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARBasicMatchesFilterOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARBasicMatchesFilterOpencv::SolARBasicMatchesFilterOpencv()
{
    setUUID(SolARBasicMatchesFilterOpencv::UUID);
    addInterface<api::features::IMatchesFilter>(this,api::features::IMatchesFilter::UUID, "interface BasicMatchesilterOpencv");
}


SolARBasicMatchesFilterOpencv::~SolARBasicMatchesFilterOpencv(){

}


bool sortMatchByDistance(const std::pair<int,float> &lhs, const std::pair<int,float> &rhs)
{
    return lhs.second < rhs.second;
}

// filter matches : keep the best match in case of multiple matches per keypoint
void filterTheMatches(std::vector<cv::DMatch>& matches){

    std::map<int,std::vector<std::pair<int,float>>> matchesMap;
    for(std::vector<cv::DMatch>::iterator itr=matches.begin();itr!=matches.end();++itr){
        matchesMap[itr->trainIdx].push_back(std::make_pair(itr->queryIdx,itr->distance));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){

            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }

        cv::DMatch dm;
        dm.trainIdx=itr->first;
        dm.queryIdx=ptr.begin()->first;
        dm.distance=ptr.begin()->second;
        matches.push_back(dm);
    }


    matchesMap.clear();
    for(std::vector<cv::DMatch>::iterator itr=matches.begin();itr!=matches.end();++itr){
        matchesMap[itr->queryIdx].push_back(std::make_pair(itr->trainIdx,itr->distance));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }
        cv::DMatch dm;
        dm.queryIdx=itr->first;
        dm.trainIdx=ptr.begin()->first;
        dm.distance=ptr.begin()->second;
        matches.push_back(dm);
    }

}

// filter matches : keep the best match in case of multiple matches per keypoint
void SolARBasicMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>&inputMatches,
                                           std::vector<DescriptorMatch>&outputMatches,
                                           const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                                           const std::vector<SRef<Keypoint>>&inputKeyPointsB){

     std::vector<cv::DMatch> good_matches;

     for(auto m : inputMatches) {
         cv::DMatch match;
         match.queryIdx=m.getIndexInDescriptorA();
         match.trainIdx=m.getIndexInDescriptorB();
         match.distance=m.getMatchingScore();
         good_matches.push_back(match);
     }

     filterTheMatches(good_matches);

     outputMatches.clear();
     for(auto currentMatch : good_matches) {
            outputMatches.push_back(DescriptorMatch(currentMatch.queryIdx, currentMatch.trainIdx,currentMatch.distance ));
     }


}

}
}
}
