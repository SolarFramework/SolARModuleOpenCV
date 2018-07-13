#include "SolARMapperOpencv.h"
#include <iostream>
#include <utility>


namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMapperOpencv)

namespace SolAR {
    using namespace datastructure;
        namespace MODULES {
            namespace OPENCV {

                SolARMapperOpencv::SolARMapperOpencv():ComponentBase(xpcf::toUUID<SolARMapperOpencv>())
                {
                     addInterface<IMapper>(this);
                #ifdef DEBUG
                    std::cout << " SolARMapperOpencv constructor" << std::endl;
                #endif
                    m_map = org::bcom::xpcf::utils::make_shared<Map>() ;
                }


                SolARMapperOpencv::~SolARMapperOpencv()
                {
                #ifdef DEBUG
                   std::cout << " SolARMapperOpencv destructor" << std::endl;
                #endif
                }

                void  SolARMapperOpencv::addNewKeyFrame(const SRef<Keyframe>&new_kframe){
                    m_kframes.push_back(new_kframe);
                }
                void  SolARMapperOpencv::removeKeyframe(const SRef<Keyframe>&old_kframe){
                }
                void SolARMapperOpencv::addMatches(const std::pair<int,int>&working_views,
                                                   const std::vector<DescriptorMatch>&new_matches){
                    m_gmatches[std::make_pair(working_views.first, working_views.second)] = new_matches;
                    // I have to add working view pair to extend matches matrix
                }


                bool SolARMapperOpencv::initMap(SRef<Keyframe>&kframe_t0,
                                                SRef<Keyframe>&kframe_t1,
                                                std::vector<SRef<CloudPoint>>&initCloud,
                                                std::vector<DescriptorMatch>&matches){
                    m_kframes.push_back(kframe_t0);
                    m_kframes.push_back(kframe_t1);
                    m_gmatches[std::make_pair(kframe_t0->m_idx, kframe_t1->m_idx)] = matches;
                    std::cout << " init map with "<<initCloud.size()<<" pointst"<<std::endl;
                    m_map->addCloudPoints(initCloud) ;
                    return true;
                }


                bool SolARMapperOpencv::updateMap(/*new_triangulated_cloud*/const SRef<Keyframe>&new_kframe,
                                                  const std::vector<DescriptorMatch>&new_matches){
					addNewKeyFrame(new_kframe);
                    std::pair<int,int>working_views = std::make_pair(new_kframe->m_idx-1,new_kframe->m_idx);
                    addMatches(working_views,new_matches);
                    // addPointCloud(m_cloud, new_cloud);

                    return true;
                }

                SRef<Keyframe> SolARMapperOpencv::getCurrentKeyframe(int idx){
                    if(idx < m_kframes.size()){
                         return m_kframes[idx];
                    }
                    else{
                        std::cerr<<"error, can't retrieve last keyframe, id must be lower than keyframes size"<<std::endl;
                    }
                }
                void SolARMapperOpencv::associateReferenceKeyFrameToFrame(SRef<Frame> frame)
                {
                    // for now : choose last one
                    frame->setReferenceKeyFrame((m_kframes[m_kframes.size()-1]));
                }

                SRef<Map> SolARMapperOpencv::getMap()
                {
                    return m_map ;
                }

                int SolARMapperOpencv::isKeyFrameCandidate(SRef<Frame> frame)
                {
                    bool enoughFrameSinceKf = ( frame->getNumberOfFramesSinceLastKeyFrame() >20) ; // more than 20 frames since last key frame
                    bool enoughTrackPoints = (frame->getCommonMapPointsWithReferenceKeyFrame().size() > 20) ; // enough map points seen in frame
                    float redudancyValue = ( (float) frame->getCommonMapPointsWithReferenceKeyFrame().size() / (float) frame->getReferenceKeyFrame()->getVisibleMapPoints().size()) ;  // percent of track kref map points seen in frame
                    bool notRedundancy = (redudancyValue < 0.9) ;

                    Transform3Df poseFrame = frame->m_pose ;
                    Transform3Df poseKeyFrame = frame->getReferenceKeyFrame()->m_pose ;

                    Vector3f tFrame = poseFrame.translation() ;
                    Vector3f tkeyFrame = poseKeyFrame.translation() ;
                    Vector3f difPos = tFrame - tkeyFrame ;
                    float translationDistance = difPos.norm() ;

                    Quaternionf rFrame(poseFrame.rotation()) ;
                    Quaternionf rkeyFrame(poseKeyFrame.rotation()) ;
                    float rotationDistance = rFrame.angularDistance(rkeyFrame) ;

					std::cout << " Translation Distance " << translationDistance << "  Angular Distance " << rotationDistance << std::endl;


                    // to do : add  these threshold as parameters somewhere
					bool enoughDistance = ((translationDistance > 0.15) || (rotationDistance > 3.14/8.0)) ;
					bool conditionAddKeyFrame = (enoughFrameSinceKf && enoughTrackPoints && notRedundancy && enoughDistance ) ;
                    // For now : 20 frames and enough track points only!!
					if (conditionAddKeyFrame)
                    {
                        std::cout << " Add new Key Frame" << std::endl ;
                        std::cout << " Nb Frames " << frame->getNumberOfFramesSinceLastKeyFrame() << std::endl ;
                        std::cout << " Track points " <<  frame->getCommonMapPointsWithReferenceKeyFrame().size() << std::endl ;
                        std::cout << " Redudancy " << redudancyValue << std::endl ;
                        std::cout << " Translation Distance " << translationDistance  << "  Angular Distance " << rotationDistance << std::endl ;
						
                        return m_kframes.size(); // index of key frame matches with array length

                    }
                    return -1 ;

                }


           }
        }
}
