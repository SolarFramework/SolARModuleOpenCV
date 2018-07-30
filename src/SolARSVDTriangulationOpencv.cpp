

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

#include "SolARSVDTriangulationOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "xpcf/component/ComponentFactory.h"


#include <map>
#include <random>

namespace xpcf  = org::bcom::xpcf;


#define EPSILON 0.0001
#define intrpmnmx(val,min,max) (max==min ? 0.0 : ((val)-min)/(max-min))



XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {
            SolARSVDTriangulationOpencv::SolARSVDTriangulationOpencv():ComponentBase(xpcf::toUUID<SolARSVDTriangulationOpencv>())
            {
                 addInterface<api::solver::map::ITriangulator>(this);



               LOG_DEBUG(" SolARSVDTriangulationOpencv constructor");

            }

            SolARSVDTriangulationOpencv::~SolARSVDTriangulationOpencv(){

            }


            cv::Mat_<double> SolARSVDTriangulationOpencv::iterativeLinearTriangulation(cv::Point3d &u,
                                                          cv::Matx34d&P,
                                                          cv::Point3d&u1,
                                                          cv::Matx34d&P1){

                double wi = 1, wi1 = 1;
                    cv::Mat_<double> X(4, 1);

                    cv::Mat_<double> X_ = linearTriangulation(u, P, u1, P1);
                    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

               //     std::cout<<" output of linear triangulation: "<<X<<std::endl;
                    for (int i = 0; i<10; i++) {
                                                 //recalculate weights
                        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
                        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

                        //breaking point
                        if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

                        wi = p2x;
                        wi1 = p2x1;

                        //reweight equations and solve
                        cv::Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
                            (u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
                            (u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
                            (u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
                        );
                        cv::Mat_<double> B = (cv::Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
                            -(u.y*P(2, 3) - P(1, 3)) / wi,
                            -(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
                            -(u1.y*P1(2, 3) - P1(1, 3)) / wi1
                            );

                        cv::solve(A, B, X_, cv::DECOMP_SVD);
                        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
                    }
                    return X;

            }

            cv::Mat_<double> SolARSVDTriangulationOpencv::linearTriangulation(cv::Point3d &u,
                                                          cv::Matx34d&P,
                                                          cv::Point3d&u1,
                                                          cv::Matx34d&P1){

                cv::Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
                        u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
                        u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
                        u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
                    );
                    cv::Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
                        -(u.y*P(2, 3) - P(1, 3)),
                        -(u1.x*P1(2, 3) - P1(0, 3)),
                        -(u1.y*P1(2, 3) - P1(1, 3)));

                    cv::Mat_<double> X;
                    cv::solve(A, B, X, cv::DECOMP_SVD);
                    return X;
            }



            double SolARSVDTriangulationOpencv::getReprojectionErrorCloud(const std::vector<SRef<CloudPoint>>& original){
                double err = 0.f;
                for(auto &p: original){
                    err+=p->getReprojError();
                }
                return (err/=double(original.size()));
            }


            double SolARSVDTriangulationOpencv::triangulate(const std::vector<SRef<Point2Df>>& pt2d_1,
                                                            const std::vector<SRef<Point2Df>>& pt2d_2,
                                                            const std::vector<DescriptorMatch>&matches,
                                                            const std::pair<int,int>&working_views,
                                                            const Transform3Df&pose_1,
                                                            const Transform3Df&pose_2,
                                                            const CamCalibration&cam,
                                                            const CamDistortion&distorsion,
                                                            std::vector<SRef<CloudPoint>>& pcloud){

                cv::Matx34d P( pose_1(0, 0),pose_1(0, 1),pose_1(0, 2), pose_1(0, 3),
                               pose_1(1, 0),pose_1(1, 1),pose_1(1, 2), pose_1(1, 3),
                               pose_1(2, 0),pose_1(2, 1),pose_1(2, 2), pose_1(2, 3));




                cv::Matx34d P1(pose_2(0, 0),pose_2(0, 1),pose_2(0, 2), pose_2(0, 3),
                               pose_2(1, 0),pose_2(1, 1),pose_2(1, 2), pose_2(1, 3),
                               pose_2(2, 0),pose_2(2, 1),pose_2(2, 2), pose_2(2, 3));

                    cv::Mat_<double> Kinv;
                    cv::Mat K(3,3,CV_64FC1);
                     cv::Mat dist(1,4,CV_64FC1);
                    for(int i = 0; i < 3; ++i){
                        for(int j = 0; j < 3; ++j){
                            K.at<double>(i,j)= cam(i,j);
                        }
                    }

                    for(int i = 0; i < 4; ++i){
                        dist.at<double>(i) = distorsion(i);
                    }

                    /*
                    std::cout<<"K_cv: "<<K<<std::endl;
                    std::cout<<"dist_cv: "<<dist<<std::endl;
                    */
                    cv::invert(K,Kinv);
                    double t = cv::getTickCount();
                    std::vector<double> reproj_error;
                    unsigned int pts_size = pt2d_1.size();
                    cv::Mat_<double> KP1 = K * cv::Mat(P1);

                    for (int i = 0; i<pts_size; i++) {
                        cv::Point2f kp = cv::Point2f(pt2d_1[i]->getX(),pt2d_1[i]->getY());
                        cv::Point3d u(kp.x, kp.y, 1.0);
                        cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
                        u.x = um(0); u.y = um(1); u.z = um(2);

                        cv::Point2f kp1 = cv::Point2f(pt2d_2[i]->getX(),pt2d_2[i]->getY());
                        cv::Point3d u1(kp1.x, kp1.y, 1.0);


                        cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
                        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

                        /*
                        std::cout<<"u: "<<u<<std::endl;
                        std::cout<<"P: "<<P<<std::endl;

                         std::cout<<"u1: "<<u1<<std::endl;
                         std::cout<<"P1: "<<P1<<std::endl;
                         */

                        cv::Mat_<double> X = iterativeLinearTriangulation(u, P, u1, P1);

                   //     std::cout<<"X: "<<X<<std::endl;

                        cv::Mat_<double> xPt_img = KP1 * X;				//reproject
                   //       std::cout <<	"Point * K: " << xPt_img << std::endl;
                        cv::Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));

                        double reprj_err = norm(xPt_img_ - kp1);
                        reproj_error.push_back(reprj_err);

                        xpcf::utils::shared_ptr<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>();
                        std::vector<int>visibility = std::vector<int>(50, -1);

                       visibility[working_views.first]  = matches[i].getIndexInDescriptorA();
                       visibility[working_views.second] = matches[i].getIndexInDescriptorB();

                       cp = xpcf::utils::make_shared<CloudPoint>(X(0), X(1), X(2),0.0,0.0,0.0,reprj_err,visibility);
                       pcloud.push_back(cp);
                     }
                    cv::Scalar mse = cv::mean(reproj_error);
                    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
                    return mse[0];
                }


        }
    }
}
