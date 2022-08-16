/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#ifndef OPENCV_EXPERIMENT_HELPERS_HPP_
#define OPENCV_EXPERIMENT_HELPERS_HPP_

#include <memory>
#include <datastructure/MathDefinitions.h>


using namespace SolAR::datastructure;

namespace SolAR::PnPTest
{

Vector3d rot2cayley( const Rotationd & R );

Rotationd cayley2rot( const Vector3d & cayley);

void generateCentralCameraSystem(   
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>> > & camRotations );

void generateRandomCameraSystem(
    int numberCameras,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>> > & camRotations );

void extractRelativePose(
    const Vector3d & position1,
    const Vector3d & position2,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation1,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation2,
    Vector3d & relativePosition,
    Maths::Matrix<double, 3, 3, Maths::RowMajor> & relativeRotation,
    bool normalize = true );

void printExperimentCharacteristics(
    const Vector3d & position,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation,
    double noise,
    double outlierFraction );

void printBearingVectorArraysMatlab(
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & bearingVectors1,
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & bearingVectors2 );

void printEssentialMatrix(
    const Vector3d & position,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation);

void getPerturbedPose(
    const Vector3d & position,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation,
    Vector3d & perturbedPosition,
    Maths::Matrix<double, 3, 3, Maths::RowMajor> & perturbedRotation,
    double amplitude );

std::vector<int> getNindices( int n );




void generateRandom2D3DCorrespondences(
    const Vector3d & position,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation,
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    const std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>>> & camRotations,
    size_t numberPoints,
    double noise,
    double outlierFraction,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & bearingVectors,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & points,
    std::vector<int> & camCorrespondences,
    Maths::MatrixXd & gt );

void generateMulti2D3DCorrespondences(
    const Vector3d & position,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation,
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    const std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>> > & camRotations,
    size_t pointsPerCam,
    double noise,
    double outlierFraction,
    std::vector<std::shared_ptr<std::vector<Vector3d, Maths::aligned_allocator<Vector3d>>>> & multiPoints,
    std::vector<std::shared_ptr<std::vector<Vector3d, Maths::aligned_allocator<Vector3d>>>> & multiBearingVectors,
    std::vector<std::shared_ptr<Maths::MatrixXd>>& gt );

void generateRandom2D2DCorrespondences(
    const Vector3d & position1,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation1,
    const Vector3d & position2,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation2,
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    const std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>>> & camRotations,
    size_t numberPoints,
    double noise,
    double outlierFraction,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & bearingVectors1,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & bearingVectors2,
    std::vector<int> & camCorrespondences1,
    std::vector<int> & camCorrespondences2,
    Maths::MatrixXd & gt );

void generateRandom3D3DCorrespondences(
    const Vector3d & position1,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation1,
    const Vector3d & position2,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation2,
    size_t numberPoints,
    double noise,
    double outlierFraction,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & points1,
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > & points2,
    Maths::MatrixXd & gt );

void generateMulti2D2DCorrespondences(
    const Vector3d & position1,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation1,
    const Vector3d & position2,
    const Maths::Matrix<double, 3, 3, Maths::RowMajor> & rotation2,
    const std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> & camOffsets,
    const std::vector<Maths::Matrix<double, 3, 3, Maths::RowMajor>, Maths::aligned_allocator<Maths::Matrix<double, 3, 3, Maths::RowMajor>> > & camRotations,
    size_t pointsPerCam,
    double noise,
    double outlierFraction,
    std::vector<std::shared_ptr<std::vector<Vector3d, Maths::aligned_allocator<Vector3d> >> > & multiBearingVectors1,
    std::vector<std::shared_ptr<std::vector<Vector3d, Maths::aligned_allocator<Vector3d> >> > & multiBearingVectors2,
    std::vector<std::shared_ptr<Maths::MatrixXd> > & gt );

}

#endif /* OPENCV_EXPERIMENT_HELPERS_HPP_ */
