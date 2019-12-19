#include <sstream>
#include <numeric>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <optional>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <sophus/average.hpp>
#include "OdometryHelpers.h"
#include "PFOdometry.h"
#include "CoreConstants.h"

PFOdometry::TriangulationFunction::TriangulationFunction(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
}

template<typename T>
bool PFOdometry::TriangulationFunction::operator()(const T* const circle, T* landmark) const
{
    const T cx = circle[0];
    const T cy = circle[1];
    const T r = circle[2];

    const double IK00 = myCalibration->cameras[0].inverse_calibration_matrix(0,0);
    const double IK02 = myCalibration->cameras[0].inverse_calibration_matrix(0,2);
    const double IK11 = myCalibration->cameras[0].inverse_calibration_matrix(1,1);
    const double IK12 = myCalibration->cameras[0].inverse_calibration_matrix(1,2);

    const T los_cx = IK00*cx + IK02;
    const T los_cy = IK11*cy + IK12;
    const T los_cxminus = IK00*(cx-r) + IK02;
    const T los_cxplus = IK00*(cx+r) + IK02;
    const T los_cyminus = IK11*(cy-r) + IK12;
    const T los_cyplus = IK11*(cy+r) + IK12;

    const T alpha_xminus = ceres::atan(los_cxminus);
    const T alpha_xplus = ceres::atan(los_cxplus);
    const T alpha_yminus = ceres::atan(los_cyminus);
    const T alpha_yplus = ceres::atan(los_cyplus);

    const T los_dirx = ceres::tan( (alpha_xminus + alpha_xplus) / 2.0 );
    const T los_diry = ceres::tan( (alpha_yminus + alpha_yplus) / 2.0 );

    const T beta = ( (alpha_xplus - alpha_xminus)/2.0 + (alpha_yplus - alpha_yminus)/2.0 ) / 2.0;

    if( M_PI*0.3/180.0 < beta && beta < M_PI*150.0/180.0)
    {
        const T distance = CORE_LANDMARK_RADIUS / ceres::sin(beta);

        T dir[3];
        dir[0] = los_dirx;
        dir[1] = los_diry;
        dir[2] = T(1.0);

        const T norm = ceres::sqrt( dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2] );

        Eigen::Matrix<T, 3, 1> landmark_in_camera;
        landmark_in_camera.x() = distance*dir[0]/norm;
        landmark_in_camera.y() = distance*dir[1]/norm;
        landmark_in_camera.z() = distance*dir[2]/norm;

        //const Eigen::Matrix<T, 3, 1> landmark_in_world = myCameraToWorld.cast<T>() * landmark_in_camera;
        const Eigen::Matrix<T, 3, 1> landmark_in_world = landmark_in_camera;

        landmark[0] = landmark_in_world.x();
        landmark[1] = landmark_in_world.y();
        landmark[2] = landmark_in_world.z();

        return true;
    }
    else
    {
        return false;
    }
}

PFOdometry::ProjectionFunction::ProjectionFunction(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
}

template<typename T>
bool PFOdometry::ProjectionFunction::operator()(const T* const landmark, T* observation) const
{
    bool ok = true;

    const Eigen::Map< const Eigen::Matrix<T,3,1> > landmark_in_world(landmark);

    //const Eigen::Matrix<T,3,1> landmark_in_camera = myCameraToWorld.cast<T>() * landmark_in_world;
    const Eigen::Matrix<T,3,1> landmark_in_camera = landmark_in_world;

    if( landmark_in_camera.z() < CORE_LANDMARK_RADIUS*0.1 )
    {
        ok = false;
    }
    else
    {
        const T fx(myCalibration->cameras[0].calibration_matrix(0,0));
        const T fy(myCalibration->cameras[0].calibration_matrix(1,1));
        const T cx(myCalibration->cameras[0].calibration_matrix(0,2));
        const T cy(myCalibration->cameras[0].calibration_matrix(1,2));

        const T distance = landmark_in_camera.norm();

        const T alpha = ceres::asin( T(CORE_LANDMARK_RADIUS) / distance );

        T center_los[2];
        center_los[0] = landmark_in_camera.x() / landmark_in_camera.z();
        center_los[1] = landmark_in_camera.y() / landmark_in_camera.z();

        T beta[2];
        beta[0] = ceres::atan( center_los[0] );
        beta[1] = ceres::atan( center_los[1] );

        T tangentlos0[2];
        tangentlos0[0] = ceres::tan( beta[0] - alpha );
        tangentlos0[1] = center_los[1];

        T tangentlos1[2];
        tangentlos1[0] = ceres::tan( beta[0] + alpha );
        tangentlos1[1] = center_los[1];

        T tangentlos2[2];
        tangentlos2[0] = center_los[0];
        tangentlos2[1] = ceres::tan( beta[1] - alpha );

        T tangentlos3[2];
        tangentlos3[0] = center_los[0];
        tangentlos3[1] = ceres::tan( beta[1] + alpha );

        T tanpoint0[2];
        tanpoint0[0] = fx * tangentlos0[0] + cx;
        tanpoint0[1] = fy * tangentlos0[1] + cy;

        T tanpoint1[2];
        tanpoint1[0] = fx * tangentlos1[0] + cx;
        tanpoint1[1] = fy * tangentlos1[1] + cy;

        T tanpoint2[2];
        tanpoint2[0] = fx * tangentlos2[0] + cx;
        tanpoint2[1] = fy * tangentlos2[1] + cy;

        T tanpoint3[2];
        tanpoint3[0] = fx * tangentlos3[0] + cx;
        tanpoint3[1] = fy * tangentlos3[1] + cy;

        const T proj_x = ( tanpoint0[0] + tanpoint1[0] + tanpoint2[0] + tanpoint3[0] ) / 4.0;
        const T proj_y = ( tanpoint0[1] + tanpoint1[1] + tanpoint2[1] + tanpoint3[1] ) / 4.0;
        const T proj_radius = ( ceres::abs(tanpoint1[0] - tanpoint0[0]) + ceres::abs(tanpoint3[1] - tanpoint2[1]) ) / 4.0;

        observation[0] = proj_x;
        observation[1] = proj_y;
        observation[2] = proj_radius;

        ok = true;
    }

    return ok;
}

PFOdometry::PFOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;

    myPredictionLinearVelocitySigma = CORE_LANDMARK_RADIUS*6.0;
    myPredictionAngularVelocitySigma = 0.15*M_PI;

    {
        const double circle_position_sdev = 3.0;
        const double circle_radius_sdev = 3.0;

        myObservationCovarianceMatrix.setZero();
        myObservationCovarianceMatrix(0,0) = circle_position_sdev*circle_position_sdev;
        myObservationCovarianceMatrix(1,1) = circle_position_sdev*circle_position_sdev;
        myObservationCovarianceMatrix(2,2) = circle_radius_sdev*circle_radius_sdev;
    }

    myProjectionFunction.reset(new CeresProjectionFunction(new ProjectionFunction(myCalibration)));
    myTriangulationFunction.reset(new CeresTriangulationFunction(new TriangulationFunction(myCalibration)));
}

bool PFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    const bool successful_tracking =

        (circles.size() >= 3) && bool(myCurrentState) && bool(myWorkingState) &&

        updateParticles(timestamp, circles) &&

        resampleParticles();

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    exportCurrentState(output, successful_tracking);

    return true;
}

bool PFOdometry::updateParticles(double timestamp, const std::vector<TrackedCircle>& circles)
{
    struct NewLandmark
    {
        bool is_new;
        bool is_seen; // is_new implies is_seen.

        size_t observation; // valid only if is_seen.

        Eigen::Vector3d position_in_camera_frame; // valid only if is_new.
        Eigen::Matrix3d covariance_in_camera_frame; // valid only if is_new.

        size_t old_index; // valid only if !is_new
    };

    std::vector<NewLandmark> landmarks;

    const auto& old_state = *myCurrentState;
    auto& new_state = *myWorkingState;

    const size_t num_particles = old_state.particles.size();

    const double timestep = timestamp - old_state.timestamp;

    std::normal_distribution<double> normal_distribution;

    new_state.frame_id = old_state.frame_id+1;
    new_state.timestamp = timestamp;

    new_state.observations.resize(circles.size());
    for(size_t i=0; i<circles.size(); i++)
    {
        new_state.observations[i].undistorted_circle = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);
        new_state.observations[i].has_landmark = false;
        new_state.observations[i].landmark = 0;

        if( circles[i].has_previous && old_state.observations[circles[i].previous].has_landmark )
        {
            new_state.observations[i].has_landmark = true;
            new_state.observations[i].landmark = landmarks.size();

            landmarks.emplace_back();
            landmarks.back().is_new = false;
            landmarks.back().is_seen = true;
            landmarks.back().observation = i;
            landmarks.back().old_index = old_state.observations[circles[i].previous].landmark;
        }
        else
        {
            NewLandmark new_landmark;

            new_landmark.is_new = true;
            new_landmark.is_seen = true;
            new_landmark.observation = i;

            const bool ok = triangulateLandmarkInCameraFrame(
                new_state.observations[i].undistorted_circle,
                new_landmark.position_in_camera_frame,
                new_landmark.covariance_in_camera_frame);

            if(ok)
            {
                new_state.observations[i].has_landmark = true;
                new_state.observations[i].landmark = landmarks.size();

                landmarks.push_back(std::move(new_landmark));
            }
        }
    }

    new_state.particles.resize(num_particles);
    new_state.landmarks.resize({num_particles, landmarks.size()});

    for(size_t i=0; i<num_particles; i++)
    {
        new_state.particles[i].have_nonzero_likelihood = true;
        new_state.particles[i].log_likelihood = 0.0;

        {
            Sophus::SE3d::Tangent epsilon;
            epsilon <<
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine);

                //epsilon *= 0.0;

            new_state.particles[i].camera_to_world = old_state.particles[i].camera_to_world * Sophus::SE3d::exp(epsilon);
        }

        for(size_t j=0; j<landmarks.size(); j++)
        {
            if(landmarks[j].is_new)
            {
                transformCameraToWorldFrame(
                    landmarks[j].position_in_camera_frame,
                    landmarks[j].covariance_in_camera_frame,
                    new_state.particles[i].camera_to_world,
                    new_state.landmarks({i,j}).position,
                    new_state.landmarks({i,j}).covariance);
            }
            else if(landmarks[j].is_seen)
            {
                double log_likelihood = 0.0;

                new_state.particles[i].have_nonzero_likelihood =
                    new_state.particles[i].have_nonzero_likelihood && computeObservationLogLikelihood(
                        new_state.particles[i].camera_to_world,
                        new_state.observations[landmarks[j].observation].undistorted_circle,
                        old_state.landmarks({i,landmarks[j].old_index}),
                        log_likelihood);

                if(new_state.particles[i].have_nonzero_likelihood)
                {
                    new_state.particles[i].log_likelihood += log_likelihood;
                }

                updateLandmark(
                    new_state.particles[i].camera_to_world,
                    new_state.observations[landmarks[j].observation].undistorted_circle,
                    old_state.landmarks({i,landmarks[j].old_index}),
                    new_state.landmarks({i,j}) );
            }
        }
    }

    std::swap(myCurrentState, myWorkingState);

    return true;
}

void PFOdometry::exportCurrentState(OdometryFrame& output, bool aligned_wrt_previous)
{
    const State& s = *myCurrentState;

    const size_t num_particles = s.landmarks.size(0);
    const size_t num_landmarks = s.landmarks.size(1);

    output.timestamp = s.timestamp;

    output.aligned_wrt_previous = aligned_wrt_previous;

    // compute average pose.

    {
        std::vector<Sophus::SE3d> camera_to_world(num_particles);

        for(size_t i=0; i<num_particles; i++)
        {
            camera_to_world[i] = s.particles[i].camera_to_world;
        }

        Sophus::optional<Sophus::SE3d> myaverage = Sophus::average(camera_to_world);

        if(!myaverage)
        {
            throw std::runtime_error("internal error");
        }

        output.camera_to_world = *myaverage;
    }

    // compute average landmarks.
    // (we use kalman filter with f = g = id.

    {
        output.landmarks.resize(num_landmarks);

        for(size_t i=0; i<num_landmarks; i++)
        {
            Eigen::Vector3d mu = s.landmarks({0,i}).position;
            Eigen::Matrix3d sigma = s.landmarks({0,i}).covariance + Eigen::Matrix3d::Identity()*1.0e-2;

            for(size_t j=1; j<num_particles; j++)
            {
                const Eigen::Vector3d observation_residual = s.landmarks({j,i}).position - mu;
                const Eigen::Matrix3d observation_covariance = sigma + s.landmarks({j,i}).covariance + Eigen::Matrix3d::Identity()*1.0e-2;

                if( std::fabs(observation_covariance.determinant()) < 1.0e-7 )
                {
                    Eigen::JacobiSVD<Eigen::Matrix3d> svd(observation_covariance);

                    std::cout << j << std::endl;
                    std::cout << svd.singularValues().transpose() << std::endl;
                    std::cout << observation_covariance << std::endl;

                    throw std::runtime_error("internal error!");
                }

                const Eigen::Matrix3d gain = sigma * observation_covariance.inverse();

                const Eigen::Vector3d new_mu = mu + gain * observation_residual;
                const Eigen::Matrix3d new_sigma = sigma - gain*sigma;

                mu = new_mu;
                sigma = new_sigma;
            }

            output.landmarks[i].position = mu;
        }
    }

    /*
    if(ret == false)
    {
        output.timestamp = 0.0;
        output.aligned_wrt_previous = false;
        output.camera_to_world = Sophus::SE3d();
        output.landmarks.clear();
    }
    */
}

void PFOdometry::reset()
{
    myCurrentState.reset();
    myWorkingState.reset();
}

bool PFOdometry::computeObservationLogLikelihood(
    const Sophus::SE3d& camera_to_world,
    const cv::Vec3f& undistorted_circle,
    const Landmark& landmark,
    double& log_likelihood)
{
    bool ok = true;
    bool ret = false;

    log_likelihood = 0.0;

    Eigen::Vector3d predicted_observation;
    Eigen::Matrix3d predicted_observation_jacobian;

    if(ok)
    {
        Eigen::Vector3d tmp0 = camera_to_world.inverse() * landmark.position;
        const double* ceres_input[] = { tmp0.data() };

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> tmp2;
        double* ceres_jacobian[] = { tmp2.data() };

        ok = myProjectionFunction->Evaluate(ceres_input, predicted_observation.data(), ceres_jacobian);

        if(ok)
        {
            predicted_observation_jacobian = tmp2 * camera_to_world.rotationMatrix().transpose();
        }
    }

    if(ok)
    {
        const Eigen::Matrix3d covariance = myObservationCovarianceMatrix + predicted_observation_jacobian * landmark.covariance * predicted_observation_jacobian.transpose();

        const double det_covariance = covariance.determinant();

        if( det_covariance > 1.0e-5 )
        {
            const Eigen::Matrix3d inv_covariance = covariance.inverse();

            Eigen::Vector3d sensed_observation;
            sensed_observation.x() = undistorted_circle[0];
            sensed_observation.y() = undistorted_circle[1];
            sensed_observation.z() = undistorted_circle[2];

            const Eigen::Vector3d error = sensed_observation - predicted_observation;

            //std::cout << error.transpose() << std::endl;

            constexpr const double cte = -1.5 * std::log(2.0*M_PI);

            log_likelihood = cte - 0.5 * error.transpose() * inv_covariance * error - 0.5 * std::log(det_covariance);
            ret = true;
            //std::cout << log_likelihood << std::endl;
        }
        //std::cout << covariance << std::endl;
        //std::cout << std::endl;
    }

    //std::cout << ret << std::endl;

    return ret;
}

bool PFOdometry::resampleParticles()
{
    struct WeightedParticle
    {
        size_t particle;
        double weight;
    };

    const auto& old_state = *myCurrentState;
    auto& new_state = *myWorkingState;

    const size_t num_particles = old_state.landmarks.size(0);
    const size_t num_landmarks = old_state.landmarks.size(1);

    std::vector<WeightedParticle> bag;

    std::uniform_real_distribution<double> U(0.0, 1.0);

    bool ret = true;

    // compute particle selection.

    if(ret)
    {
        double total = 0.0;

        for(size_t i=0; i<num_particles; i++)
        {
            //std::cout << std::exp(old_state.particles[i].log_likelihood) << std::endl;
            if( old_state.particles[i].have_nonzero_likelihood )
            {
                bag.emplace_back();
                bag.back().particle = i;
                bag.back().weight = std::exp(old_state.particles[i].log_likelihood / double(num_landmarks));
                total += bag.back().weight;
            }
        }

        ret = (bag.empty() == false && total > 1.0e-9);

        if(ret)
        {
            for(WeightedParticle& p : bag)
            {
                p.weight /= total;
            }
        }
    }

    if(ret == false)
    {
        std::cout << "PFOdometry: no particle with non-zero likelihood!" << std::endl;
    }

    // compute new state.

    if(ret)
    {
        new_state.frame_id = old_state.frame_id;
        new_state.timestamp = old_state.timestamp;

        new_state.observations.resize(old_state.observations.size());
        std::copy(old_state.observations.begin(), old_state.observations.end(), new_state.observations.begin());

        new_state.particles.resize(old_state.particles.size());
        new_state.landmarks.resize({num_particles, num_landmarks});

        for(size_t i=0; i<num_particles; i++)
        {
            // sample a particle.

            size_t old_particle = 0;

            {
                const double u = U(myRandomEngine);

                size_t j = 0;
                double v = 0.0;

                while( j+1 < bag.size() && v + bag[j].weight < u )
                {
                    v += bag[j].weight;
                    j++;
                }

                old_particle = bag[j].particle;
            }

            // copy particle.

            new_state.particles[i] = old_state.particles[old_particle];

            for(size_t j=0; j<num_landmarks; j++)
            {
                new_state.landmarks({i,j}) = old_state.landmarks({old_particle,j});
            }
        }

        std::swap(myWorkingState, myCurrentState);
    }

    return ret;
}

void PFOdometry::transformCameraToWorldFrame(
    const Eigen::Vector3d& position_in_camera,
    const Eigen::Matrix3d& covariance_in_camera,
    const Sophus::SE3d& camera_to_world,
    Eigen::Vector3d& position_in_world,
    Eigen::Matrix3d& covariance_in_world)
{
    position_in_world = camera_to_world * position_in_camera;

    const Eigen::Matrix3d R = camera_to_world.rotationMatrix();

    covariance_in_world = R * covariance_in_camera * R.transpose();
}

void PFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    std::vector<Landmark> landmarks;

    myCurrentState.reset(new State());
    myWorkingState.reset(new State());

    auto& state = *myCurrentState;

    state.frame_id = 0;
    state.timestamp = timestamp;

    state.observations.resize(circles.size());
    for(size_t i=0; i<circles.size(); i++)
    {
        state.observations[i].undistorted_circle = OdometryHelpers::undistortCircle(circles[i].circle, myCalibration);
        state.observations[i].has_landmark = false;
        state.observations[i].landmark = 0;

        Landmark landmark;
        const bool triangulated = triangulateLandmarkInCameraFrame(
            state.observations[i].undistorted_circle,
            landmark.position,
            landmark.covariance);

        if(triangulated)
        {
            state.observations[i].has_landmark = true;
            state.observations[i].landmark = landmarks.size();

            landmarks.push_back(landmark);
        }
    }

    const size_t num_particles = 1000;

    state.particles.resize(num_particles);
    state.landmarks.resize({num_particles, landmarks.size()});

    for(size_t i=0; i<num_particles; i++)
    {
        state.particles[i].have_nonzero_likelihood = true;
        state.particles[i].log_likelihood = 0.0;
        state.particles[i].camera_to_world = Sophus::SE3d();

        for(size_t j=0; j<landmarks.size(); j++)
        {
            state.landmarks({i,j}) = landmarks[j];
        }
    }
}

bool PFOdometry::triangulateLandmarkInCameraFrame(
    const cv::Vec3d& undistorted_circle, 
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    Eigen::Matrix<double,3,3,Eigen::RowMajor> jacobian;
    bool ok = true;

    if(ok)
    {
        const double* ceres_input[] = { undistorted_circle.val };
        double* ceres_jacobian[] = { jacobian.data() };

        ok = myTriangulationFunction->Evaluate( ceres_input, position.data(), ceres_jacobian );
    }

    if(ok)
    {
        covariance = jacobian * myObservationCovarianceMatrix * jacobian.transpose();
    }

    return ok;
}

void PFOdometry::updateLandmark(
    const Sophus::SE3d& camera_to_world,
    const cv::Vec3f& undistorted_circle,
    const Landmark& old_landmark,
    Landmark& new_landmark)
{
    bool ok = true;

    Eigen::Vector3d predicted_observation;
    Eigen::Matrix3d predicted_observation_jacobian;

    Eigen::Vector3d sensed_observation;
    sensed_observation(0) = undistorted_circle[0];
    sensed_observation(1) = undistorted_circle[1];
    sensed_observation(2) = undistorted_circle[2];

    const Eigen::Matrix3d sensed_observation_covariance = myObservationCovarianceMatrix;

    new_landmark = old_landmark;

    if(ok)
    {
        const Eigen::Vector3d in_camera = camera_to_world.inverse() * old_landmark.position;

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian;

        const double* ceres_input[] = { in_camera.data() };
        double* ceres_jacobian[] = { jacobian.data() };

        ok = myProjectionFunction->Evaluate(ceres_input, predicted_observation.data(), ceres_jacobian);

        if(ok)
        {
            predicted_observation_jacobian = jacobian * camera_to_world.rotationMatrix().transpose();
        }
    }

    if(ok)
    {
        const Eigen::Vector3d residual = sensed_observation - predicted_observation;

        const Eigen::Matrix3d residual_covariance = predicted_observation_jacobian * old_landmark.covariance * predicted_observation_jacobian.transpose() + sensed_observation_covariance;

        if( std::fabs(residual_covariance.determinant()) > 1.0e-4 )
        {
            const Eigen::Matrix3d inv_residual_covariance = residual_covariance.inverse();

            new_landmark.position = old_landmark.position + old_landmark.covariance * predicted_observation_jacobian.transpose() * inv_residual_covariance * residual;
            new_landmark.covariance = old_landmark.covariance - old_landmark.covariance * predicted_observation_jacobian.transpose() * inv_residual_covariance * predicted_observation_jacobian * old_landmark.covariance;
        }
    }
}

