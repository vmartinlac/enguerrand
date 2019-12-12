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

    myPredictionLinearVelocitySigma = CORE_LANDMARK_RADIUS*10.0;
    myPredictionAngularVelocitySigma = 0.6*M_PI;

    myCirclePositionNoise = 2.0;
    myCircleRadiusNoise = 2.0;

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

            const bool ok = triangulateLandmark(
                Sophus::SE3d(),
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
        {
            Sophus::SE3d::Tangent epsilon;
            epsilon <<
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionLinearVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine),
                timestep * myPredictionAngularVelocitySigma * normal_distribution(myRandomEngine);

            new_state.particles[i].camera_to_world = old_state.particles[i].camera_to_world * Sophus::SE3d::exp(epsilon);
            new_state.particles[i].importance_factor = 1.0;
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
                const double likelihood = computeObservationLikelihood(
                    new_state.particles[i].camera_to_world,
                    new_state.observations[landmarks[j].observation].undistorted_circle,
                    old_state.landmarks({i,landmarks[j].old_index}));

                new_state.particles[i].importance_factor *= likelihood;

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
    throw "Not implemented!"; // TODO
    /*
    bool ret = true;

    State& s = *myCurrentState;

    output.timestamp = s.timestamp;

    output.aligned_wrt_previous = aligned_wrt_previous;

    // compute average pose.

    if(ret)
    {
        std::vector<Sophus::SE3d> camera_to_world(myNumParticles);

        for(size_t i=0; i<myNumParticles; i++)
        {
            camera_to_world[i] = s.particles[i].camera_to_world;
        }

        Sophus::optional<Sophus::SE3d> myaverage = Sophus::average(camera_to_world);

        if(myaverage)
        {
            output.camera_to_world = *myaverage;
        }
        else
        {
            ret = false;
            std::cerr << "PFOdometry: averaging poses failed!" << std::endl;
        }
    }

    // compute average landmarks.
    // (we use kalman filter with f = g = id.

    if(ret)
    {
        const size_t num_particles = s.landmark_estimations.size(0);
        const size_t num_landmarks = s.landmark_estimations.size(1);

        output.landmarks.resize(num_landmarks);

        for(size_t i=0; i<num_landmarks; i++)
        {
            Eigen::Vector3d mu = s.landmark_estimations({0,i}).position;
            Eigen::Matrix3d sigma = s.landmark_estimations({0,i}).covariance;

            for(size_t j=1; j<num_particles; j++)
            {
                const Eigen::Vector3d observation_residual = s.landmark_estimations({j,i}).position - mu;
                const Eigen::Matrix3d observation_covariance = sigma + s.landmark_estimations({j,i}).covariance;
                const Eigen::Matrix3d gain = sigma * observation_covariance.inverse();

                const Eigen::Vector3d new_mu = mu + gain * observation_residual;
                const Eigen::Matrix3d new_sigma = sigma - gain*sigma;

                mu = new_mu;
                sigma = new_sigma;
            }

            output.landmarks[i].position = mu;
        }
    }

    if(ret == false)
    {
        output.timestamp = 0.0;
        output.aligned_wrt_previous = false;
        output.camera_to_world = Sophus::SE3d();
        output.landmarks.clear();
    }

    return ret;
    */
}

void PFOdometry::reset()
{
    myCurrentState.reset();
    myWorkingState.reset();
}

double PFOdometry::computeObservationLikelihood(
    const Sophus::SE3d& camera_to_world,
    const cv::Vec3f& undistorted_circle,
    const Landmark& landmark)
{
    double ret = 0.0;

    bool ok = true;

    Eigen::Vector3d predicted_observation;
    Eigen::Matrix3d predicted_observation_jacobian;

    Eigen::Matrix3d observation_covariance;
    observation_covariance <<
        myCirclePositionNoise*myCirclePositionNoise, 0.0, 0.0,
        0.0, myCirclePositionNoise*myCirclePositionNoise, 0.0,
        0.0, 0.0, myCircleRadiusNoise*myCircleRadiusNoise;

    if(ok)
    {
        Eigen::Vector3d tmp0 = camera_to_world.inverse() * landmark.position;
        const double* tmp1 = tmp0.data();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> tmp2;
        double* tmp3 = tmp2.data();

        ok = myProjectionFunction->Evaluate(&tmp1, predicted_observation.data(), &tmp3);

        if(ok)
        {
            //predicted_observation_jacobian
        }
    }

    if(ok)
    {
        const Eigen::Matrix3d covariance = observation_covariance + predicted_observation_jacobian * landmark.covariance * predicted_observation_jacobian.transpose();

        const double det_covariance = covariance.determinant();

        if( det_covariance > 1.0e-5 )
        {
            const Eigen::Matrix3d inv_covariance = covariance.inverse();

            Eigen::Vector3d sensed_observation;
            sensed_observation.x() = undistorted_circle[0];
            sensed_observation.y() = undistorted_circle[1];
            sensed_observation.z() = undistorted_circle[2];

            const Eigen::Vector3d error = predicted_observation - sensed_observation;

            //std::cout << error.transpose() << std::endl;

            constexpr const double cte = 1.0 / std::pow(2.0*M_PI, 3.0/2.0);

            ret = cte * std::exp(-0.5 * error.transpose() * inv_covariance * error) / std::sqrt(det_covariance);
        }
    }

    return ret;
}

bool PFOdometry::resampleParticles()
{
    const auto& old_state = *myCurrentState;
    auto& new_state = *myWorkingState;

    const size_t num_particles = old_state.particles.size();

    std::vector<double> weights(num_particles);
    std::vector<size_t> sample(num_particles);

    double total_weight = 0.0;

    for(size_t i=0; i<num_particles; i++)
    {
        //total_weight += old_state
        weights[i] = total_weight;
    }

    new_state.frame_id = old_state.frame_id;
    new_state.timestamp = old_state.timestamp;

    //new_state.observations = old_state.observations.size();


    std::swap(myWorkingState, myCurrentState);

    return true;
}

void PFOdometry::transformCameraToWorldFrame(
    const Eigen::Vector3d& position_in_camera,
    const Eigen::Matrix3d& covariance_in_camera,
    const Sophus::SE3d& camera_to_world,
    Eigen::Vector3d& position_in_world,
    Eigen::Matrix3d& covariance_in_world)
{
    // TODO
    throw "not implemented!";
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
        const bool triangulated = triangulateLandmark(
            Sophus::SE3d(),
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

    const size_t num_particles = 200;

    state.particles.resize(num_particles);
    state.landmarks.resize({num_particles, landmarks.size()});

    for(size_t i=0; i<num_particles; i++)
    {
        state.particles[i].importance_factor = 1.0;
        state.particles[i].camera_to_world = Sophus::SE3d();

        for(size_t j=0; j<landmarks.size(); j++)
        {
            state.landmarks({i,j}) = landmarks[j];
        }
    }
}

/*
bool PFOdometry::resamplingStep()
{
    bool ret = true;

    Eigen::Matrix3d observation_covariance;
    observation_covariance <<
        myCirclePositionNoise*myCirclePositionNoise, 0.0, 0.0,
        0.0, myCirclePositionNoise*myCirclePositionNoise, 0.0,
        0.0, 0.0, myCircleRadiusNoise*myCircleRadiusNoise;

    std::unique_ptr<CeresLandmarkObservationFunction> function;
    function.reset(new CeresLandmarkObservationFunction(new LandmarkObservationFunction(myCalibration)));

    std::vector<bool> failed(myNumParticles, false);
    std::vector<double> weights(myNumParticles);

    std::fill(weights.begin(), weights.end(), 1.0);

    for(size_t observation_index=0; observation_index<myCurrentState->observations.size(); observation_index++)
    {
        if(myCurrentState->observations[observation_index].has_landmark)
        {
            const size_t landmark_index = myCurrentState->observations[observation_index].landmark;

            for(size_t particle_index=0; particle_index<myNumParticles; particle_index++)
            {
            }
        }
    }

    // resample.

    {
        bool all_failed = true;
        bool sum_weights = 0.0;

        for(size_t i=0; i<myNumParticles; i++)
        {
            all_failed = all_failed && failed[i];
            sum_weights += weights[i];
            std::cout << weights[i] << std::endl;
        }

        ret = (!all_failed) && (sum_weights > 1.0e-4);

        if(ret)
        {
            std::discrete_distribution<size_t> distrib(weights.begin(), weights.end());

            myWorkingState->particles.resize(myNumParticles);

            for(size_t i=0; i<myNumParticles; i++)
            {
                const size_t j = distrib(myRandomEngine);

                myWorkingState->particles[i] = myCurrentState->particles[j];
            }

            myCurrentState->particles.swap(myWorkingState->particles);
        }
        else
        {
            std::cout << "PFOdometry: no predicted particle compatible with observations." << std::endl;
        }
    }

    return ret;
}
*/

bool PFOdometry::triangulateLandmark(
    const Sophus::SE3d& camera_to_world,
    const cv::Vec3d& undistorted_circle, 
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    Eigen::Matrix<double,3,3,Eigen::RowMajor> jacobian;

    const double* ceres_variable_ptr = undistorted_circle.val;
    double* ceres_jacobian_ptr = jacobian.data();

    const bool ok = myTriangulationFunction->Evaluate( &ceres_variable_ptr, position.data(), &ceres_jacobian_ptr );

    if(ok)
    {
        const double sigma_center = 1.5;
        const double sigma_radius = 1.5;

        Eigen::Matrix3d S0;
        S0 <<
            sigma_center*sigma_center, 0.0, 0.0,
            0.0, sigma_center*sigma_center, 0.0,
            0.0, 0.0, sigma_radius*sigma_radius;

        covariance = jacobian * S0 * jacobian.transpose();

        //std::cout << sqrt(covariance(0,0)) << std::endl;
        //std::cout << sqrt(covariance(1,1)) << std::endl;
        //std::cout << sqrt(covariance(2,2)) << std::endl;
    }

    return ok;
}

void PFOdometry::updateLandmark(
    const Sophus::SE3d& camera_to_world,
    const cv::Vec3f& undistorted_circle,
    const Landmark& old_landmark,
    Landmark& new_landmark)
{
    bool ret = false;

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian;

    Eigen::Vector3d predicted_observation;

    Eigen::Vector3d sensed_observation;
    sensed_observation(0) = undistorted_circle[0];
    sensed_observation(1) = undistorted_circle[1];
    sensed_observation(2) = undistorted_circle[2];

    Eigen::Matrix3d sensed_observation_covariance;
    sensed_observation_covariance.setZero();
    sensed_observation_covariance(0,0) = myCirclePositionNoise*myCirclePositionNoise;
    sensed_observation_covariance(1,1) = myCirclePositionNoise*myCirclePositionNoise;
    sensed_observation_covariance(2,2) = myCircleRadiusNoise*myCircleRadiusNoise;

    // TODO: fix usage of myProjectionFunction
    throw;

    {
        const double* ceres_old_state = old_landmark.position.data();
        double* ceres_jacobian = jacobian.data();
        ret = myProjectionFunction->Evaluate(&ceres_old_state, predicted_observation.data(), &ceres_jacobian);
    }

    if(ret)
    {
        const Eigen::VectorXd residual = sensed_observation - predicted_observation;

        const Eigen::MatrixXd residual_covariance = jacobian * old_landmark.covariance * jacobian.transpose() + sensed_observation_covariance;

        Eigen::FullPivHouseholderQR<Eigen::Matrix3d> solver;
        solver.compute(residual_covariance);

        if(solver.isInvertible())
        {
            new_landmark.position = old_landmark.position + old_landmark.covariance * jacobian.transpose() * solver.solve(residual);
            new_landmark.covariance = old_landmark.covariance - old_landmark.covariance * jacobian.transpose() * solver.solve(jacobian * old_landmark.covariance);
        }
    }
}

