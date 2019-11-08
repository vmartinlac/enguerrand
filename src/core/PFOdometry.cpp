#include "OdometryHelpers.h"
#include "PFOdometry.h"
#include "CoreConstants.h"

struct PFOdometry::TriangulationFunction
{

    TriangulationFunction(CalibrationDataPtr calibration, const Sophus::SE3d& camera_to_world)
    {
        myCalibration = calibration;
        myCameraToWorld = camera_to_world;
    }

    template<typename T>
    bool operator()(const T* const circle, T* landmark) const
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

            const Eigen::Matrix<T, 3, 1> landmark_in_world = myCameraToWorld.cast<T>() * landmark_in_camera;

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

    Sophus::SE3d myCameraToWorld;
    CalibrationDataPtr myCalibration;
};

struct PFOdometry::LandmarkObservationFunction
{
    LandmarkObservationFunction(CalibrationDataPtr calibration, const Sophus::SE3d& camera_to_world)
    {
        myCalibration = calibration;
        myCameraToWorld = camera_to_world;
    }

    template<typename T>
    bool operator()(const T* const landmark, T* observation) const
    {
        bool ok = true;

        const Eigen::Map< const Eigen::Matrix<T,3,1> > landmark_in_world(landmark);

        const Eigen::Matrix<T,3,1> landmark_in_camera = myCameraToWorld.cast<T>() * landmark_in_world;

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

    Sophus::SE3d myCameraToWorld;
    CalibrationDataPtr myCalibration;
};

PFOdometry::PFOdometry(CalibrationDataPtr calibration)
{
    myCalibration = calibration;
    myNumParticles = 5000;
    myPredictionPositionNoise = CORE_LANDMARK_RADIUS*0.6;
    myPredictionAttitudeNoise = M_PI*0.1;
    myCirclePositionNoise = 1.0;
    myCircleRadiusNoise = 1.5;
}

bool PFOdometry::track(double timestamp, const std::vector<TrackedCircle>& circles, OdometryFrame& output)
{
    const bool successful_tracking = trackAndMap(timestamp, circles);

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    myCurrentState->save(output, successful_tracking);

    return true;
}

void PFOdometry::State::save(OdometryFrame& output, bool aligned_wrt_previous)
{
    output.timestamp = timestamp;
    output.aligned_wrt_previous = aligned_wrt_previous;
    output.camera_to_world = Sophus::SE3d(); // TODO set average pose or take most likely one?
    output.landmarks.clear(); // TODO set average landmark positions or take most likely one?
}

void PFOdometry::reset()
{
    myCurrentState.reset();
    myWorkingState.reset();
}

void PFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    std::vector<Observation> observations;
    std::vector<LandmarkEstimation> prototype_landmark_estimations;
    Particle prototype_particle;

    // initialize prototypes.

    {
        prototype_particle.weight = 1.0;
        prototype_particle.camera_to_world = Sophus::SE3d(); //identity.

        observations.resize(circles.size());

        for(size_t i=0; i<circles.size(); i++)
        {
            observations[i].circle = circles[i].circle;

            LandmarkEstimation est;

            observations[i].has_landmark = triangulateLandmark(
                circles[i].circle,
                prototype_particle.camera_to_world,
                est.position,
                est.covariance);

            if(observations[i].has_landmark)
            {
                observations[i].landmark = prototype_landmark_estimations.size();
                prototype_landmark_estimations.push_back(std::move(est));
            }
        }
    }

    // create new state.

    {
        myCurrentState.reset(new State());
        myWorkingState.reset(new State());

        myCurrentState->frame_id = 0;
        myCurrentState->timestamp = timestamp;
        myCurrentState->num_landmarks = prototype_landmark_estimations.size();

        myCurrentState->observations.swap(observations);
        myCurrentState->particles.assign(myNumParticles, prototype_particle);
        myCurrentState->landmark_estimations.resize(prototype_landmark_estimations.size() * myNumParticles);

        for(size_t i=0; i<myNumParticles; i++)
        {
            for(size_t j=0; j<prototype_landmark_estimations.size(); j++)
            {
                myCurrentState->refLandmarkEstimation(i,j) = prototype_landmark_estimations[j];
            }
        }
    }
}

bool PFOdometry::trackAndMap(double timestamp, const std::vector<TrackedCircle>& circles)
{
    bool ret = bool(myCurrentState) && bool(myWorkingState);

    if(ret)
    {
        // sample predicted pose. This reduces to add noise to current pose.

        /*
        {
            myWorkingState->frame_id = myCurrentState->frame_id+1;
            myWorkingState->timestamp = timestamp;
            myWorkingState->particles.swap(myCurrentState->particles);
            myWorkingState->landmarks.swap(myCurrentState->landmarks);
            myWorkingState->landmark_estimations.swap(myCurrentState->landmark_estimations);

            std::normal_distribution<double> distribution; // N(0,1) distribution.

            for(Particle& p : myWorkingState->particles)
            {
                Sophus::SE3d::Tangent epsilon;
                epsilon <<
                    myPredictionPositionNoise*distribution(myRandomEngine),
                    myPredictionPositionNoise*distribution(myRandomEngine),
                    myPredictionPositionNoise*distribution(myRandomEngine),
                    myPredictionAttitudeNoise*distribution(myRandomEngine),
                    myPredictionAttitudeNoise*distribution(myRandomEngine),
                    myPredictionAttitudeNoise*distribution(myRandomEngine);

                p.camera_to_world = p.camera_to_world * Sophus::SE3d::exp(epsilon);
            }

            std::swap(myWorkingState, myCurrentState);
        }
        */

        // update landmarks.

        {
            std::swap(myWorkingState, myCurrentState);
            /*
            myWorkingState->frame_id = myCurrentState->frame_id;
            myWorkingState->timestamp = myCurrentState->timestamp;
            myWorkingState->particles.swap(myCurrentState->particles);
            myWorkingState->landmarks.swap(myCurrentState->landmarks);
            myWorkingState->landmark_estimations.swap(myCurrentState->landmark_estimations);
            */

            /*
            for(size_t i=0; i<myWorkingState->landmarks.size(); i++)
            {
                if( myWorkingState->landmarks[i].last_frame_id == myWorkingState->frame_id-1 )
                {
                    // TODO
                    for(size_t j=0; j<myWorkingState->particles.size(); j++)
                    {
                        updateLandmark(
                            myWorkingState->particles[i].camera_to_world,
                    }
                }
            }
            */

            std::swap(myWorkingState, myCurrentState);
        }

        // resample.

        {
        }

        // add new landmarks.

        {
            // TODO
        }
    }

    return ret;
}

bool PFOdometry::triangulateLandmark(
    const cv::Vec3f& circle, 
    const Sophus::SE3d& camera_to_world,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    const cv::Vec3d undistorted_circle = OdometryHelpers::undistortCircle(circle, myCalibration);

    Eigen::Matrix<double,3,3,Eigen::RowMajor> jacobian;

    const double* ceres_variable_ptr = undistorted_circle.val;
    double* ceres_jacobian_ptr = jacobian.data();

    TriangulationFunction* fn0 = new TriangulationFunction(myCalibration, camera_to_world);

    std::unique_ptr<CeresTriangulationFunction> fn1(new CeresTriangulationFunction(fn0));

    const bool ok = fn1->Evaluate( &ceres_variable_ptr, position.data(), &ceres_jacobian_ptr );

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

PFOdometry::State::State()
{
    frame_id = 0;
    timestamp = 0.0;
    num_landmarks = 0;
}

PFOdometry::Particle::Particle()
{
    weight = 1.0;
}

PFOdometry::Landmark::Landmark()
{
    last_frame_id = 0;
    circle_index_in_last_frame = 0;
}

PFOdometry::LandmarkEstimation::LandmarkEstimation()
{
    position.setZero();
    covariance.setIdentity();
}

PFOdometry::Observation::Observation()
{
    has_landmark = false;
    landmark = 0;
}

PFOdometry::LandmarkEstimation& PFOdometry::State::refLandmarkEstimation(size_t particle, size_t landmark)
{
    return landmark_estimations[particle*landmarks.size() + landmark];
}

void PFOdometry::updateLandmark(const Sophus::SE3d& camera_to_world, const cv::Vec3f& observation, LandmarkEstimation& landmark)
{
    if(landmark.available)
    {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian;

        Eigen::Vector3d predicted_observation;

        Eigen::Vector3d sensed_observation;
        sensed_observation(0) = observation(0);
        sensed_observation(1) = observation(1);
        sensed_observation(2) = observation(2);

        Eigen::Matrix3d sensed_observation_covariance;
        sensed_observation_covariance.setZero();
        sensed_observation_covariance(0,0) = myCirclePositionNoise*myCirclePositionNoise;
        sensed_observation_covariance(1,1) = myCirclePositionNoise*myCirclePositionNoise;
        sensed_observation_covariance(2,2) = myCircleRadiusNoise*myCircleRadiusNoise;

        const Eigen::Vector3d old_state = landmark.position;

        const Eigen::Matrix3d old_state_covariance = landmark.covariance;

        std::unique_ptr<CeresLandmarkObservationFunction> function;
        function.reset(new CeresLandmarkObservationFunction(new LandmarkObservationFunction(myCalibration, camera_to_world)));

        {
            const double* ceres_old_state = old_state.data();
            double* ceres_jacobian = jacobian.data();
            landmark.available = function->Evaluate(&ceres_old_state, predicted_observation.data(), &ceres_jacobian);
        }

        if(landmark.available)
        {
            const Eigen::VectorXd residual = sensed_observation - predicted_observation;

            const Eigen::MatrixXd residual_covariance = jacobian * old_state_covariance * jacobian.transpose() + sensed_observation_covariance;

            Eigen::LDLT<Eigen::Matrix3d> solver;
            solver.compute(residual_covariance);
            // TODO: check invertibility.

            const Eigen::Vector3d new_state = old_state + old_state_covariance * jacobian.transpose() * solver.solve(residual);

            const Eigen::Matrix3d new_state_covariance = old_state_covariance - old_state_covariance * jacobian.transpose() * solver.solve(jacobian * old_state_covariance);

            landmark.position = new_state;
            landmark.covariance = new_state_covariance;
        }
    }
}

