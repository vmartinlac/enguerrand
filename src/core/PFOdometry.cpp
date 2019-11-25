#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <sophus/average.hpp>
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
    LandmarkObservationFunction(CalibrationDataPtr calibration, const Sophus::SE3d& camera_to_world = Sophus::SE3d())
    {
        myCalibration = calibration;
        myCameraToWorld = camera_to_world;
    }

    void setCameraToWorld(const Sophus::SE3d& camera_to_world)
    {
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
    const bool successful_tracking =
        bool(myCurrentState) &&
        bool(myWorkingState) &&
        predictionStep(timestamp, circles) &&
        landmarkUpdateStep() &&
        resamplingStep() &&
        mappingStep();

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    exportCurrentState(output, successful_tracking);

    return true;
}

void PFOdometry::exportCurrentState(OdometryFrame& output, bool aligned_wrt_previous)
{
    State& s = *myCurrentState;

    output.timestamp = s.timestamp;

    output.aligned_wrt_previous = aligned_wrt_previous;

    {
        std::vector<Sophus::SE3d> camera_to_world(myNumParticles);

        for(size_t i=0; i<myNumParticles; i++)
        {
            camera_to_world[i] = s.particles[i].camera_to_world;
        }

        //std::optional<Sophus::SE3d> tmp = Sophus::average(camera_to_world);
        //output.camera_to_world = Sophus::average(camera_to_world);

        // TODO
    }

    {
        Eigen::Vector3d sum;

        for(size_t i=0; i<s.landmark_estimations.size(1); i++)
        {
            sum.setZero();

        }
    }
    //output.landmarks.resize(s.landmark_estimations.size(1));
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

        myCurrentState->observations.swap(observations);
        myCurrentState->particles.assign(myNumParticles, prototype_particle);
        myCurrentState->landmark_estimations.resize({myNumParticles, prototype_landmark_estimations.size()});

        for(size_t i=0; i<myNumParticles; i++)
        {
            for(size_t j=0; j<prototype_landmark_estimations.size(); j++)
            {
                myCurrentState->landmark_estimations({i,j}) = prototype_landmark_estimations[j];
            }
        }
    }
}

bool PFOdometry::predictionStep(double timestamp, const std::vector<TrackedCircle>& circles)
{
    // sample predicted pose. This reduces to add noise to current pose.

    myWorkingState->frame_id = myCurrentState->frame_id+1;
    myWorkingState->timestamp = timestamp;
    myWorkingState->observations.resize(circles.size());

    size_t num_landmarks = 0;
    for(size_t i=0; i<circles.size(); i++)
    {
        myWorkingState->observations[i].circle = circles[i].circle;

        myWorkingState->observations[i].has_landmark = (circles[i].has_previous && myCurrentState->observations[circles[i].previous].has_landmark);

        if( myWorkingState->observations[i].has_landmark )
        {
            myWorkingState->observations[i].landmark = num_landmarks;
            num_landmarks++;
        }
    }

    myWorkingState->landmark_estimations.resize({myNumParticles, num_landmarks});

    for(size_t i=0; i<circles.size(); i++)
    {
        if( myWorkingState->observations[i].has_landmark )
        {
            if( circles[i].has_previous == false || myCurrentState->observations[circles[i].previous].has_landmark == false )
            {
                throw std::runtime_error("internal error!");
            }

            const size_t old_landmark_index = myCurrentState->observations[circles[i].previous].landmark;

            const size_t new_landmark_index = myWorkingState->observations[i].landmark;

            for(size_t j=0; j<myNumParticles; j++)
            {
                myWorkingState->landmark_estimations({j, new_landmark_index}) = myCurrentState->landmark_estimations({j, old_landmark_index});
            }
        }
    }

    myWorkingState->particles.swap(myCurrentState->particles);

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

    return true;
}

bool PFOdometry::landmarkUpdateStep()
{
    // update landmarks.

    for(size_t i=0; i<myCurrentState->observations.size(); i++)
    {
        if( myCurrentState->observations[i].has_landmark )
        {
            const size_t k = myCurrentState->observations[i].landmark;

            for(size_t j=0; j<myCurrentState->particles.size(); j++)
            {
                updateLandmark(
                    myCurrentState->particles[j].camera_to_world,
                    myCurrentState->observations[i].circle,
                    myCurrentState->landmark_estimations({j,k}));
            }
        }
    }

    return true;
}

bool PFOdometry::resamplingStep()
{
    Eigen::Matrix3d observation_covariance;
    observation_covariance <<
        myCirclePositionNoise, 0.0, 0.0,
        0.0, myCirclePositionNoise, 0.0,
        0.0, 0.0, myCircleRadiusNoise;

    std::unique_ptr<CeresLandmarkObservationFunction> function;
    function.reset(new CeresLandmarkObservationFunction(new LandmarkObservationFunction(myCalibration)));

    std::vector<double> weights(myNumParticles);

    std::fill(weights.begin(), weights.end(), 1.0);

    for(size_t observation_index=0; observation_index<myCurrentState->observations.size(); observation_index++)
    {
        if(myCurrentState->observations[observation_index].has_landmark)
        {
            const size_t landmark_index = myCurrentState->observations[observation_index].landmark;

            for(size_t particle_index=0; particle_index<myNumParticles; particle_index++)
            {
                LandmarkEstimation& landmark_estimation = myCurrentState->landmark_estimations({particle_index, landmark_index});

                Eigen::Matrix<double, 3, 3, Eigen::RowMajor> jacobian;

                Eigen::Vector3d predicted_observation;

                const double* ceres_landmark = landmark_estimation.position.data();
                double* ceres_jacobian = jacobian.data();
                const bool ok = function->Evaluate(&ceres_landmark, predicted_observation.data(), &ceres_jacobian);

                if(ok)
                {
                    const Eigen::Matrix3d covariance = observation_covariance + jacobian * landmark_estimation.covariance * jacobian.transpose();

                    const double det_covariance = covariance.determinant();

                    if( det_covariance > 1.0e-5 )
                    {
                        Eigen::Vector3d sensed_observation;
                        sensed_observation.x() = myCurrentState->observations[observation_index].circle[0];
                        sensed_observation.y() = myCurrentState->observations[observation_index].circle[1];
                        sensed_observation.z() = myCurrentState->observations[observation_index].circle[2];

                        const Eigen::Vector3d error = predicted_observation - sensed_observation;

                        constexpr const double cte = 1.0 / std::pow(2.0*M_PI, 3.0/2.0);

                        const double landmark_weight = cte * std::exp(-0.5 * error.transpose() * covariance * error) / std::sqrt(det_covariance);

                        weights[particle_index] *= landmark_weight;
                    }
                    else
                    {
                        weights[particle_index] *= 0.0;
                    }
                }
                else
                {
                    weights[particle_index] *= 0.0;
                }
            }
        }
    }

    // resample.

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

    return true;
}

bool PFOdometry::mappingStep()
{
    // Remove unseen landmarks and triangulate new ones.

    enum LandmarkStock
    {
        LANDMARKSTOCK_OLD,
        LANDMARKSTOCK_NEW
    };

    struct LandmarkLookup
    {
        LandmarkStock stock;
        size_t index; // references a landmark in myCurrentState or an observation in myCurrentState.
    };

    std::vector<LandmarkLookup> landmarks;
    bool need_update = false;
    bool ret = true;

    myWorkingState->frame_id = myCurrentState->frame_id;
    myWorkingState->timestamp = myCurrentState->timestamp;
    myWorkingState->observations.resize(myCurrentState->observations.size());

    for(size_t i=0; i<myCurrentState->observations.size(); i++)
    {
        Eigen::Vector3d unused0;
        Eigen::Matrix3d unused1;

        myWorkingState->observations[i].circle = myCurrentState->observations[i].circle;
        myWorkingState->observations[i].has_landmark = false;
        myWorkingState->observations[i].landmark = 0;

        if( myCurrentState->observations[i].has_landmark)
        {
            myWorkingState->observations[i].has_landmark = true;
            myWorkingState->observations[i].landmark = landmarks.size();

            landmarks.emplace_back();
            landmarks.back().stock = LANDMARKSTOCK_OLD;
            landmarks.back().index = myCurrentState->observations[i].landmark;
        }
        else if(triangulateLandmark(myCurrentState->observations[i].circle, Sophus::SE3d(), unused0, unused1))
        {
            myWorkingState->observations[i].has_landmark = true;
            myWorkingState->observations[i].landmark = landmarks.size();

            landmarks.emplace_back();
            landmarks.back().stock = LANDMARKSTOCK_NEW;
            landmarks.back().index = i;

            need_update = true;
        }
    }

    if(need_update)
    {
        myWorkingState->landmark_estimations.resize({myNumParticles, landmarks.size()});

        bool all_landmarks_successfully_triangulated = true;

        for(size_t i=0; all_landmarks_successfully_triangulated && i<myNumParticles; i++)
        {
            for(size_t j=0; all_landmarks_successfully_triangulated && j<landmarks.size(); j++)
            {
                if( landmarks[j].stock == LANDMARKSTOCK_OLD )
                {
                    myWorkingState->landmark_estimations({i, j}) = myCurrentState->landmark_estimations({i, landmarks[j].index});
                }
                else if( landmarks[j].stock == LANDMARKSTOCK_NEW )
                {
                    LandmarkEstimation& est = myWorkingState->landmark_estimations({i, j});

                    all_landmarks_successfully_triangulated =
                        all_landmarks_successfully_triangulated &&
                        triangulateLandmark(
                            myCurrentState->observations[landmarks[j].index].circle,
                            myCurrentState->particles[i].camera_to_world,
                            est.position,
                            est.covariance);
                }
                else
                {
                    std::cerr << "Internal error!" << std::endl;
                    exit(1);
                }
            }
        }

        if(all_landmarks_successfully_triangulated)
        {
            // This should not have happen because we have already successfully triangulared them using identity camera pose.
            // There should be no reason why triangulating them using another camera pose would fail.
            std::cerr << "Unexpected stuff in particle filter!" << std::endl;
            //ret = false;
        }
        else
        {
            myWorkingState->particles.swap(myCurrentState->particles);

            std::swap(myWorkingState, myCurrentState);
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
}

PFOdometry::Particle::Particle()
{
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

bool PFOdometry::updateLandmark(const Sophus::SE3d& camera_to_world, const cv::Vec3f& observation, LandmarkEstimation& landmark)
{
    bool ret = false;

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
        ret = function->Evaluate(&ceres_old_state, predicted_observation.data(), &ceres_jacobian);
    }

    if(ret)
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

    return ret;
}

