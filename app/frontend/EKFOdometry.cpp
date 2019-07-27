#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include "EKFOdometry.h"

struct EKFOdometry::TriangulationFunction
{
    EKFOdometry* mParent;

    TriangulationFunction(EKFOdometry* parent)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(const T* const circle, T* landmark) const
    {
        T cx = circle[0];
        T cy = circle[1];
        T r = circle[2];

        T invK[] =
        {
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(0,2)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(1,2)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,0)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,1)),
            T(mParent->mCalibration->cameras[0].inverse_calibration_matrix(2,2)),
        };

        T dir[3] =
        {
            cx*invK[0] + cy*invK[1] + invK[2],
            cx*invK[3] + cy*invK[4] + invK[5],
            cx*invK[6] + cy*invK[7] + invK[8]
        };

        T norm = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
        dir[0] /= norm;
        dir[1] /= norm;
        dir[2] /= norm;

        T tdir[3] =
        {
            (cx+r)*invK[0] + cy*invK[1] + invK[2],
            (cx+r)*invK[3] + cy*invK[4] + invK[5],
            (cx+r)*invK[6] + cy*invK[7] + invK[8]
        };

        T tnorm = sqrt(tdir[0]*tdir[0] + tdir[1]*tdir[1] + tdir[2]*tdir[2]);
        tdir[0] /= tnorm;
        tdir[1] /= tnorm;
        tdir[2] /= tnorm;

        T cosalpha = dir[0]*tdir[0] + dir[1]*tdir[1] + dir[2]*tdir[2];

        constexpr double threshold = std::cos(M_PI*0.3/180.0);

        if( M_PI*0.1 < cosalpha && cosalpha < threshold)
        {
            T sinalpha = sqrt( T(1.0) - cosalpha*cosalpha );

            T distance = T(mParent->mLandmarkRadius) / sinalpha;

            landmark[0] = distance*dir[0];
            landmark[1] = distance*dir[1];
            landmark[2] = distance*dir[2];

            return true;
        }
        else
        {
            return false;
        }
    }
};

struct EKFOdometry::PredictionFunction
{
    EKFOdometry* mParent;
    double mTimestep;
    size_t mNumLandmarks;

    PredictionFunction(double dt, size_t num_landmarks, EKFOdometry* parent)
    {
        mParent = parent;
        mTimestep = dt;
        mNumLandmarks = num_landmarks;
    }

    template<typename T>
    bool operator()(T const* const* old_state_arr, T* new_state) const
    {
        const T* old_state = *old_state_arr;

        T linear_velocity[3];
        T angular_velocity[3];

        linear_velocity[0] = old_state[7];
        linear_velocity[1] = old_state[8];
        linear_velocity[2] = old_state[9];

        angular_velocity[0] = old_state[10];
        angular_velocity[1] = old_state[11];
        angular_velocity[2] = old_state[12];

        // update position.

        new_state[0] = old_state[0] + mTimestep*linear_velocity[0];
        new_state[1] = old_state[1] + mTimestep*linear_velocity[1];
        new_state[2] = old_state[2] + mTimestep*linear_velocity[2];

        // update attitude.

        // TODO

        // copy linear and angular momenta.

        new_state[7] = old_state[7];
        new_state[8] = old_state[8];
        new_state[9] = old_state[9];

        new_state[10] = old_state[10];
        new_state[11] = old_state[11];
        new_state[12] = old_state[12];

        // copy landmarks.

        for(size_t i=0; i<mNumLandmarks; i++)
        {
            new_state[13+3*i+0] = old_state[13+3*i+0];
            new_state[13+3*i+1] = old_state[13+3*i+1];
            new_state[13+3*i+2] = old_state[13+3*i+2];
        }

        return true;
    }
};

struct EKFOdometry::ObservationFunction
{
    EKFOdometry* mParent;
    std::vector<ObservedLandmark>& mVisibleLandmarks;

    ObservationFunction(std::vector<ObservedLandmark>& visible_landmarks, EKFOdometry* parent) :
        mVisibleLandmarks(visible_landmarks)
    {
        mParent = parent;
    }

    template<typename T>
    bool operator()(T const* const* state_arr, T* prediction) const
    {
        const T* state = *state_arr;
        for(size_t i=0; i<mVisibleLandmarks.size(); i++)
        {
        }

        return true;
    }
};

EKFOdometry::EKFOdometry(CalibrationDataPtr calibration)
{
    mLandmarkRadius = 1.0;
    mMaxLandmarks = 330;
    mCalibration = calibration;
    mPredictionLinearMomentumSigmaRate = 1.0;
    mPredictionLinearMomentumSigmaRate = M_PI*10.0/180.0;

    mInitialized = false;

    mStates[0].reset(new State());
    mStates[1].reset(new State());

    if(mCalibration->num_cameras == 0)
    {
        std::cerr << "Empty calibration" << std::endl;
        exit(1);
    }
}

bool EKFOdometry::track(
    double timestamp,
    const std::vector<TrackedCircle>& circles,
    Sophus::SE3d& camera_to_world,
    bool& aligned_wrt_previous)
{
    bool successful_tracking = false;

    if( mInitialized && circles.empty() == false )
    {
        switchStates();
        successful_tracking = trackingPrediction(timestamp);

        if(successful_tracking)
        {
            switchStates();
            successful_tracking = trackingUpdate(circles);
        }

        if(successful_tracking)
        {
            switchStates();
            successful_tracking = mappingAugment(circles);
        }
    }

    if(successful_tracking == false)
    {
        initialize(timestamp, circles);
    }

    camera_to_world = newState().camera_to_world;
    aligned_wrt_previous = successful_tracking;

    return true;
}

void EKFOdometry::reset()
{
    mInitialized = false;
}

bool EKFOdometry::triangulateLandmark(
    const TrackedCircle& tc,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& covariance)
{
    double ceres_variable[3];
    double ceres_value[3];
    double ceres_jacobian[9];
    double* ceres_variable_ptr = ceres_variable;
    double* ceres_jacobian_ptr = ceres_jacobian;

    const cv::Vec3f undistorted = undistortCircle(tc.circle);

    ceres_variable[0] = undistorted[0];
    ceres_variable[1] = undistorted[1];
    ceres_variable[2] = undistorted[2];

    std::unique_ptr<ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>> function(new ceres::AutoDiffCostFunction<TriangulationFunction, 3, 3>(new TriangulationFunction(this)));

    const bool ok = function->Evaluate( &ceres_variable_ptr, ceres_value, &ceres_jacobian_ptr );

    if(ok)
    {
        Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> > J(ceres_jacobian);

        const double sigma_center = 1.5;
        const double sigma_radius = 1.5;

        Eigen::Matrix3d S0;
        S0 <<
            sigma_center*sigma_center, 0.0, 0.0,
            0.0, sigma_center*sigma_center, 0.0,
            0.0, 0.0, sigma_radius*sigma_radius;

        covariance = J * S0 * J.transpose();
        position.x() = ceres_value[0];
        position.y() = ceres_value[1];
        position.z() = ceres_value[2];

        /*
        std::cout << sqrt(covariance(0,0)) << std::endl;
        std::cout << sqrt(covariance(1,1)) << std::endl;
        std::cout << sqrt(covariance(2,2)) << std::endl;
        */
    }

    return ok;
}

void EKFOdometry::initialize(double timestamp, const std::vector<TrackedCircle>& circles)
{
    State& new_state = newState();

    const size_t num_circles = circles.size();

    new_state.timestamp = timestamp;
    new_state.camera_to_world = Sophus::SE3d(); // identity
    new_state.linear_momentum.setZero();
    new_state.angular_momentum.setZero();
    new_state.landmarks.clear();

    std::vector<Eigen::Vector3d> landmark_positions(num_circles);
    std::vector<Eigen::Matrix3d> landmark_covariances(num_circles);
    mCircleToLandmark.resize(num_circles);

    // triangulate circles into landmarks.

    size_t num_triangulated = 0;

    for(size_t i=0; i<num_circles; i++)
    {
        mCircleToLandmark[i].has_landmark = triangulateLandmark(
            circles[i],
            landmark_positions[i],
            landmark_covariances[i]);

        if( mCircleToLandmark[i].has_landmark )
        {
            num_triangulated++;
        }
    }

    // take only mMaxLandmarks landmarks (those with less variance).

    if( num_triangulated > mMaxLandmarks )
    {
        std::vector<size_t> sorted;
        sorted.reserve(num_triangulated);

        for(size_t i=0; i<num_circles; i++)
        {
            if(mCircleToLandmark[i].has_landmark)
            {
                sorted.push_back(i);
                mCircleToLandmark[i].has_landmark = false;
            }
        }

        auto pred = [&landmark_covariances] (size_t a, size_t b) -> bool
        {
            const double var_a = landmark_covariances[a].trace();
            const double var_b = landmark_covariances[b].trace();
            return var_a <= var_b;
        };

        std::sort(sorted.begin(), sorted.end(), pred);

        for(size_t i=0; i<mMaxLandmarks; i++)
        {
            mCircleToLandmark[ sorted[i] ].has_landmark = true;
        }
    }

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCircleToLandmark[i].has_landmark )
        {
            mCircleToLandmark[i].landmark = new_state.landmarks.size();

            new_state.landmarks.emplace_back();
            new_state.landmarks.back().position = landmark_positions[i];
            new_state.landmarks.back().seen_count = 1;
        }
        else
        {
            mCircleToLandmark[i].landmark = 0; // static_cast<size_t>(-1);
        }
    }

    const size_t num_landmarks = new_state.landmarks.size();

    new_state.covariance.resize(13+3*num_landmarks, 13+3*num_landmarks);
    new_state.covariance.setZero();

    /*
    const double initial_sigma_position = 1.0e-3;
    const double initial_sigma_attitude = 1.0e-3;
    const double initial_sigma_linear_momentum = 1.0e-3;
    const double initial_sigma_angular_momentum = 1.0e-3;

    new_state.covariance.block<3,3>(0,0).diagonal().fill(initial_sigma_position*initial_sigma_position);
    new_state.covariance.block<4,4>(3,3).diagonal().fill(initial_sigma_attitude*initial_sigma_attitude);
    new_state.covariance.block<3,3>(7,7).diagonal().fill(initial_sigma_linear_momentum*initial_sigma_linear_momentum);
    new_state.covariance.block<3,3>(10,10).diagonal().fill(initial_sigma_angular_momentum*initial_sigma_angular_momentum);
    */

    for(size_t i=0; i<num_circles; i++)
    {
        if( mCircleToLandmark[i].has_landmark )
        {
            const size_t j = mCircleToLandmark[i].landmark;
            new_state.covariance.block<3,3>(13+3*j, 13+3*j) = landmark_covariances[i];
        }
    }

    //std::cout << new_state.covariance.block<3,3>(13, 13) << std::endl;
    //std::cout << "num_landmarks: " << num_landmarks << std::endl;

    mInitialized = true;
    //std::cout << new_state.covariance << std::endl;
}

cv::Vec3f EKFOdometry::undistortCircle(const cv::Vec3f& c)
{
    std::array< cv::Vec2d, 5 > distorted;
    std::array< cv::Vec2d, 5 > undistorted;

    distorted[0][0] = c[0];
    distorted[0][1] = c[1];
    distorted[1][0] = c[0]+c[2];
    distorted[1][1] = c[1];
    distorted[2][0] = c[0]-c[2];
    distorted[2][1] = c[1];
    distorted[3][0] = c[0];
    distorted[3][1] = c[1]+c[2];
    distorted[4][0] = c[0];
    distorted[4][1] = c[1]-c[2];

    cv::undistortPoints(
        distorted,
        undistorted,
        mCalibration->cameras[0].calibration_matrix,
        mCalibration->cameras[0].distortion_coefficients,
        cv::noArray(),
        mCalibration->cameras[0].calibration_matrix);

    Eigen::Matrix<double, 5, 2, Eigen::RowMajor> eig;

    eig <<
        undistorted[0][0], undistorted[0][1],
        undistorted[1][0], undistorted[1][1],
        undistorted[2][0], undistorted[2][1],
        undistorted[3][0], undistorted[3][1],
        undistorted[4][0], undistorted[4][1];

    eig.row(1) -= eig.topRows<1>();
    eig.row(2) -= eig.topRows<1>();
    eig.row(3) -= eig.topRows<1>();
    eig.row(4) -= eig.topRows<1>();

    cv::Vec3f ret;
    ret[0] = undistorted[0][0];
    ret[1] = undistorted[0][1];
    ret[2] = ( eig.row(1).norm() + eig.row(2).norm() + eig.row(3).norm() + eig.row(4).norm() ) / 4.0;

    return ret;
}

bool EKFOdometry::mappingAugment(const std::vector<TrackedCircle>& circles)
{
    State& old_state = oldState();
    State& new_state = newState();

    // TODO

    std::vector<CircleToLandmark> new_circle_to_landmark(circles.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        if( circles[i].has_previous && mCircleToLandmark[circles[i].previous].has_landmark)
        {
            new_circle_to_landmark[i].has_landmark = true;
            new_circle_to_landmark[i].landmark = mCircleToLandmark[circles[i].previous].landmark;
        }
        else
        {
            new_circle_to_landmark[i].has_landmark = false;
            new_circle_to_landmark[i].landmark = 0;
        }
    }

    mCircleToLandmark = std::move(new_circle_to_landmark);

    return true;
}

bool EKFOdometry::trackingUpdate(const std::vector<TrackedCircle>& circles)
{
    State& old_state = oldState();
    State& new_state = newState();

    std::vector<ObservedLandmark> observed_landmarks;
    observed_landmarks.reserve(old_state.landmarks.size());

    for(size_t i=0; i<circles.size(); i++)
    {
        if(circles[i].has_previous && mCircleToLandmark[circles[i].previous].has_landmark)
        {
            observed_landmarks.emplace_back();
            observed_landmarks.back().landmark = mCircleToLandmark[circles[i].previous].landmark;
            observed_landmarks.back().undistorted_circle = undistortCircle( circles[i].circle );
        }
    }

    const size_t observation_dim = 3*observed_landmarks.size();

    Eigen::VectorXd state_vector = old_state.toVector();

    Eigen::VectorXd observation;
    observation.resize(observation_dim);

    std::unique_ptr< ceres::DynamicAutoDiffCostFunction<ObservationFunction> > function(new ceres::DynamicAutoDiffCostFunction<ObservationFunction>(new ObservationFunction(observed_landmarks, this)));
    function->AddParameterBlock(old_state.getDimension());
    function->SetNumResiduals(observation_dim);

    // TODO

    return true;
}

bool EKFOdometry::trackingPrediction(double timestamp)
{
    State& old_state = oldState();
    State& new_state = newState();

    const double timestep = timestamp - old_state.timestamp;

    const size_t dim = 13 + 3*old_state.landmarks.size();

    Eigen::VectorXd initial_state = old_state.toVector();
    Eigen::VectorXd predicted_state(old_state.getDimension());

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(dim, dim);

    std::unique_ptr< ceres::DynamicAutoDiffCostFunction<PredictionFunction> > function(new ceres::DynamicAutoDiffCostFunction<PredictionFunction>(new PredictionFunction(timestep, old_state.landmarks.size(), this)));
    function->AddParameterBlock(dim);
    function->SetNumResiduals(dim);
    const double* ceres_params = initial_state.data();
    double* ceres_derivative = jacobian.data();
    bool ok = function->Evaluate(&ceres_params, predicted_state.data(), &ceres_derivative);

    if(ok)
    {
        new_state.timestamp = timestamp;

        new_state.camera_to_world.translation() = predicted_state.segment<3>(0);

        // TODO: set new_state.camera_to_world.

        new_state.linear_momentum = old_state.linear_momentum;
        new_state.angular_momentum = old_state.angular_momentum;
        new_state.landmarks = std::move(old_state.landmarks);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> noise(dim, dim);
        noise.setZero();

        noise(7,7) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(8,8) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(9,9) = mPredictionLinearMomentumSigmaRate * mPredictionLinearMomentumSigmaRate * timestep * timestep;
        noise(10,10) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(11,11) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;
        noise(12,12) = mPredictionAngularMomentumSigmaRate * mPredictionAngularMomentumSigmaRate * timestep * timestep;

        new_state.covariance = jacobian * (old_state.covariance + noise) * jacobian.transpose();
    }

    return ok;
}

EKFOdometry::Landmark::Landmark()
{
    seen_count = 0;
}

EKFOdometry::CircleToLandmark::CircleToLandmark()
{
    has_landmark = false;
    landmark = 0;
}

EKFOdometry::State::State()
{
    timestamp = 0.0;
}

size_t EKFOdometry::State::getDimension()
{
    return 13 + 3*landmarks.size();
}

Eigen::VectorXd EKFOdometry::State::toVector()
{
    Eigen::VectorXd ret(getDimension());

    ret.segment<3>(0) = camera_to_world.translation();
    ret.segment<3>(3) = camera_to_world.unit_quaternion().vec();
    ret(6) = camera_to_world.unit_quaternion().w();
    ret.segment<3>(7) = linear_momentum;
    ret.segment<3>(10) = angular_momentum;

    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.segment<3>(13+3*i) = landmarks[i].position;
    }

    return ret;
}

