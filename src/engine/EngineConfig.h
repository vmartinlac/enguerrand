
#pragma once

#include <memory>
#include <QMetaType>
#include "ObservationValidator.h"
#include "VideoSource.h"
#include "OdometryCode.h"
#include "CalibrationData.h"
#include "EdgeDetection.h"

class EngineConfig
{
public:

    EngineConfig() = default;

    ObservationValidatorPtr observation_validator;
    VideoSourcePtr video_input;
    OdometryCodePtr odometry_code;
    EdgeDetectionPtr edge_detector;
    CalibrationDataPtr calibration;
};

using EngineConfigPtr = std::shared_ptr<EngineConfig>;

Q_DECLARE_METATYPE(EngineConfigPtr)

