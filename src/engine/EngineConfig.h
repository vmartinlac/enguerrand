
#pragma once

#include <memory>
#include <QMetaType>
#include "ObservationValidator.h"
#include "VideoSource.h"
#include "OdometryCode.h"
#include "CalibrationData.h"

class EngineConfig
{
public:

    EngineConfig();

    ObservationValidatorPtr observation_validator;
    VideoSourcePtr video_input;
    OdometryCodePtr odometry_code;
    CalibrationDataPtr calibration;

public:

    bool loadFromFile(const std::string& path);

    void clear();
};

using EngineConfigPtr = std::shared_ptr<EngineConfig>;

Q_DECLARE_METATYPE(EngineConfigPtr)

