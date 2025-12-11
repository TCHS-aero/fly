[Original Source](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry_server.html)

# TelemetryServer[¶](#telemetryserver "Link to this heading")

*class* mavsdk.telemetry\_server.AccelerationFrd(*forward\_m\_s2*, *right\_m\_s2*, *down\_m\_s2*)[¶](#mavsdk.telemetry_server.AccelerationFrd "Link to this definition")
:   Bases: `object`

    AccelerationFrd message type.

    Parameters:
    :   * **forward\_m\_s2** (*float*) – Acceleration in forward direction in metres per second^2
        * **right\_m\_s2** (*float*) – Acceleration in right direction in metres per second^2
        * **down\_m\_s2** (*float*) – Acceleration in down direction in metres per second^2

*class* mavsdk.telemetry\_server.ActuatorControlTarget(*group*, *controls*)[¶](#mavsdk.telemetry_server.ActuatorControlTarget "Link to this definition")
:   Bases: `object`

    Actuator control target type.

    Parameters:
    :   * **group** (*int32\_t*) – An actuator control group is e.g. ‘attitude’ for the core flight controls, or ‘gimbal’ for a payload.
        * **controls** (*[**float**]*) – Controls normed from -1 to 1, where 0 is neutral position.

*class* mavsdk.telemetry\_server.ActuatorOutputStatus(*active*, *actuator*)[¶](#mavsdk.telemetry_server.ActuatorOutputStatus "Link to this definition")
:   Bases: `object`

    Actuator output status type.

    Parameters:
    :   * **active** (*uint32\_t*) – Active outputs
        * **actuator** (*[**float**]*) – Servo/motor output values

*class* mavsdk.telemetry\_server.AngularVelocityBody(*roll\_rad\_s*, *pitch\_rad\_s*, *yaw\_rad\_s*)[¶](#mavsdk.telemetry_server.AngularVelocityBody "Link to this definition")
:   Bases: `object`

    Angular velocity type.

    Parameters:
    :   * **roll\_rad\_s** (*float*) – Roll angular velocity
        * **pitch\_rad\_s** (*float*) – Pitch angular velocity
        * **yaw\_rad\_s** (*float*) – Yaw angular velocity

*class* mavsdk.telemetry\_server.AngularVelocityFrd(*forward\_rad\_s*, *right\_rad\_s*, *down\_rad\_s*)[¶](#mavsdk.telemetry_server.AngularVelocityFrd "Link to this definition")
:   Bases: `object`

    AngularVelocityFrd message type.

    Parameters:
    :   * **forward\_rad\_s** (*float*) – Angular velocity in forward direction in radians per second
        * **right\_rad\_s** (*float*) – Angular velocity in right direction in radians per second
        * **down\_rad\_s** (*float*) – Angular velocity in Down direction in radians per second

*class* mavsdk.telemetry\_server.Battery(*voltage\_v*, *remaining\_percent*)[¶](#mavsdk.telemetry_server.Battery "Link to this definition")
:   Bases: `object`

    Battery type.

    Parameters:
    :   * **voltage\_v** (*float*) – Voltage in volts
        * **remaining\_percent** (*float*) – Estimated battery remaining (range: 0.0 to 1.0)

*class* mavsdk.telemetry\_server.Covariance(*covariance\_matrix*)[¶](#mavsdk.telemetry_server.Covariance "Link to this definition")
:   Bases: `object`

    Covariance type.

    Row-major representation of a 6x6 cross-covariance matrix
    upper right triangle.
    Set first to NaN if unknown.

    Parameters:
    :   **covariance\_matrix** (*[**float**]*) – Representation of a covariance matrix.

*class* mavsdk.telemetry\_server.DistanceSensor(*minimum\_distance\_m*, *maximum\_distance\_m*, *current\_distance\_m*)[¶](#mavsdk.telemetry_server.DistanceSensor "Link to this definition")
:   Bases: `object`

    DistanceSensor message type.

    Parameters:
    :   * **minimum\_distance\_m** (*float*) – Minimum distance the sensor can measure, NaN if unknown.
        * **maximum\_distance\_m** (*float*) – Maximum distance the sensor can measure, NaN if unknown.
        * **current\_distance\_m** (*float*) – Current distance reading, NaN if unknown.

*class* mavsdk.telemetry\_server.EulerAngle(*roll\_deg*, *pitch\_deg*, *yaw\_deg*, *timestamp\_us*)[¶](#mavsdk.telemetry_server.EulerAngle "Link to this definition")
:   Bases: `object`

    Euler angle type.

    All rotations and axis systems follow the right-hand rule.
    The Euler angles follow the convention of a 3-2-1 intrinsic Tait-Bryan rotation sequence.

    For more info see <https://en.wikipedia.org/wiki/Euler_angles>

    Parameters:
    :   * **roll\_deg** (*float*) – Roll angle in degrees, positive is banking to the right
        * **pitch\_deg** (*float*) – Pitch angle in degrees, positive is pitching nose up
        * **yaw\_deg** (*float*) – Yaw angle in degrees, positive is clock-wise seen from above
        * **timestamp\_us** (*uint64\_t*) – Timestamp in microseconds

*class* mavsdk.telemetry\_server.FixType(*value*)[¶](#mavsdk.telemetry_server.FixType "Link to this definition")
:   Bases: `Enum`

    GPS fix type.

    ## Values[¶](#values "Link to this heading")

    NO\_GPS
    :   No GPS connected

    NO\_FIX
    :   No position information, GPS is connected

    FIX\_2D
    :   2D position

    FIX\_3D
    :   3D position

    FIX\_DGPS
    :   DGPS/SBAS aided 3D position

    RTK\_FLOAT
    :   RTK float, 3D position

    RTK\_FIXED
    :   RTK Fixed, 3D position

    FIX\_2D *= 2*[¶](#mavsdk.telemetry_server.FixType.FIX_2D "Link to this definition")

    FIX\_3D *= 3*[¶](#mavsdk.telemetry_server.FixType.FIX_3D "Link to this definition")

    FIX\_DGPS *= 4*[¶](#mavsdk.telemetry_server.FixType.FIX_DGPS "Link to this definition")

    NO\_FIX *= 1*[¶](#mavsdk.telemetry_server.FixType.NO_FIX "Link to this definition")

    NO\_GPS *= 0*[¶](#mavsdk.telemetry_server.FixType.NO_GPS "Link to this definition")

    RTK\_FIXED *= 6*[¶](#mavsdk.telemetry_server.FixType.RTK_FIXED "Link to this definition")

    RTK\_FLOAT *= 5*[¶](#mavsdk.telemetry_server.FixType.RTK_FLOAT "Link to this definition")

*class* mavsdk.telemetry\_server.FixedwingMetrics(*airspeed\_m\_s*, *throttle\_percentage*, *climb\_rate\_m\_s*, *groundspeed\_m\_s*, *heading\_deg*, *absolute\_altitude\_m*)[¶](#mavsdk.telemetry_server.FixedwingMetrics "Link to this definition")
:   Bases: `object`

    FixedwingMetrics message type.

    Parameters:
    :   * **airspeed\_m\_s** (*float*) – Current indicated airspeed (IAS) in metres per second
        * **throttle\_percentage** (*float*) – Current throttle setting (0 to 100)
        * **climb\_rate\_m\_s** (*float*) – Current climb rate in metres per second
        * **groundspeed\_m\_s** (*float*) – Current groundspeed metres per second
        * **heading\_deg** (*float*) – Current heading in compass units (0-360, 0=north)
        * **absolute\_altitude\_m** (*float*) – Current altitude in metres (MSL)

*class* mavsdk.telemetry\_server.GpsInfo(*num\_satellites*, *fix\_type*)[¶](#mavsdk.telemetry_server.GpsInfo "Link to this definition")
:   Bases: `object`

    GPS information type.

    Parameters:
    :   * **num\_satellites** (*int32\_t*) – Number of visible satellites in use
        * **fix\_type** ([*FixType*](#mavsdk.telemetry_server.FixType "mavsdk.telemetry_server.FixType")) – Fix type

*class* mavsdk.telemetry\_server.GroundTruth(*latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*)[¶](#mavsdk.telemetry_server.GroundTruth "Link to this definition")
:   Bases: `object`

    GroundTruth message type.

    Parameters:
    :   * **latitude\_deg** (*double*) – Latitude in degrees (range: -90 to +90)
        * **longitude\_deg** (*double*) – Longitude in degrees (range: -180 to 180)
        * **absolute\_altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres

*class* mavsdk.telemetry\_server.Heading(*heading\_deg*)[¶](#mavsdk.telemetry_server.Heading "Link to this definition")
:   Bases: `object`

    Heading type used for global position

    Parameters:
    :   **heading\_deg** (*double*) – Heading in degrees (range: 0 to +360)

*class* mavsdk.telemetry\_server.Imu(*acceleration\_frd*, *angular\_velocity\_frd*, *magnetic\_field\_frd*, *temperature\_degc*, *timestamp\_us*)[¶](#mavsdk.telemetry_server.Imu "Link to this definition")
:   Bases: `object`

    Imu message type.

    Parameters:
    :   * **acceleration\_frd** ([*AccelerationFrd*](#mavsdk.telemetry_server.AccelerationFrd "mavsdk.telemetry_server.AccelerationFrd")) – Acceleration
        * **angular\_velocity\_frd** ([*AngularVelocityFrd*](#mavsdk.telemetry_server.AngularVelocityFrd "mavsdk.telemetry_server.AngularVelocityFrd")) – Angular velocity
        * **magnetic\_field\_frd** ([*MagneticFieldFrd*](#mavsdk.telemetry_server.MagneticFieldFrd "mavsdk.telemetry_server.MagneticFieldFrd")) – Magnetic field
        * **temperature\_degc** (*float*) – Temperature
        * **timestamp\_us** (*uint64\_t*) – Timestamp in microseconds

*class* mavsdk.telemetry\_server.LandedState(*value*)[¶](#mavsdk.telemetry_server.LandedState "Link to this definition")
:   Bases: `Enum`

    Landed State enumeration.

    ## Values[¶](#id1 "Link to this heading")

    UNKNOWN
    :   Landed state is unknown

    ON\_GROUND
    :   The vehicle is on the ground

    IN\_AIR
    :   The vehicle is in the air

    TAKING\_OFF
    :   The vehicle is taking off

    LANDING
    :   The vehicle is landing

    IN\_AIR *= 2*[¶](#mavsdk.telemetry_server.LandedState.IN_AIR "Link to this definition")

    LANDING *= 4*[¶](#mavsdk.telemetry_server.LandedState.LANDING "Link to this definition")

    ON\_GROUND *= 1*[¶](#mavsdk.telemetry_server.LandedState.ON_GROUND "Link to this definition")

    TAKING\_OFF *= 3*[¶](#mavsdk.telemetry_server.LandedState.TAKING_OFF "Link to this definition")

    UNKNOWN *= 0*[¶](#mavsdk.telemetry_server.LandedState.UNKNOWN "Link to this definition")

*class* mavsdk.telemetry\_server.MagneticFieldFrd(*forward\_gauss*, *right\_gauss*, *down\_gauss*)[¶](#mavsdk.telemetry_server.MagneticFieldFrd "Link to this definition")
:   Bases: `object`

    MagneticFieldFrd message type.

    Parameters:
    :   * **forward\_gauss** (*float*) – Magnetic field in forward direction measured in Gauss
        * **right\_gauss** (*float*) – Magnetic field in East direction measured in Gauss
        * **down\_gauss** (*float*) – Magnetic field in Down direction measured in Gauss

*class* mavsdk.telemetry\_server.Odometry(*time\_usec*, *frame\_id*, *child\_frame\_id*, *position\_body*, *q*, *velocity\_body*, *angular\_velocity\_body*, *pose\_covariance*, *velocity\_covariance*)[¶](#mavsdk.telemetry_server.Odometry "Link to this definition")
:   Bases: `object`

    Odometry message type.

    Parameters:
    :   * **time\_usec** (*uint64\_t*) – Timestamp (0 to use Backend timestamp).
        * **frame\_id** ([*MavFrame*](#mavsdk.telemetry_server.Odometry.MavFrame "mavsdk.telemetry_server.Odometry.MavFrame")) – Coordinate frame of reference for the pose data.
        * **child\_frame\_id** ([*MavFrame*](#mavsdk.telemetry_server.Odometry.MavFrame "mavsdk.telemetry_server.Odometry.MavFrame")) – Coordinate frame of reference for the velocity in free space (twist) data.
        * **position\_body** ([*PositionBody*](#mavsdk.telemetry_server.PositionBody "mavsdk.telemetry_server.PositionBody")) – Position.
        * **q** ([*Quaternion*](#mavsdk.telemetry_server.Quaternion "mavsdk.telemetry_server.Quaternion")) – Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation).
        * **velocity\_body** ([*VelocityBody*](#mavsdk.telemetry_server.VelocityBody "mavsdk.telemetry_server.VelocityBody")) – Linear velocity (m/s).
        * **angular\_velocity\_body** ([*AngularVelocityBody*](#mavsdk.telemetry_server.AngularVelocityBody "mavsdk.telemetry_server.AngularVelocityBody")) – Angular velocity (rad/s).
        * **pose\_covariance** ([*Covariance*](#mavsdk.telemetry_server.Covariance "mavsdk.telemetry_server.Covariance")) – Pose cross-covariance matrix.
        * **velocity\_covariance** ([*Covariance*](#mavsdk.telemetry_server.Covariance "mavsdk.telemetry_server.Covariance")) – Velocity cross-covariance matrix.

    *class* MavFrame(*value*)[¶](#mavsdk.telemetry_server.Odometry.MavFrame "Link to this definition")
    :   Bases: `Enum`

        Mavlink frame id

        ## Values[¶](#id2 "Link to this heading")

        UNDEF
        :   Frame is undefined.

        BODY\_NED
        :   Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.

        VISION\_NED
        :   Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).

        ESTIM\_NED
        :   Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).

        BODY\_NED *= 1*[¶](#mavsdk.telemetry_server.Odometry.MavFrame.BODY_NED "Link to this definition")

        ESTIM\_NED *= 3*[¶](#mavsdk.telemetry_server.Odometry.MavFrame.ESTIM_NED "Link to this definition")

        UNDEF *= 0*[¶](#mavsdk.telemetry_server.Odometry.MavFrame.UNDEF "Link to this definition")

        VISION\_NED *= 2*[¶](#mavsdk.telemetry_server.Odometry.MavFrame.VISION_NED "Link to this definition")

*class* mavsdk.telemetry\_server.Position(*latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*, *relative\_altitude\_m*)[¶](#mavsdk.telemetry_server.Position "Link to this definition")
:   Bases: `object`

    Position type in global coordinates.

    Parameters:
    :   * **latitude\_deg** (*double*) – Latitude in degrees (range: -90 to +90)
        * **longitude\_deg** (*double*) – Longitude in degrees (range: -180 to +180)
        * **absolute\_altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres
        * **relative\_altitude\_m** (*float*) – Altitude relative to takeoff altitude in metres

*class* mavsdk.telemetry\_server.PositionBody(*x\_m*, *y\_m*, *z\_m*)[¶](#mavsdk.telemetry_server.PositionBody "Link to this definition")
:   Bases: `object`

    Position type, represented in the Body (X Y Z) frame

    Parameters:
    :   * **x\_m** (*float*) – X Position in metres.
        * **y\_m** (*float*) – Y Position in metres.
        * **z\_m** (*float*) – Z Position in metres.

*class* mavsdk.telemetry\_server.PositionNed(*north\_m*, *east\_m*, *down\_m*)[¶](#mavsdk.telemetry_server.PositionNed "Link to this definition")
:   Bases: `object`

    PositionNed message type.

    Parameters:
    :   * **north\_m** (*float*) – Position along north direction in metres
        * **east\_m** (*float*) – Position along east direction in metres
        * **down\_m** (*float*) – Position along down direction in metres

*class* mavsdk.telemetry\_server.PositionVelocityNed(*position*, *velocity*)[¶](#mavsdk.telemetry_server.PositionVelocityNed "Link to this definition")
:   Bases: `object`

    PositionVelocityNed message type.

    Parameters:
    :   * **position** ([*PositionNed*](#mavsdk.telemetry_server.PositionNed "mavsdk.telemetry_server.PositionNed")) – Position (NED)
        * **velocity** ([*VelocityNed*](#mavsdk.telemetry_server.VelocityNed "mavsdk.telemetry_server.VelocityNed")) – Velocity (NED)

*class* mavsdk.telemetry\_server.Quaternion(*w*, *x*, *y*, *z*, *timestamp\_us*)[¶](#mavsdk.telemetry_server.Quaternion "Link to this definition")
:   Bases: `object`

    Quaternion type.

    All rotations and axis systems follow the right-hand rule.
    The Hamilton quaternion product definition is used.
    A zero-rotation quaternion is represented by (1,0,0,0).
    The quaternion could also be written as w + xi + yj + zk.

    For more info see: <https://en.wikipedia.org/wiki/Quaternion>

    Parameters:
    :   * **w** (*float*) – Quaternion entry 0, also denoted as a
        * **x** (*float*) – Quaternion entry 1, also denoted as b
        * **y** (*float*) – Quaternion entry 2, also denoted as c
        * **z** (*float*) – Quaternion entry 3, also denoted as d
        * **timestamp\_us** (*uint64\_t*) – Timestamp in microseconds

*class* mavsdk.telemetry\_server.RawGps(*timestamp\_us*, *latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*, *hdop*, *vdop*, *velocity\_m\_s*, *cog\_deg*, *altitude\_ellipsoid\_m*, *horizontal\_uncertainty\_m*, *vertical\_uncertainty\_m*, *velocity\_uncertainty\_m\_s*, *heading\_uncertainty\_deg*, *yaw\_deg*)[¶](#mavsdk.telemetry_server.RawGps "Link to this definition")
:   Bases: `object`

    Raw GPS information type.

    Warning: this is an advanced type! If you want the location of the drone, use
    the position instead. This message exposes the raw values of the GNSS sensor.

    Parameters:
    :   * **timestamp\_us** (*uint64\_t*) – Timestamp in microseconds (UNIX Epoch time or time since system boot, to be inferred)
        * **latitude\_deg** (*double*) – Latitude in degrees (WGS84, EGM96 ellipsoid)
        * **longitude\_deg** (*double*) – Longitude in degrees (WGS84, EGM96 ellipsoid)
        * **absolute\_altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres
        * **hdop** (*float*) – GPS HDOP horizontal dilution of position (unitless). If unknown, set to NaN
        * **vdop** (*float*) – GPS VDOP vertical dilution of position (unitless). If unknown, set to NaN
        * **velocity\_m\_s** (*float*) – Ground velocity in metres per second
        * **cog\_deg** (*float*) – Course over ground (NOT heading, but direction of movement) in degrees. If unknown, set to NaN
        * **altitude\_ellipsoid\_m** (*float*) – Altitude in metres (above WGS84, EGM96 ellipsoid)
        * **horizontal\_uncertainty\_m** (*float*) – Position uncertainty in metres
        * **vertical\_uncertainty\_m** (*float*) – Altitude uncertainty in metres
        * **velocity\_uncertainty\_m\_s** (*float*) – Velocity uncertainty in metres per second
        * **heading\_uncertainty\_deg** (*float*) – Heading uncertainty in degrees
        * **yaw\_deg** (*float*) – Yaw in earth frame from north.

*class* mavsdk.telemetry\_server.RcStatus(*was\_available\_once*, *is\_available*, *signal\_strength\_percent*)[¶](#mavsdk.telemetry_server.RcStatus "Link to this definition")
:   Bases: `object`

    Remote control status type.

    Parameters:
    :   * **was\_available\_once** (*bool*) – True if an RC signal has been available once
        * **is\_available** (*bool*) – True if the RC signal is available now
        * **signal\_strength\_percent** (*float*) – Signal strength (range: 0 to 100, NaN if unknown)

*class* mavsdk.telemetry\_server.ScaledPressure(*timestamp\_us*, *absolute\_pressure\_hpa*, *differential\_pressure\_hpa*, *temperature\_deg*, *differential\_pressure\_temperature\_deg*)[¶](#mavsdk.telemetry_server.ScaledPressure "Link to this definition")
:   Bases: `object`

    Scaled Pressure message type.

    Parameters:
    :   * **timestamp\_us** (*uint64\_t*) – Timestamp (time since system boot)
        * **absolute\_pressure\_hpa** (*float*) – Absolute pressure in hPa
        * **differential\_pressure\_hpa** (*float*) – Differential pressure 1 in hPa
        * **temperature\_deg** (*float*) – Absolute pressure temperature (in celsius)
        * **differential\_pressure\_temperature\_deg** (*float*) – Differential pressure temperature (in celsius, 0 if not available)

*class* mavsdk.telemetry\_server.StatusText(*type*, *text*)[¶](#mavsdk.telemetry_server.StatusText "Link to this definition")
:   Bases: `object`

    StatusText information type.

    Parameters:
    :   * **type** ([*StatusTextType*](#mavsdk.telemetry_server.StatusTextType "mavsdk.telemetry_server.StatusTextType")) – Message type
        * **text** (*std::string*) – MAVLink status message

*class* mavsdk.telemetry\_server.StatusTextType(*value*)[¶](#mavsdk.telemetry_server.StatusTextType "Link to this definition")
:   Bases: `Enum`

    Status types.

    ## Values[¶](#id3 "Link to this heading")

    DEBUG
    :   Debug

    INFO
    :   Information

    NOTICE
    :   Notice

    WARNING
    :   Warning

    ERROR
    :   Error

    CRITICAL
    :   Critical

    ALERT
    :   Alert

    EMERGENCY
    :   Emergency

    ALERT *= 6*[¶](#mavsdk.telemetry_server.StatusTextType.ALERT "Link to this definition")

    CRITICAL *= 5*[¶](#mavsdk.telemetry_server.StatusTextType.CRITICAL "Link to this definition")

    DEBUG *= 0*[¶](#mavsdk.telemetry_server.StatusTextType.DEBUG "Link to this definition")

    EMERGENCY *= 7*[¶](#mavsdk.telemetry_server.StatusTextType.EMERGENCY "Link to this definition")

    ERROR *= 4*[¶](#mavsdk.telemetry_server.StatusTextType.ERROR "Link to this definition")

    INFO *= 1*[¶](#mavsdk.telemetry_server.StatusTextType.INFO "Link to this definition")

    NOTICE *= 2*[¶](#mavsdk.telemetry_server.StatusTextType.NOTICE "Link to this definition")

    WARNING *= 3*[¶](#mavsdk.telemetry_server.StatusTextType.WARNING "Link to this definition")

*class* mavsdk.telemetry\_server.TelemetryServer(*async\_plugin\_manager*)[¶](#mavsdk.telemetry_server.TelemetryServer "Link to this definition")
:   Bases: `AsyncBase`

    Allow users to provide vehicle telemetry and state information
    (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates.

    Generated by dcsdkgen - MAVSDK TelemetryServer API

    name *= 'TelemetryServer'*[¶](#mavsdk.telemetry_server.TelemetryServer.name "Link to this definition")

    *async* publish\_attitude(*angle*, *angular\_velocity*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_attitude "Link to this definition")
    :   Publish to “attitude” updates.

        Parameters:
        :   * **angle** ([*EulerAngle*](#mavsdk.telemetry_server.EulerAngle "mavsdk.telemetry_server.EulerAngle")) – roll/pitch/yaw body angles
            * **angular\_velocity** ([*AngularVelocityBody*](#mavsdk.telemetry_server.AngularVelocityBody "mavsdk.telemetry_server.AngularVelocityBody")) – roll/pitch/yaw angular velocities

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_battery(*battery*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_battery "Link to this definition")
    :   Publish to ‘battery’ updates.

        Parameters:
        :   **battery** ([*Battery*](#mavsdk.telemetry_server.Battery "mavsdk.telemetry_server.Battery")) – The next ‘battery’ state

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_distance\_sensor(*distance\_sensor*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_distance_sensor "Link to this definition")
    :   Publish to “distance sensor” updates.

        Parameters:
        :   **distance\_sensor** ([*DistanceSensor*](#mavsdk.telemetry_server.DistanceSensor "mavsdk.telemetry_server.DistanceSensor")) – The next ‘Distance Sensor’ status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_extended\_sys\_state(*vtol\_state*, *landed\_state*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_extended_sys_state "Link to this definition")
    :   Publish ‘extended sys state’ updates.

        Parameters:
        :   * **vtol\_state** ([*VtolState*](#mavsdk.telemetry_server.VtolState "mavsdk.telemetry_server.VtolState"))
            * **landed\_state** ([*LandedState*](#mavsdk.telemetry_server.LandedState "mavsdk.telemetry_server.LandedState"))

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_ground\_truth(*ground\_truth*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_ground_truth "Link to this definition")
    :   Publish to ‘ground truth’ updates.

        Parameters:
        :   **ground\_truth** ([*GroundTruth*](#mavsdk.telemetry_server.GroundTruth "mavsdk.telemetry_server.GroundTruth")) – Ground truth position information available in simulation

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_home(*home*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_home "Link to this definition")
    :   Publish to ‘home position’ updates.

        Parameters:
        :   **home** ([*Position*](#mavsdk.telemetry_server.Position "mavsdk.telemetry_server.Position")) – The next home position

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_imu(*imu*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_imu "Link to this definition")
    :   Publish to ‘IMU’ updates (in SI units in NED body frame).

        Parameters:
        :   **imu** ([*Imu*](#mavsdk.telemetry_server.Imu "mavsdk.telemetry_server.Imu")) – The next IMU status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_odometry(*odometry*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_odometry "Link to this definition")
    :   Publish to ‘odometry’ updates.

        Parameters:
        :   **odometry** ([*Odometry*](#mavsdk.telemetry_server.Odometry "mavsdk.telemetry_server.Odometry")) – The next odometry status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_position(*position*, *velocity\_ned*, *heading*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_position "Link to this definition")
    :   Publish to ‘position’ updates.

        Parameters:
        :   * **position** ([*Position*](#mavsdk.telemetry_server.Position "mavsdk.telemetry_server.Position")) – The next position
            * **velocity\_ned** ([*VelocityNed*](#mavsdk.telemetry_server.VelocityNed "mavsdk.telemetry_server.VelocityNed")) – The next velocity (NED)
            * **heading** ([*Heading*](#mavsdk.telemetry_server.Heading "mavsdk.telemetry_server.Heading")) – Heading (yaw) in degrees

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_position\_velocity\_ned(*position\_velocity\_ned*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_position_velocity_ned "Link to this definition")
    :   Publish to ‘position velocity’ updates.

        Parameters:
        :   **position\_velocity\_ned** ([*PositionVelocityNed*](#mavsdk.telemetry_server.PositionVelocityNed "mavsdk.telemetry_server.PositionVelocityNed")) – The next position and velocity status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_raw\_gps(*raw\_gps*, *gps\_info*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_raw_gps "Link to this definition")
    :   Publish to ‘Raw GPS’ updates.

        Parameters:
        :   * **raw\_gps** ([*RawGps*](#mavsdk.telemetry_server.RawGps "mavsdk.telemetry_server.RawGps")) – The next ‘Raw GPS’ state. Warning: this is an advanced feature, use Position updates to get the location of the drone!
            * **gps\_info** ([*GpsInfo*](#mavsdk.telemetry_server.GpsInfo "mavsdk.telemetry_server.GpsInfo")) – The next ‘GPS info’ state

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_raw\_imu(*imu*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_raw_imu "Link to this definition")
    :   Publish to ‘Raw IMU’ updates.

        Parameters:
        :   **imu** ([*Imu*](#mavsdk.telemetry_server.Imu "mavsdk.telemetry_server.Imu")) – The next raw IMU status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_scaled\_imu(*imu*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_scaled_imu "Link to this definition")
    :   Publish to ‘Scaled IMU’ updates.

        Parameters:
        :   **imu** ([*Imu*](#mavsdk.telemetry_server.Imu "mavsdk.telemetry_server.Imu")) – The next scaled IMU status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_status\_text(*status\_text*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_status_text "Link to this definition")
    :   Publish to ‘status text’ updates.

        Parameters:
        :   **status\_text** ([*StatusText*](#mavsdk.telemetry_server.StatusText "mavsdk.telemetry_server.StatusText")) – The next ‘status text’

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_sys\_status(*battery*, *rc\_receiver\_status*, *gyro\_status*, *accel\_status*, *mag\_status*, *gps\_status*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_sys_status "Link to this definition")
    :   Publish ‘sys status’ updates.

        Parameters:
        :   * **battery** ([*Battery*](#mavsdk.telemetry_server.Battery "mavsdk.telemetry_server.Battery")) – The next ‘battery’ state
            * **rc\_receiver\_status** (*bool*) – rc receiver status
            * **gyro\_status** (*bool*)
            * **accel\_status** (*bool*)
            * **mag\_status** (*bool*)
            * **gps\_status** (*bool*)

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_unix\_epoch\_time(*time\_us*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_unix_epoch_time "Link to this definition")
    :   Publish to ‘unix epoch time’ updates.

        Parameters:
        :   **time\_us** (*uint64\_t*) – The next ‘unix epoch time’ status

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

    *async* publish\_visual\_flight\_rules\_hud(*fixed\_wing\_metrics*)[¶](#mavsdk.telemetry_server.TelemetryServer.publish_visual_flight_rules_hud "Link to this definition")
    :   Publish to “Visual Flight Rules HUD” updates.

        Parameters:
        :   **fixed\_wing\_metrics** ([*FixedwingMetrics*](#mavsdk.telemetry_server.FixedwingMetrics "mavsdk.telemetry_server.FixedwingMetrics"))

        Raises:
        :   [**TelemetryServerError**](#mavsdk.telemetry_server.TelemetryServerError "mavsdk.telemetry_server.TelemetryServerError") – If the request fails. The error contains the reason for the failure.

*exception* mavsdk.telemetry\_server.TelemetryServerError(*result*, *origin*, *\*params*)[¶](#mavsdk.telemetry_server.TelemetryServerError "Link to this definition")
:   Bases: `Exception`

    Raised when a TelemetryServerResult is a fail code

*class* mavsdk.telemetry\_server.TelemetryServerResult(*result*, *result\_str*)[¶](#mavsdk.telemetry_server.TelemetryServerResult "Link to this definition")
:   Bases: `object`

    Result type.

    Parameters:
    :   * **result** ([*Result*](#mavsdk.telemetry_server.TelemetryServerResult.Result "mavsdk.telemetry_server.TelemetryServerResult.Result")) – Result enum value
        * **result\_str** (*std::string*) – Human-readable English string describing the result

    *class* Result(*value*)[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result "Link to this definition")
    :   Bases: `Enum`

        Possible results returned for telemetry requests.

        ## Values[¶](#id4 "Link to this heading")

        UNKNOWN
        :   Unknown result

        SUCCESS
        :   Success: the telemetry command was accepted by the vehicle

        NO\_SYSTEM
        :   No system connected

        CONNECTION\_ERROR
        :   Connection error

        BUSY
        :   Vehicle is busy

        COMMAND\_DENIED
        :   Command refused by vehicle

        TIMEOUT
        :   Request timed out

        UNSUPPORTED
        :   Request not supported

        BUSY *= 4*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.BUSY "Link to this definition")

        COMMAND\_DENIED *= 5*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.COMMAND_DENIED "Link to this definition")

        CONNECTION\_ERROR *= 3*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.CONNECTION_ERROR "Link to this definition")

        NO\_SYSTEM *= 2*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.NO_SYSTEM "Link to this definition")

        SUCCESS *= 1*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.SUCCESS "Link to this definition")

        TIMEOUT *= 6*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.TIMEOUT "Link to this definition")

        UNKNOWN *= 0*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.UNKNOWN "Link to this definition")

        UNSUPPORTED *= 7*[¶](#mavsdk.telemetry_server.TelemetryServerResult.Result.UNSUPPORTED "Link to this definition")

*class* mavsdk.telemetry\_server.VelocityBody(*x\_m\_s*, *y\_m\_s*, *z\_m\_s*)[¶](#mavsdk.telemetry_server.VelocityBody "Link to this definition")
:   Bases: `object`

    Velocity type, represented in the Body (X Y Z) frame and in metres/second.

    Parameters:
    :   * **x\_m\_s** (*float*) – Velocity in X in metres/second
        * **y\_m\_s** (*float*) – Velocity in Y in metres/second
        * **z\_m\_s** (*float*) – Velocity in Z in metres/second

*class* mavsdk.telemetry\_server.VelocityNed(*north\_m\_s*, *east\_m\_s*, *down\_m\_s*)[¶](#mavsdk.telemetry_server.VelocityNed "Link to this definition")
:   Bases: `object`

    VelocityNed message type.

    Parameters:
    :   * **north\_m\_s** (*float*) – Velocity along north direction in metres per second
        * **east\_m\_s** (*float*) – Velocity along east direction in metres per second
        * **down\_m\_s** (*float*) – Velocity along down direction in metres per second

*class* mavsdk.telemetry\_server.VtolState(*value*)[¶](#mavsdk.telemetry_server.VtolState "Link to this definition")
:   Bases: `Enum`

    Maps to MAV\_VTOL\_STATE

    ## Values[¶](#id5 "Link to this heading")

    UNDEFINED
    :   Not VTOL

    TRANSITION\_TO\_FW
    :   Transitioning to fixed-wing

    TRANSITION\_TO\_MC
    :   Transitioning to multi-copter

    MC
    :   Multi-copter

    FW
    :   Fixed-wing

    FW *= 4*[¶](#mavsdk.telemetry_server.VtolState.FW "Link to this definition")

    MC *= 3*[¶](#mavsdk.telemetry_server.VtolState.MC "Link to this definition")

    TRANSITION\_TO\_FW *= 1*[¶](#mavsdk.telemetry_server.VtolState.TRANSITION_TO_FW "Link to this definition")

    TRANSITION\_TO\_MC *= 2*[¶](#mavsdk.telemetry_server.VtolState.TRANSITION_TO_MC "Link to this definition")

    UNDEFINED *= 0*[¶](#mavsdk.telemetry_server.VtolState.UNDEFINED "Link to this definition")