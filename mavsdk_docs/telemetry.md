[Original Source](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html)

# Telemetry[¶](#telemetry "Link to this heading")

*class* mavsdk.telemetry.AccelerationFrd(*forward\_m\_s2*, *right\_m\_s2*, *down\_m\_s2*)[¶](#mavsdk.telemetry.AccelerationFrd "Link to this definition")
:   Bases: `object`

    AccelerationFrd message type.

    Parameters:
    :   * **forward\_m\_s2** (*float*) – Acceleration in forward direction in metres per second^2
        * **right\_m\_s2** (*float*) – Acceleration in right direction in metres per second^2
        * **down\_m\_s2** (*float*) – Acceleration in down direction in metres per second^2

*class* mavsdk.telemetry.ActuatorControlTarget(*group*, *controls*)[¶](#mavsdk.telemetry.ActuatorControlTarget "Link to this definition")
:   Bases: `object`

    Actuator control target type.

    Parameters:
    :   * **group** (*int32\_t*) – An actuator control group is e.g. ‘attitude’ for the core flight controls, or ‘gimbal’ for a payload.
        * **controls** (*[**float**]*) – Controls normed from -1 to 1, where 0 is neutral position.

*class* mavsdk.telemetry.ActuatorOutputStatus(*active*, *actuator*)[¶](#mavsdk.telemetry.ActuatorOutputStatus "Link to this definition")
:   Bases: `object`

    Actuator output status type.

    Parameters:
    :   * **active** (*uint32\_t*) – Active outputs
        * **actuator** (*[**float**]*) – Servo/motor output values

*class* mavsdk.telemetry.Altitude(*altitude\_monotonic\_m*, *altitude\_amsl\_m*, *altitude\_local\_m*, *altitude\_relative\_m*, *altitude\_terrain\_m*, *bottom\_clearance\_m*)[¶](#mavsdk.telemetry.Altitude "Link to this definition")
:   Bases: `object`

    Altitude message type

    Parameters:
    :   * **altitude\_monotonic\_m** (*float*) – Altitude in meters is initialized on system boot and monotonic
        * **altitude\_amsl\_m** (*float*) – Altitude AMSL (above mean sea level) in meters
        * **altitude\_local\_m** (*float*) – Local altitude in meters
        * **altitude\_relative\_m** (*float*) – Altitude above home position in meters
        * **altitude\_terrain\_m** (*float*) – Altitude above terrain in meters
        * **bottom\_clearance\_m** (*float*) – This is not the altitude, but the clear space below the system according to the fused clearance estimate in meters.

*class* mavsdk.telemetry.AngularVelocityBody(*roll\_rad\_s*, *pitch\_rad\_s*, *yaw\_rad\_s*)[¶](#mavsdk.telemetry.AngularVelocityBody "Link to this definition")
:   Bases: `object`

    Angular velocity type.

    Parameters:
    :   * **roll\_rad\_s** (*float*) – Roll angular velocity
        * **pitch\_rad\_s** (*float*) – Pitch angular velocity
        * **yaw\_rad\_s** (*float*) – Yaw angular velocity

*class* mavsdk.telemetry.AngularVelocityFrd(*forward\_rad\_s*, *right\_rad\_s*, *down\_rad\_s*)[¶](#mavsdk.telemetry.AngularVelocityFrd "Link to this definition")
:   Bases: `object`

    AngularVelocityFrd message type.

    Parameters:
    :   * **forward\_rad\_s** (*float*) – Angular velocity in forward direction in radians per second
        * **right\_rad\_s** (*float*) – Angular velocity in right direction in radians per second
        * **down\_rad\_s** (*float*) – Angular velocity in Down direction in radians per second

*class* mavsdk.telemetry.Battery(*id*, *temperature\_degc*, *voltage\_v*, *current\_battery\_a*, *capacity\_consumed\_ah*, *remaining\_percent*, *time\_remaining\_s*, *battery\_function*)[¶](#mavsdk.telemetry.Battery "Link to this definition")
:   Bases: `object`

    Battery type.

    Parameters:
    :   * **id** (*uint32\_t*) – Battery ID, for systems with multiple batteries
        * **temperature\_degc** (*float*) – Temperature of the battery in degrees Celsius. NAN for unknown temperature
        * **voltage\_v** (*float*) – Voltage in volts
        * **current\_battery\_a** (*float*) – Battery current in Amps, NAN if autopilot does not measure the current
        * **capacity\_consumed\_ah** (*float*) – Consumed charge in Amp hours, NAN if autopilot does not provide consumption estimate
        * **remaining\_percent** (*float*) – Estimated battery remaining (range: 0 to 100)
        * **time\_remaining\_s** (*float*) – Estimated battery usage time remaining
        * **battery\_function** ([*BatteryFunction*](#mavsdk.telemetry.BatteryFunction "mavsdk.telemetry.BatteryFunction")) – Function of the battery

*class* mavsdk.telemetry.BatteryFunction(*value*)[¶](#mavsdk.telemetry.BatteryFunction "Link to this definition")
:   Bases: `Enum`

    Battery function type.

    ## Values[¶](#values "Link to this heading")

    UNKNOWN
    :   Battery function is unknown

    ALL
    :   Battery supports all flight systems

    PROPULSION
    :   Battery for the propulsion system

    AVIONICS
    :   Avionics battery

    PAYLOAD
    :   Payload battery

    ALL *= 1*[¶](#mavsdk.telemetry.BatteryFunction.ALL "Link to this definition")

    AVIONICS *= 3*[¶](#mavsdk.telemetry.BatteryFunction.AVIONICS "Link to this definition")

    PAYLOAD *= 4*[¶](#mavsdk.telemetry.BatteryFunction.PAYLOAD "Link to this definition")

    PROPULSION *= 2*[¶](#mavsdk.telemetry.BatteryFunction.PROPULSION "Link to this definition")

    UNKNOWN *= 0*[¶](#mavsdk.telemetry.BatteryFunction.UNKNOWN "Link to this definition")

*class* mavsdk.telemetry.Covariance(*covariance\_matrix*)[¶](#mavsdk.telemetry.Covariance "Link to this definition")
:   Bases: `object`

    Covariance type.

    Row-major representation of a 6x6 cross-covariance matrix
    upper right triangle.
    Set first to NaN if unknown.

    Parameters:
    :   **covariance\_matrix** (*[**float**]*) – Representation of a covariance matrix.

*class* mavsdk.telemetry.DistanceSensor(*minimum\_distance\_m*, *maximum\_distance\_m*, *current\_distance\_m*, *orientation*)[¶](#mavsdk.telemetry.DistanceSensor "Link to this definition")
:   Bases: `object`

    DistanceSensor message type.

    Parameters:
    :   * **minimum\_distance\_m** (*float*) – Minimum distance the sensor can measure, NaN if unknown.
        * **maximum\_distance\_m** (*float*) – Maximum distance the sensor can measure, NaN if unknown.
        * **current\_distance\_m** (*float*) – Current distance reading, NaN if unknown.
        * **orientation** ([*EulerAngle*](#mavsdk.telemetry.EulerAngle "mavsdk.telemetry.EulerAngle")) – Sensor Orientation reading.

*class* mavsdk.telemetry.EulerAngle(*roll\_deg*, *pitch\_deg*, *yaw\_deg*, *timestamp\_us*)[¶](#mavsdk.telemetry.EulerAngle "Link to this definition")
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

*class* mavsdk.telemetry.FixType(*value*)[¶](#mavsdk.telemetry.FixType "Link to this definition")
:   Bases: `Enum`

    GPS fix type.

    ## Values[¶](#id1 "Link to this heading")

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

    FIX\_2D *= 2*[¶](#mavsdk.telemetry.FixType.FIX_2D "Link to this definition")

    FIX\_3D *= 3*[¶](#mavsdk.telemetry.FixType.FIX_3D "Link to this definition")

    FIX\_DGPS *= 4*[¶](#mavsdk.telemetry.FixType.FIX_DGPS "Link to this definition")

    NO\_FIX *= 1*[¶](#mavsdk.telemetry.FixType.NO_FIX "Link to this definition")

    NO\_GPS *= 0*[¶](#mavsdk.telemetry.FixType.NO_GPS "Link to this definition")

    RTK\_FIXED *= 6*[¶](#mavsdk.telemetry.FixType.RTK_FIXED "Link to this definition")

    RTK\_FLOAT *= 5*[¶](#mavsdk.telemetry.FixType.RTK_FLOAT "Link to this definition")

*class* mavsdk.telemetry.FixedwingMetrics(*airspeed\_m\_s*, *throttle\_percentage*, *climb\_rate\_m\_s*, *groundspeed\_m\_s*, *heading\_deg*, *absolute\_altitude\_m*)[¶](#mavsdk.telemetry.FixedwingMetrics "Link to this definition")
:   Bases: `object`

    FixedwingMetrics message type.

    Parameters:
    :   * **airspeed\_m\_s** (*float*) – Current indicated airspeed (IAS) in metres per second
        * **throttle\_percentage** (*float*) – Current throttle setting (0 to 100)
        * **climb\_rate\_m\_s** (*float*) – Current climb rate in metres per second
        * **groundspeed\_m\_s** (*float*) – Current groundspeed metres per second
        * **heading\_deg** (*float*) – Current heading in compass units (0-360, 0=north)
        * **absolute\_altitude\_m** (*float*) – Current altitude in metres (MSL)

*class* mavsdk.telemetry.FlightMode(*value*)[¶](#mavsdk.telemetry.FlightMode "Link to this definition")
:   Bases: `Enum`

    Flight modes.

    For more information about flight modes, check out
    <https://docs.px4.io/master/en/config/flight_mode.html>.

    ## Values[¶](#id2 "Link to this heading")

    UNKNOWN
    :   Mode not known

    READY
    :   Armed and ready to take off

    TAKEOFF
    :   Taking off

    HOLD
    :   Holding (hovering in place (or circling for fixed-wing vehicles)

    MISSION
    :   In mission

    RETURN\_TO\_LAUNCH
    :   Returning to launch position (then landing)

    LAND
    :   Landing

    OFFBOARD
    :   In ‘offboard’ mode

    FOLLOW\_ME
    :   In ‘follow-me’ mode

    MANUAL
    :   In ‘Manual’ mode

    ALTCTL
    :   In ‘Altitude Control’ mode

    POSCTL
    :   In ‘Position Control’ mode

    ACRO
    :   In ‘Acro’ mode

    STABILIZED
    :   In ‘Stabilize’ mode

    RATTITUDE
    :   In ‘Rattitude’ mode

    ACRO *= 12*[¶](#mavsdk.telemetry.FlightMode.ACRO "Link to this definition")

    ALTCTL *= 10*[¶](#mavsdk.telemetry.FlightMode.ALTCTL "Link to this definition")

    FOLLOW\_ME *= 8*[¶](#mavsdk.telemetry.FlightMode.FOLLOW_ME "Link to this definition")

    HOLD *= 3*[¶](#mavsdk.telemetry.FlightMode.HOLD "Link to this definition")

    LAND *= 6*[¶](#mavsdk.telemetry.FlightMode.LAND "Link to this definition")

    MANUAL *= 9*[¶](#mavsdk.telemetry.FlightMode.MANUAL "Link to this definition")

    MISSION *= 4*[¶](#mavsdk.telemetry.FlightMode.MISSION "Link to this definition")

    OFFBOARD *= 7*[¶](#mavsdk.telemetry.FlightMode.OFFBOARD "Link to this definition")

    POSCTL *= 11*[¶](#mavsdk.telemetry.FlightMode.POSCTL "Link to this definition")

    RATTITUDE *= 14*[¶](#mavsdk.telemetry.FlightMode.RATTITUDE "Link to this definition")

    READY *= 1*[¶](#mavsdk.telemetry.FlightMode.READY "Link to this definition")

    RETURN\_TO\_LAUNCH *= 5*[¶](#mavsdk.telemetry.FlightMode.RETURN_TO_LAUNCH "Link to this definition")

    STABILIZED *= 13*[¶](#mavsdk.telemetry.FlightMode.STABILIZED "Link to this definition")

    TAKEOFF *= 2*[¶](#mavsdk.telemetry.FlightMode.TAKEOFF "Link to this definition")

    UNKNOWN *= 0*[¶](#mavsdk.telemetry.FlightMode.UNKNOWN "Link to this definition")

*class* mavsdk.telemetry.GpsGlobalOrigin(*latitude\_deg*, *longitude\_deg*, *altitude\_m*)[¶](#mavsdk.telemetry.GpsGlobalOrigin "Link to this definition")
:   Bases: `object`

    Gps global origin type.

    Parameters:
    :   * **latitude\_deg** (*double*) – Latitude of the origin
        * **longitude\_deg** (*double*) – Longitude of the origin
        * **altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres

*class* mavsdk.telemetry.GpsInfo(*num\_satellites*, *fix\_type*)[¶](#mavsdk.telemetry.GpsInfo "Link to this definition")
:   Bases: `object`

    GPS information type.

    Parameters:
    :   * **num\_satellites** (*int32\_t*) – Number of visible satellites in use
        * **fix\_type** ([*FixType*](#mavsdk.telemetry.FixType "mavsdk.telemetry.FixType")) – Fix type

*class* mavsdk.telemetry.GroundTruth(*latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*)[¶](#mavsdk.telemetry.GroundTruth "Link to this definition")
:   Bases: `object`

    GroundTruth message type.

    Parameters:
    :   * **latitude\_deg** (*double*) – Latitude in degrees (range: -90 to +90)
        * **longitude\_deg** (*double*) – Longitude in degrees (range: -180 to 180)
        * **absolute\_altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres

*class* mavsdk.telemetry.Heading(*heading\_deg*)[¶](#mavsdk.telemetry.Heading "Link to this definition")
:   Bases: `object`

    Heading type used for global position

    Parameters:
    :   **heading\_deg** (*double*) – Heading in degrees (range: 0 to +360)

*class* mavsdk.telemetry.Health(*is\_gyrometer\_calibration\_ok*, *is\_accelerometer\_calibration\_ok*, *is\_magnetometer\_calibration\_ok*, *is\_local\_position\_ok*, *is\_global\_position\_ok*, *is\_home\_position\_ok*, *is\_armable*)[¶](#mavsdk.telemetry.Health "Link to this definition")
:   Bases: `object`

    Health type.

    Parameters:
    :   * **is\_gyrometer\_calibration\_ok** (*bool*) – True if the gyrometer is calibrated
        * **is\_accelerometer\_calibration\_ok** (*bool*) – True if the accelerometer is calibrated
        * **is\_magnetometer\_calibration\_ok** (*bool*) – True if the magnetometer is calibrated
        * **is\_local\_position\_ok** (*bool*) – True if the local position estimate is good enough to fly in ‘position control’ mode
        * **is\_global\_position\_ok** (*bool*) – True if the global position estimate is good enough to fly in ‘position control’ mode
        * **is\_home\_position\_ok** (*bool*) – True if the home position has been initialized properly
        * **is\_armable** (*bool*) – True if system can be armed

*class* mavsdk.telemetry.Imu(*acceleration\_frd*, *angular\_velocity\_frd*, *magnetic\_field\_frd*, *temperature\_degc*, *timestamp\_us*)[¶](#mavsdk.telemetry.Imu "Link to this definition")
:   Bases: `object`

    Imu message type.

    Parameters:
    :   * **acceleration\_frd** ([*AccelerationFrd*](#mavsdk.telemetry.AccelerationFrd "mavsdk.telemetry.AccelerationFrd")) – Acceleration
        * **angular\_velocity\_frd** ([*AngularVelocityFrd*](#mavsdk.telemetry.AngularVelocityFrd "mavsdk.telemetry.AngularVelocityFrd")) – Angular velocity
        * **magnetic\_field\_frd** ([*MagneticFieldFrd*](#mavsdk.telemetry.MagneticFieldFrd "mavsdk.telemetry.MagneticFieldFrd")) – Magnetic field
        * **temperature\_degc** (*float*) – Temperature
        * **timestamp\_us** (*uint64\_t*) – Timestamp in microseconds

*class* mavsdk.telemetry.LandedState(*value*)[¶](#mavsdk.telemetry.LandedState "Link to this definition")
:   Bases: `Enum`

    Landed State enumeration.

    ## Values[¶](#id3 "Link to this heading")

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

    IN\_AIR *= 2*[¶](#mavsdk.telemetry.LandedState.IN_AIR "Link to this definition")

    LANDING *= 4*[¶](#mavsdk.telemetry.LandedState.LANDING "Link to this definition")

    ON\_GROUND *= 1*[¶](#mavsdk.telemetry.LandedState.ON_GROUND "Link to this definition")

    TAKING\_OFF *= 3*[¶](#mavsdk.telemetry.LandedState.TAKING_OFF "Link to this definition")

    UNKNOWN *= 0*[¶](#mavsdk.telemetry.LandedState.UNKNOWN "Link to this definition")

*class* mavsdk.telemetry.MagneticFieldFrd(*forward\_gauss*, *right\_gauss*, *down\_gauss*)[¶](#mavsdk.telemetry.MagneticFieldFrd "Link to this definition")
:   Bases: `object`

    MagneticFieldFrd message type.

    Parameters:
    :   * **forward\_gauss** (*float*) – Magnetic field in forward direction measured in Gauss
        * **right\_gauss** (*float*) – Magnetic field in East direction measured in Gauss
        * **down\_gauss** (*float*) – Magnetic field in Down direction measured in Gauss

*class* mavsdk.telemetry.Odometry(*time\_usec*, *frame\_id*, *child\_frame\_id*, *position\_body*, *q*, *velocity\_body*, *angular\_velocity\_body*, *pose\_covariance*, *velocity\_covariance*)[¶](#mavsdk.telemetry.Odometry "Link to this definition")
:   Bases: `object`

    Odometry message type.

    Parameters:
    :   * **time\_usec** (*uint64\_t*) – Timestamp (0 to use Backend timestamp).
        * **frame\_id** ([*MavFrame*](#mavsdk.telemetry.Odometry.MavFrame "mavsdk.telemetry.Odometry.MavFrame")) – Coordinate frame of reference for the pose data.
        * **child\_frame\_id** ([*MavFrame*](#mavsdk.telemetry.Odometry.MavFrame "mavsdk.telemetry.Odometry.MavFrame")) – Coordinate frame of reference for the velocity in free space (twist) data.
        * **position\_body** ([*PositionBody*](#mavsdk.telemetry.PositionBody "mavsdk.telemetry.PositionBody")) – Position.
        * **q** ([*Quaternion*](#mavsdk.telemetry.Quaternion "mavsdk.telemetry.Quaternion")) – Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation).
        * **velocity\_body** ([*VelocityBody*](#mavsdk.telemetry.VelocityBody "mavsdk.telemetry.VelocityBody")) – Linear velocity (m/s).
        * **angular\_velocity\_body** ([*AngularVelocityBody*](#mavsdk.telemetry.AngularVelocityBody "mavsdk.telemetry.AngularVelocityBody")) – Angular velocity (rad/s).
        * **pose\_covariance** ([*Covariance*](#mavsdk.telemetry.Covariance "mavsdk.telemetry.Covariance")) – Pose cross-covariance matrix.
        * **velocity\_covariance** ([*Covariance*](#mavsdk.telemetry.Covariance "mavsdk.telemetry.Covariance")) – Velocity cross-covariance matrix.

    *class* MavFrame(*value*)[¶](#mavsdk.telemetry.Odometry.MavFrame "Link to this definition")
    :   Bases: `Enum`

        Mavlink frame id

        ## Values[¶](#id4 "Link to this heading")

        UNDEF
        :   Frame is undefined.

        BODY\_NED
        :   Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.

        VISION\_NED
        :   Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).

        ESTIM\_NED
        :   Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).

        BODY\_NED *= 1*[¶](#mavsdk.telemetry.Odometry.MavFrame.BODY_NED "Link to this definition")

        ESTIM\_NED *= 3*[¶](#mavsdk.telemetry.Odometry.MavFrame.ESTIM_NED "Link to this definition")

        UNDEF *= 0*[¶](#mavsdk.telemetry.Odometry.MavFrame.UNDEF "Link to this definition")

        VISION\_NED *= 2*[¶](#mavsdk.telemetry.Odometry.MavFrame.VISION_NED "Link to this definition")

*class* mavsdk.telemetry.Position(*latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*, *relative\_altitude\_m*)[¶](#mavsdk.telemetry.Position "Link to this definition")
:   Bases: `object`

    Position type in global coordinates.

    Parameters:
    :   * **latitude\_deg** (*double*) – Latitude in degrees (range: -90 to +90)
        * **longitude\_deg** (*double*) – Longitude in degrees (range: -180 to +180)
        * **absolute\_altitude\_m** (*float*) – Altitude AMSL (above mean sea level) in metres
        * **relative\_altitude\_m** (*float*) – Altitude relative to takeoff altitude in metres

*class* mavsdk.telemetry.PositionBody(*x\_m*, *y\_m*, *z\_m*)[¶](#mavsdk.telemetry.PositionBody "Link to this definition")
:   Bases: `object`

    Position type, represented in the Body (X Y Z) frame

    Parameters:
    :   * **x\_m** (*float*) – X Position in metres.
        * **y\_m** (*float*) – Y Position in metres.
        * **z\_m** (*float*) – Z Position in metres.

*class* mavsdk.telemetry.PositionNed(*north\_m*, *east\_m*, *down\_m*)[¶](#mavsdk.telemetry.PositionNed "Link to this definition")
:   Bases: `object`

    PositionNed message type.

    Parameters:
    :   * **north\_m** (*float*) – Position along north direction in metres
        * **east\_m** (*float*) – Position along east direction in metres
        * **down\_m** (*float*) – Position along down direction in metres

*class* mavsdk.telemetry.PositionVelocityNed(*position*, *velocity*)[¶](#mavsdk.telemetry.PositionVelocityNed "Link to this definition")
:   Bases: `object`

    PositionVelocityNed message type.

    Parameters:
    :   * **position** ([*PositionNed*](#mavsdk.telemetry.PositionNed "mavsdk.telemetry.PositionNed")) – Position (NED)
        * **velocity** ([*VelocityNed*](#mavsdk.telemetry.VelocityNed "mavsdk.telemetry.VelocityNed")) – Velocity (NED)

*class* mavsdk.telemetry.Quaternion(*w*, *x*, *y*, *z*, *timestamp\_us*)[¶](#mavsdk.telemetry.Quaternion "Link to this definition")
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

*class* mavsdk.telemetry.RawGps(*timestamp\_us*, *latitude\_deg*, *longitude\_deg*, *absolute\_altitude\_m*, *hdop*, *vdop*, *velocity\_m\_s*, *cog\_deg*, *altitude\_ellipsoid\_m*, *horizontal\_uncertainty\_m*, *vertical\_uncertainty\_m*, *velocity\_uncertainty\_m\_s*, *heading\_uncertainty\_deg*, *yaw\_deg*)[¶](#mavsdk.telemetry.RawGps "Link to this definition")
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

*class* mavsdk.telemetry.RcStatus(*was\_available\_once*, *is\_available*, *signal\_strength\_percent*)[¶](#mavsdk.telemetry.RcStatus "Link to this definition")
:   Bases: `object`

    Remote control status type.

    Parameters:
    :   * **was\_available\_once** (*bool*) – True if an RC signal has been available once
        * **is\_available** (*bool*) – True if the RC signal is available now
        * **signal\_strength\_percent** (*float*) – Signal strength (range: 0 to 100, NaN if unknown)

*class* mavsdk.telemetry.ScaledPressure(*timestamp\_us*, *absolute\_pressure\_hpa*, *differential\_pressure\_hpa*, *temperature\_deg*, *differential\_pressure\_temperature\_deg*)[¶](#mavsdk.telemetry.ScaledPressure "Link to this definition")
:   Bases: `object`

    Scaled Pressure message type.

    Parameters:
    :   * **timestamp\_us** (*uint64\_t*) – Timestamp (time since system boot)
        * **absolute\_pressure\_hpa** (*float*) – Absolute pressure in hPa
        * **differential\_pressure\_hpa** (*float*) – Differential pressure 1 in hPa
        * **temperature\_deg** (*float*) – Absolute pressure temperature (in celsius)
        * **differential\_pressure\_temperature\_deg** (*float*) – Differential pressure temperature (in celsius, 0 if not available)

*class* mavsdk.telemetry.StatusText(*type*, *text*)[¶](#mavsdk.telemetry.StatusText "Link to this definition")
:   Bases: `object`

    StatusText information type.

    Parameters:
    :   * **type** ([*StatusTextType*](#mavsdk.telemetry.StatusTextType "mavsdk.telemetry.StatusTextType")) – Message type
        * **text** (*std::string*) – MAVLink status message

*class* mavsdk.telemetry.StatusTextType(*value*)[¶](#mavsdk.telemetry.StatusTextType "Link to this definition")
:   Bases: `Enum`

    Status types.

    ## Values[¶](#id5 "Link to this heading")

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

    ALERT *= 6*[¶](#mavsdk.telemetry.StatusTextType.ALERT "Link to this definition")

    CRITICAL *= 5*[¶](#mavsdk.telemetry.StatusTextType.CRITICAL "Link to this definition")

    DEBUG *= 0*[¶](#mavsdk.telemetry.StatusTextType.DEBUG "Link to this definition")

    EMERGENCY *= 7*[¶](#mavsdk.telemetry.StatusTextType.EMERGENCY "Link to this definition")

    ERROR *= 4*[¶](#mavsdk.telemetry.StatusTextType.ERROR "Link to this definition")

    INFO *= 1*[¶](#mavsdk.telemetry.StatusTextType.INFO "Link to this definition")

    NOTICE *= 2*[¶](#mavsdk.telemetry.StatusTextType.NOTICE "Link to this definition")

    WARNING *= 3*[¶](#mavsdk.telemetry.StatusTextType.WARNING "Link to this definition")

*class* mavsdk.telemetry.Telemetry(*async\_plugin\_manager*)[¶](#mavsdk.telemetry.Telemetry "Link to this definition")
:   Bases: `AsyncBase`

    Allow users to get vehicle telemetry and state information
    (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates.
    Certain Telemetry Topics such as, Position or Velocity\_Ned require GPS Fix before data gets published.

    Generated by dcsdkgen - MAVSDK Telemetry API

    *async* actuator\_control\_target()[¶](#mavsdk.telemetry.Telemetry.actuator_control_target "Link to this definition")
    :   Subscribe to ‘actuator control target’ updates.

        Yields:
        :   **actuator\_control\_target** (*ActuatorControlTarget*) – The next actuator control target

    *async* actuator\_output\_status()[¶](#mavsdk.telemetry.Telemetry.actuator_output_status "Link to this definition")
    :   Subscribe to ‘actuator output status’ updates.

        Yields:
        :   **actuator\_output\_status** (*ActuatorOutputStatus*) – The next actuator output status

    *async* altitude()[¶](#mavsdk.telemetry.Telemetry.altitude "Link to this definition")
    :   Subscribe to ‘Altitude’ updates.

        Yields:
        :   **altitude** (*Altitude*) – The next altitude

    *async* armed()[¶](#mavsdk.telemetry.Telemetry.armed "Link to this definition")
    :   Subscribe to armed updates.

        Yields:
        :   **is\_armed** (*bool*) – The next ‘armed’ state

    *async* attitude\_angular\_velocity\_body()[¶](#mavsdk.telemetry.Telemetry.attitude_angular_velocity_body "Link to this definition")
    :   Subscribe to ‘attitude’ updates (angular velocity)

        Yields:
        :   **attitude\_angular\_velocity\_body** (*AngularVelocityBody*) – The next angular velocity (rad/s)

    *async* attitude\_euler()[¶](#mavsdk.telemetry.Telemetry.attitude_euler "Link to this definition")
    :   Subscribe to ‘attitude’ updates (Euler).

        Yields:
        :   **attitude\_euler** (*EulerAngle*) – The next attitude (Euler)

    *async* attitude\_quaternion()[¶](#mavsdk.telemetry.Telemetry.attitude_quaternion "Link to this definition")
    :   Subscribe to ‘attitude’ updates (quaternion).

        Yields:
        :   **attitude\_quaternion** (*Quaternion*) – The next attitude (quaternion)

    *async* battery()[¶](#mavsdk.telemetry.Telemetry.battery "Link to this definition")
    :   Subscribe to ‘battery’ updates.

        Yields:
        :   **battery** (*Battery*) – The next ‘battery’ state

    *async* distance\_sensor()[¶](#mavsdk.telemetry.Telemetry.distance_sensor "Link to this definition")
    :   Subscribe to ‘Distance Sensor’ updates.

        Yields:
        :   **distance\_sensor** (*DistanceSensor*) – The next Distance Sensor status

    *async* fixedwing\_metrics()[¶](#mavsdk.telemetry.Telemetry.fixedwing_metrics "Link to this definition")
    :   Subscribe to ‘fixedwing metrics’ updates.

        Yields:
        :   **fixedwing\_metrics** (*FixedwingMetrics*) – The next fixedwing metrics

    *async* flight\_mode()[¶](#mavsdk.telemetry.Telemetry.flight_mode "Link to this definition")
    :   Subscribe to ‘flight mode’ updates.

        Yields:
        :   **flight\_mode** (*FlightMode*) – The next flight mode

    *async* get\_gps\_global\_origin()[¶](#mavsdk.telemetry.Telemetry.get_gps_global_origin "Link to this definition")
    :   Get the GPS location of where the estimator has been initialized.

        Returns:
        :   **gps\_global\_origin**

        Return type:
        :   [GpsGlobalOrigin](#mavsdk.telemetry.GpsGlobalOrigin "mavsdk.telemetry.GpsGlobalOrigin")

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* gps\_info()[¶](#mavsdk.telemetry.Telemetry.gps_info "Link to this definition")
    :   Subscribe to ‘GPS info’ updates.

        Yields:
        :   **gps\_info** (*GpsInfo*) – The next ‘GPS info’ state

    *async* ground\_truth()[¶](#mavsdk.telemetry.Telemetry.ground_truth "Link to this definition")
    :   Subscribe to ‘ground truth’ updates.

        Yields:
        :   **ground\_truth** (*GroundTruth*) – Ground truth position information available in simulation

    *async* heading()[¶](#mavsdk.telemetry.Telemetry.heading "Link to this definition")
    :   Subscribe to ‘Heading’ updates.

        Yields:
        :   **heading\_deg** (*Heading*) – The next heading (yaw) in degrees

    *async* health()[¶](#mavsdk.telemetry.Telemetry.health "Link to this definition")
    :   Subscribe to ‘health’ updates.

        Yields:
        :   **health** (*Health*) – The next ‘health’ state

    *async* health\_all\_ok()[¶](#mavsdk.telemetry.Telemetry.health_all_ok "Link to this definition")
    :   Subscribe to ‘HealthAllOk’ updates.

        Yields:
        :   **is\_health\_all\_ok** (*bool*) – The next ‘health all ok’ status

    *async* home()[¶](#mavsdk.telemetry.Telemetry.home "Link to this definition")
    :   Subscribe to ‘home position’ updates.

        Yields:
        :   **home** (*Position*) – The next home position

    *async* imu()[¶](#mavsdk.telemetry.Telemetry.imu "Link to this definition")
    :   Subscribe to ‘IMU’ updates (in SI units in NED body frame).

        Yields:
        :   **imu** (*Imu*) – The next IMU status

    *async* in\_air()[¶](#mavsdk.telemetry.Telemetry.in_air "Link to this definition")
    :   Subscribe to in-air updates.

        Yields:
        :   **is\_in\_air** (*bool*) – The next ‘in-air’ state

    *async* landed\_state()[¶](#mavsdk.telemetry.Telemetry.landed_state "Link to this definition")
    :   Subscribe to landed state updates

        Yields:
        :   **landed\_state** (*LandedState*) – The next ‘landed’ state

    name *= 'Telemetry'*[¶](#mavsdk.telemetry.Telemetry.name "Link to this definition")

    *async* odometry()[¶](#mavsdk.telemetry.Telemetry.odometry "Link to this definition")
    :   Subscribe to ‘odometry’ updates.

        Yields:
        :   **odometry** (*Odometry*) – The next odometry status

    *async* position()[¶](#mavsdk.telemetry.Telemetry.position "Link to this definition")
    :   Subscribe to ‘position’ updates.

        Yields:
        :   **position** (*Position*) – The next position

    *async* position\_velocity\_ned()[¶](#mavsdk.telemetry.Telemetry.position_velocity_ned "Link to this definition")
    :   Subscribe to ‘position velocity’ updates.

        Yields:
        :   **position\_velocity\_ned** (*PositionVelocityNed*) – The next position and velocity status

    *async* raw\_gps()[¶](#mavsdk.telemetry.Telemetry.raw_gps "Link to this definition")
    :   Subscribe to ‘Raw GPS’ updates.

        Yields:
        :   **raw\_gps** (*RawGps*) – The next ‘Raw GPS’ state. Warning: this is an advanced feature, use Position updates to get the location of the drone!

    *async* raw\_imu()[¶](#mavsdk.telemetry.Telemetry.raw_imu "Link to this definition")
    :   Subscribe to ‘Raw IMU’ updates (note that units are are incorrect and “raw” as provided by the sensor)

        Yields:
        :   **imu** (*Imu*) – The next raw IMU status

    *async* rc\_status()[¶](#mavsdk.telemetry.Telemetry.rc_status "Link to this definition")
    :   Subscribe to ‘RC status’ updates.

        Yields:
        :   **rc\_status** (*RcStatus*) – The next RC status

    *async* scaled\_imu()[¶](#mavsdk.telemetry.Telemetry.scaled_imu "Link to this definition")
    :   Subscribe to ‘Scaled IMU’ updates.

        Yields:
        :   **imu** (*Imu*) – The next scaled IMU status

    *async* scaled\_pressure()[¶](#mavsdk.telemetry.Telemetry.scaled_pressure "Link to this definition")
    :   Subscribe to ‘Scaled Pressure’ updates.

        Yields:
        :   **scaled\_pressure** (*ScaledPressure*) – The next Scaled Pressure status

    *async* set\_rate\_actuator\_control\_target(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_actuator_control_target "Link to this definition")
    :   Set rate to ‘actuator control target’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_actuator\_output\_status(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_actuator_output_status "Link to this definition")
    :   Set rate to ‘actuator output status’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_altitude(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_altitude "Link to this definition")
    :   Set rate to ‘Altitude’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_attitude\_euler(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_attitude_euler "Link to this definition")
    :   Set rate to ‘attitude quaternion’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_attitude\_quaternion(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_attitude_quaternion "Link to this definition")
    :   Set rate to ‘attitude euler angle’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_battery(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_battery "Link to this definition")
    :   Set rate to ‘battery’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_distance\_sensor(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_distance_sensor "Link to this definition")
    :   Set rate to ‘Distance Sensor’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_fixedwing\_metrics(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_fixedwing_metrics "Link to this definition")
    :   Set rate to ‘fixedwing metrics’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_gps\_info(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_gps_info "Link to this definition")
    :   Set rate to ‘GPS info’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_ground\_truth(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_ground_truth "Link to this definition")
    :   Set rate to ‘ground truth’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_health(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_health "Link to this definition")
    :   Set rate to ‘Health’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_home(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_home "Link to this definition")
    :   Set rate to ‘home position’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_imu(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_imu "Link to this definition")
    :   Set rate to ‘IMU’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_in\_air(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_in_air "Link to this definition")
    :   Set rate to in-air updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_landed\_state(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_landed_state "Link to this definition")
    :   Set rate to landed state updates

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_odometry(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_odometry "Link to this definition")
    :   Set rate to ‘odometry’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_position(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_position "Link to this definition")
    :   Set rate to ‘position’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_position\_velocity\_ned(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_position_velocity_ned "Link to this definition")
    :   Set rate to ‘position velocity’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_raw\_imu(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_raw_imu "Link to this definition")
    :   Set rate to ‘Raw IMU’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_rc\_status(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_rc_status "Link to this definition")
    :   Set rate to ‘RC status’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_scaled\_imu(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_scaled_imu "Link to this definition")
    :   Set rate to ‘Scaled IMU’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_unix\_epoch\_time(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_unix_epoch_time "Link to this definition")
    :   Set rate to ‘unix epoch time’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_velocity\_ned(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_velocity_ned "Link to this definition")
    :   Set rate of camera attitude updates.
        Set rate to ‘ground speed’ updates (NED).

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* set\_rate\_vtol\_state(*rate\_hz*)[¶](#mavsdk.telemetry.Telemetry.set_rate_vtol_state "Link to this definition")
    :   Set rate to VTOL state updates

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TelemetryError**](#mavsdk.telemetry.TelemetryError "mavsdk.telemetry.TelemetryError") – If the request fails. The error contains the reason for the failure.

    *async* status\_text()[¶](#mavsdk.telemetry.Telemetry.status_text "Link to this definition")
    :   Subscribe to ‘status text’ updates.

        Yields:
        :   **status\_text** (*StatusText*) – The next ‘status text’

    *async* unix\_epoch\_time()[¶](#mavsdk.telemetry.Telemetry.unix_epoch_time "Link to this definition")
    :   Subscribe to ‘unix epoch time’ updates.

        Yields:
        :   **time\_us** (*uint64\_t*) – The next ‘unix epoch time’ status

    *async* velocity\_ned()[¶](#mavsdk.telemetry.Telemetry.velocity_ned "Link to this definition")
    :   Subscribe to ‘ground speed’ updates (NED).

        Yields:
        :   **velocity\_ned** (*VelocityNed*) – The next velocity (NED)

    *async* vtol\_state()[¶](#mavsdk.telemetry.Telemetry.vtol_state "Link to this definition")
    :   subscribe to vtol state Updates

        Yields:
        :   **vtol\_state** (*VtolState*) – The next ‘vtol’ state

    *async* wind()[¶](#mavsdk.telemetry.Telemetry.wind "Link to this definition")
    :   Subscribe to ‘Wind Estimated’ updates.

        Yields:
        :   **wind** (*Wind*) – The next wind

*exception* mavsdk.telemetry.TelemetryError(*result*, *origin*, *\*params*)[¶](#mavsdk.telemetry.TelemetryError "Link to this definition")
:   Bases: `Exception`

    Raised when a TelemetryResult is a fail code

*class* mavsdk.telemetry.TelemetryResult(*result*, *result\_str*)[¶](#mavsdk.telemetry.TelemetryResult "Link to this definition")
:   Bases: `object`

    Result type.

    Parameters:
    :   * **result** ([*Result*](#mavsdk.telemetry.TelemetryResult.Result "mavsdk.telemetry.TelemetryResult.Result")) – Result enum value
        * **result\_str** (*std::string*) – Human-readable English string describing the result

    *class* Result(*value*)[¶](#mavsdk.telemetry.TelemetryResult.Result "Link to this definition")
    :   Bases: `Enum`

        Possible results returned for telemetry requests.

        ## Values[¶](#id6 "Link to this heading")

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

        BUSY *= 4*[¶](#mavsdk.telemetry.TelemetryResult.Result.BUSY "Link to this definition")

        COMMAND\_DENIED *= 5*[¶](#mavsdk.telemetry.TelemetryResult.Result.COMMAND_DENIED "Link to this definition")

        CONNECTION\_ERROR *= 3*[¶](#mavsdk.telemetry.TelemetryResult.Result.CONNECTION_ERROR "Link to this definition")

        NO\_SYSTEM *= 2*[¶](#mavsdk.telemetry.TelemetryResult.Result.NO_SYSTEM "Link to this definition")

        SUCCESS *= 1*[¶](#mavsdk.telemetry.TelemetryResult.Result.SUCCESS "Link to this definition")

        TIMEOUT *= 6*[¶](#mavsdk.telemetry.TelemetryResult.Result.TIMEOUT "Link to this definition")

        UNKNOWN *= 0*[¶](#mavsdk.telemetry.TelemetryResult.Result.UNKNOWN "Link to this definition")

        UNSUPPORTED *= 7*[¶](#mavsdk.telemetry.TelemetryResult.Result.UNSUPPORTED "Link to this definition")

*class* mavsdk.telemetry.VelocityBody(*x\_m\_s*, *y\_m\_s*, *z\_m\_s*)[¶](#mavsdk.telemetry.VelocityBody "Link to this definition")
:   Bases: `object`

    Velocity type, represented in the Body (X Y Z) frame and in metres/second.

    Parameters:
    :   * **x\_m\_s** (*float*) – Velocity in X in metres/second
        * **y\_m\_s** (*float*) – Velocity in Y in metres/second
        * **z\_m\_s** (*float*) – Velocity in Z in metres/second

*class* mavsdk.telemetry.VelocityNed(*north\_m\_s*, *east\_m\_s*, *down\_m\_s*)[¶](#mavsdk.telemetry.VelocityNed "Link to this definition")
:   Bases: `object`

    VelocityNed message type.

    Parameters:
    :   * **north\_m\_s** (*float*) – Velocity along north direction in metres per second
        * **east\_m\_s** (*float*) – Velocity along east direction in metres per second
        * **down\_m\_s** (*float*) – Velocity along down direction in metres per second

*class* mavsdk.telemetry.VtolState(*value*)[¶](#mavsdk.telemetry.VtolState "Link to this definition")
:   Bases: `Enum`

    VTOL State enumeration

    ## Values[¶](#id7 "Link to this heading")

    UNDEFINED
    :   MAV is not configured as VTOL

    TRANSITION\_TO\_FW
    :   VTOL is in transition from multicopter to fixed-wing

    TRANSITION\_TO\_MC
    :   VTOL is in transition from fixed-wing to multicopter

    MC
    :   VTOL is in multicopter state

    FW
    :   VTOL is in fixed-wing state

    FW *= 4*[¶](#mavsdk.telemetry.VtolState.FW "Link to this definition")

    MC *= 3*[¶](#mavsdk.telemetry.VtolState.MC "Link to this definition")

    TRANSITION\_TO\_FW *= 1*[¶](#mavsdk.telemetry.VtolState.TRANSITION_TO_FW "Link to this definition")

    TRANSITION\_TO\_MC *= 2*[¶](#mavsdk.telemetry.VtolState.TRANSITION_TO_MC "Link to this definition")

    UNDEFINED *= 0*[¶](#mavsdk.telemetry.VtolState.UNDEFINED "Link to this definition")

*class* mavsdk.telemetry.Wind(*wind\_x\_ned\_m\_s*, *wind\_y\_ned\_m\_s*, *wind\_z\_ned\_m\_s*, *horizontal\_variability\_stddev\_m\_s*, *vertical\_variability\_stddev\_m\_s*, *wind\_altitude\_msl\_m*, *horizontal\_wind\_speed\_accuracy\_m\_s*, *vertical\_wind\_speed\_accuracy\_m\_s*)[¶](#mavsdk.telemetry.Wind "Link to this definition")
:   Bases: `object`

    Wind message type

    Parameters:
    :   * **wind\_x\_ned\_m\_s** (*float*) – Wind in North (NED) direction
        * **wind\_y\_ned\_m\_s** (*float*) – Wind in East (NED) direction
        * **wind\_z\_ned\_m\_s** (*float*) – Wind in down (NED) direction
        * **horizontal\_variability\_stddev\_m\_s** (*float*) – Variability of wind in XY, 1-STD estimated from a 1 Hz lowpassed wind estimate
        * **vertical\_variability\_stddev\_m\_s** (*float*) – Variability of wind in Z, 1-STD estimated from a 1 Hz lowpassed wind estimate
        * **wind\_altitude\_msl\_m** (*float*) – Altitude (MSL) that this measurement was taken at
        * **horizontal\_wind\_speed\_accuracy\_m\_s** (*float*) – Horizontal speed 1-STD accuracy
        * **vertical\_wind\_speed\_accuracy\_m\_s** (*float*) – Vertical speed 1-STD accuracy