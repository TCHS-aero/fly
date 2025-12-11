[Original Source](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/transponder.html)

# Transponder[¶](#transponder "Link to this heading")

*class* mavsdk.transponder.AdsbAltitudeType(*value*)[¶](#mavsdk.transponder.AdsbAltitudeType "Link to this definition")
:   Bases: `Enum`

    Altitude type used in AdsbVehicle message

    ## Values[¶](#values "Link to this heading")

    PRESSURE\_QNH
    :   Altitude reported from a Baro source using QNH reference

    GEOMETRIC
    :   Altitude reported from a GNSS source

    GEOMETRIC *= 1*[¶](#mavsdk.transponder.AdsbAltitudeType.GEOMETRIC "Link to this definition")

    PRESSURE\_QNH *= 0*[¶](#mavsdk.transponder.AdsbAltitudeType.PRESSURE_QNH "Link to this definition")

*class* mavsdk.transponder.AdsbEmitterType(*value*)[¶](#mavsdk.transponder.AdsbEmitterType "Link to this definition")
:   Bases: `Enum`

    ADSB classification for the type of vehicle emitting the transponder signal.

    ## Values[¶](#id1 "Link to this heading")

    NO\_INFO
    :   No emitter info.

    LIGHT
    :   Light emitter.

    SMALL
    :   Small emitter.

    LARGE
    :   Large emitter.

    HIGH\_VORTEX\_LARGE
    :   High vortex emitter.

    HEAVY
    :   Heavy emitter.

    HIGHLY\_MANUV
    :   Highly maneuverable emitter.

    ROTOCRAFT
    :   Rotorcraft emitter.

    UNASSIGNED
    :   Unassigned emitter.

    GLIDER
    :   Glider emitter.

    LIGHTER\_AIR
    :   Lighter air emitter.

    PARACHUTE
    :   Parachute emitter.

    ULTRA\_LIGHT
    :   Ultra light emitter.

    UNASSIGNED2
    :   Unassigned2 emitter.

    UAV
    :   UAV emitter.

    SPACE
    :   Space emitter.

    UNASSGINED3
    :   Unassigned3 emitter.

    EMERGENCY\_SURFACE
    :   Emergency emitter.

    SERVICE\_SURFACE
    :   Service surface emitter.

    POINT\_OBSTACLE
    :   Point obstacle emitter.

    EMERGENCY\_SURFACE *= 17*[¶](#mavsdk.transponder.AdsbEmitterType.EMERGENCY_SURFACE "Link to this definition")

    GLIDER *= 9*[¶](#mavsdk.transponder.AdsbEmitterType.GLIDER "Link to this definition")

    HEAVY *= 5*[¶](#mavsdk.transponder.AdsbEmitterType.HEAVY "Link to this definition")

    HIGHLY\_MANUV *= 6*[¶](#mavsdk.transponder.AdsbEmitterType.HIGHLY_MANUV "Link to this definition")

    HIGH\_VORTEX\_LARGE *= 4*[¶](#mavsdk.transponder.AdsbEmitterType.HIGH_VORTEX_LARGE "Link to this definition")

    LARGE *= 3*[¶](#mavsdk.transponder.AdsbEmitterType.LARGE "Link to this definition")

    LIGHT *= 1*[¶](#mavsdk.transponder.AdsbEmitterType.LIGHT "Link to this definition")

    LIGHTER\_AIR *= 10*[¶](#mavsdk.transponder.AdsbEmitterType.LIGHTER_AIR "Link to this definition")

    NO\_INFO *= 0*[¶](#mavsdk.transponder.AdsbEmitterType.NO_INFO "Link to this definition")

    PARACHUTE *= 11*[¶](#mavsdk.transponder.AdsbEmitterType.PARACHUTE "Link to this definition")

    POINT\_OBSTACLE *= 19*[¶](#mavsdk.transponder.AdsbEmitterType.POINT_OBSTACLE "Link to this definition")

    ROTOCRAFT *= 7*[¶](#mavsdk.transponder.AdsbEmitterType.ROTOCRAFT "Link to this definition")

    SERVICE\_SURFACE *= 18*[¶](#mavsdk.transponder.AdsbEmitterType.SERVICE_SURFACE "Link to this definition")

    SMALL *= 2*[¶](#mavsdk.transponder.AdsbEmitterType.SMALL "Link to this definition")

    SPACE *= 15*[¶](#mavsdk.transponder.AdsbEmitterType.SPACE "Link to this definition")

    UAV *= 14*[¶](#mavsdk.transponder.AdsbEmitterType.UAV "Link to this definition")

    ULTRA\_LIGHT *= 12*[¶](#mavsdk.transponder.AdsbEmitterType.ULTRA_LIGHT "Link to this definition")

    UNASSGINED3 *= 16*[¶](#mavsdk.transponder.AdsbEmitterType.UNASSGINED3 "Link to this definition")

    UNASSIGNED *= 8*[¶](#mavsdk.transponder.AdsbEmitterType.UNASSIGNED "Link to this definition")

    UNASSIGNED2 *= 13*[¶](#mavsdk.transponder.AdsbEmitterType.UNASSIGNED2 "Link to this definition")

*class* mavsdk.transponder.AdsbVehicle(*icao\_address*, *latitude\_deg*, *longitude\_deg*, *altitude\_type*, *absolute\_altitude\_m*, *heading\_deg*, *horizontal\_velocity\_m\_s*, *vertical\_velocity\_m\_s*, *callsign*, *emitter\_type*, *squawk*, *tslc\_s*)[¶](#mavsdk.transponder.AdsbVehicle "Link to this definition")
:   Bases: `object`

    ADSB Vehicle type.

    Parameters:
    :   * **icao\_address** (*uint32\_t*) – ICAO (International Civil Aviation Organization) unique worldwide identifier
        * **latitude\_deg** (*double*) – Latitude in degrees (range: -90 to +90)
        * **longitude\_deg** (*double*) – Longitude in degrees (range: -180 to +180).
        * **altitude\_type** ([*AdsbAltitudeType*](#mavsdk.transponder.AdsbAltitudeType "mavsdk.transponder.AdsbAltitudeType")) – ADSB altitude type.
        * **absolute\_altitude\_m** (*float*) – Altitude in metres according to altitude\_type
        * **heading\_deg** (*float*) – Course over ground, in degrees
        * **horizontal\_velocity\_m\_s** (*float*) – The horizontal velocity in metres/second
        * **vertical\_velocity\_m\_s** (*float*) – The vertical velocity in metres/second. Positive is up.
        * **callsign** (*std::string*) – The callsign
        * **emitter\_type** ([*AdsbEmitterType*](#mavsdk.transponder.AdsbEmitterType "mavsdk.transponder.AdsbEmitterType")) – ADSB emitter type.
        * **squawk** (*uint32\_t*) – Squawk code.
        * **tslc\_s** (*uint32\_t*) – Time Since Last Communication in seconds.

*class* mavsdk.transponder.Transponder(*async\_plugin\_manager*)[¶](#mavsdk.transponder.Transponder "Link to this definition")
:   Bases: `AsyncBase`

    Allow users to get ADS-B information
    and set ADS-B update rates.

    Generated by dcsdkgen - MAVSDK Transponder API

    name *= 'Transponder'*[¶](#mavsdk.transponder.Transponder.name "Link to this definition")

    *async* set\_rate\_transponder(*rate\_hz*)[¶](#mavsdk.transponder.Transponder.set_rate_transponder "Link to this definition")
    :   Set rate to ‘transponder’ updates.

        Parameters:
        :   **rate\_hz** (*double*) – The requested rate (in Hertz)

        Raises:
        :   [**TransponderError**](#mavsdk.transponder.TransponderError "mavsdk.transponder.TransponderError") – If the request fails. The error contains the reason for the failure.

    *async* transponder()[¶](#mavsdk.transponder.Transponder.transponder "Link to this definition")
    :   Subscribe to ‘transponder’ updates.

        Yields:
        :   **transponder** (*AdsbVehicle*) – The next detection

*exception* mavsdk.transponder.TransponderError(*result*, *origin*, *\*params*)[¶](#mavsdk.transponder.TransponderError "Link to this definition")
:   Bases: `Exception`

    Raised when a TransponderResult is a fail code

*class* mavsdk.transponder.TransponderResult(*result*, *result\_str*)[¶](#mavsdk.transponder.TransponderResult "Link to this definition")
:   Bases: `object`

    Result type.

    Parameters:
    :   * **result** ([*Result*](#mavsdk.transponder.TransponderResult.Result "mavsdk.transponder.TransponderResult.Result")) – Result enum value
        * **result\_str** (*std::string*) – Human-readable English string describing the result

    *class* Result(*value*)[¶](#mavsdk.transponder.TransponderResult.Result "Link to this definition")
    :   Bases: `Enum`

        Possible results returned for transponder requests.

        ## Values[¶](#id2 "Link to this heading")

        UNKNOWN
        :   Unknown result

        SUCCESS
        :   Success: the transponder command was accepted by the vehicle

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

        BUSY *= 4*[¶](#mavsdk.transponder.TransponderResult.Result.BUSY "Link to this definition")

        COMMAND\_DENIED *= 5*[¶](#mavsdk.transponder.TransponderResult.Result.COMMAND_DENIED "Link to this definition")

        CONNECTION\_ERROR *= 3*[¶](#mavsdk.transponder.TransponderResult.Result.CONNECTION_ERROR "Link to this definition")

        NO\_SYSTEM *= 2*[¶](#mavsdk.transponder.TransponderResult.Result.NO_SYSTEM "Link to this definition")

        SUCCESS *= 1*[¶](#mavsdk.transponder.TransponderResult.Result.SUCCESS "Link to this definition")

        TIMEOUT *= 6*[¶](#mavsdk.transponder.TransponderResult.Result.TIMEOUT "Link to this definition")

        UNKNOWN *= 0*[¶](#mavsdk.transponder.TransponderResult.Result.UNKNOWN "Link to this definition")