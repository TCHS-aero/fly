[Original Source](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/system.html)

# System[¶](#module-mavsdk.system "Link to this heading")

*class* mavsdk.system.System(*mavsdk\_server\_address=None*, *port=50051*, *sysid=245*, *compid=190*)[¶](#mavsdk.system.System "Link to this definition")
:   Bases: `object`

    Instantiate a System object, that will serve as a proxy to
    all the MAVSDK plugins.

    Parameters:
    :   * **mavsdk\_server\_address** (*str*) – Address of a running mavsdk\_server instance. If None,
          an instance of mavsdk\_server will be automatically
          started (on localhost).
        * **port** (*int*) – Port of the running mavsdk\_server instance specified by
          mavsdk\_server\_address.
        * **sysid** (*int*) – MAVLink system ID of the mavsdk\_server (1..255).
        * **compid** (*int*) – MAVLink component ID of the mavsdk\_server (1..255).

    *property* action*: [Action](plugins/action.html#mavsdk.action.Action "mavsdk.action.Action")*[¶](#mavsdk.system.System.action "Link to this definition")

    *property* action\_server*: [ActionServer](plugins/action_server.html#mavsdk.action_server.ActionServer "mavsdk.action_server.ActionServer")*[¶](#mavsdk.system.System.action_server "Link to this definition")

    *property* arm\_authorizer\_server*: [ArmAuthorizerServer](plugins/arm_authorizer_server.html#mavsdk.arm_authorizer_server.ArmAuthorizerServer "mavsdk.arm_authorizer_server.ArmAuthorizerServer")*[¶](#mavsdk.system.System.arm_authorizer_server "Link to this definition")

    *property* calibration*: [Calibration](plugins/calibration.html#mavsdk.calibration.Calibration "mavsdk.calibration.Calibration")*[¶](#mavsdk.system.System.calibration "Link to this definition")

    *property* camera*: [Camera](plugins/camera.html#mavsdk.camera.Camera "mavsdk.camera.Camera")*[¶](#mavsdk.system.System.camera "Link to this definition")

    *property* camera\_server*: [CameraServer](plugins/camera_server.html#mavsdk.camera_server.CameraServer "mavsdk.camera_server.CameraServer")*[¶](#mavsdk.system.System.camera_server "Link to this definition")

    *property* component\_information*: [ComponentInformation](plugins/component_information.html#mavsdk.component_information.ComponentInformation "mavsdk.component_information.ComponentInformation")*[¶](#mavsdk.system.System.component_information "Link to this definition")

    *property* component\_information\_server*: [ComponentInformationServer](plugins/component_information_server.html#mavsdk.component_information_server.ComponentInformationServer "mavsdk.component_information_server.ComponentInformationServer")*[¶](#mavsdk.system.System.component_information_server "Link to this definition")

    *property* component\_metadata*: [ComponentMetadata](plugins/component_metadata.html#mavsdk.component_metadata.ComponentMetadata "mavsdk.component_metadata.ComponentMetadata")*[¶](#mavsdk.system.System.component_metadata "Link to this definition")

    *property* component\_metadata\_server*: [ComponentMetadataServer](plugins/component_metadata_server.html#mavsdk.component_metadata_server.ComponentMetadataServer "mavsdk.component_metadata_server.ComponentMetadataServer")*[¶](#mavsdk.system.System.component_metadata_server "Link to this definition")

    *async* connect(*system\_address=None*)[¶](#mavsdk.system.System.connect "Link to this definition")
    :   Connect the System object to a remote system.

        Parameters:
        :   **system\_address** (*str*) –

            The address of the remote system. If None, it will
            default to udpin://0.0.0.0:14540. Supported URL formats:

            > * Serial: serial:///path/to/serial/dev[:baudrate]
            > * UDP in: udpin://bind\_host:bind\_port
            > * UDP out: udpout://dest\_host:dest\_port
            > * TCP in: tcpin://bind\_host:bind\_port
            > * TCP out: tcpout://dest\_host:dest\_port

    *property* core*: [Core](plugins/core.html#mavsdk.core.Core "mavsdk.core.Core")*[¶](#mavsdk.system.System.core "Link to this definition")

    *static* error\_uninitialized(*plugin\_name: str*) → str[¶](#mavsdk.system.System.error_uninitialized "Link to this definition")

    *property* events*: [Events](plugins/events.html#mavsdk.events.Events "mavsdk.events.Events")*[¶](#mavsdk.system.System.events "Link to this definition")

    *property* failure*: [Failure](plugins/failure.html#mavsdk.failure.Failure "mavsdk.failure.Failure")*[¶](#mavsdk.system.System.failure "Link to this definition")

    *property* follow\_me*: [FollowMe](plugins/follow_me.html#mavsdk.follow_me.FollowMe "mavsdk.follow_me.FollowMe")*[¶](#mavsdk.system.System.follow_me "Link to this definition")

    *property* ftp*: [Ftp](plugins/ftp.html#mavsdk.ftp.Ftp "mavsdk.ftp.Ftp")*[¶](#mavsdk.system.System.ftp "Link to this definition")

    *property* ftp\_server*: [FtpServer](plugins/ftp_server.html#mavsdk.ftp_server.FtpServer "mavsdk.ftp_server.FtpServer")*[¶](#mavsdk.system.System.ftp_server "Link to this definition")

    *property* geofence*: [Geofence](plugins/geofence.html#mavsdk.geofence.Geofence "mavsdk.geofence.Geofence")*[¶](#mavsdk.system.System.geofence "Link to this definition")

    *property* gimbal*: [Gimbal](plugins/gimbal.html#mavsdk.gimbal.Gimbal "mavsdk.gimbal.Gimbal")*[¶](#mavsdk.system.System.gimbal "Link to this definition")

    *property* gripper*: [Gripper](plugins/gripper.html#mavsdk.gripper.Gripper "mavsdk.gripper.Gripper")*[¶](#mavsdk.system.System.gripper "Link to this definition")

    *property* info*: [Info](plugins/info.html#mavsdk.info.Info "mavsdk.info.Info")*[¶](#mavsdk.system.System.info "Link to this definition")

    *property* log\_files*: [LogFiles](plugins/log_files.html#mavsdk.log_files.LogFiles "mavsdk.log_files.LogFiles")*[¶](#mavsdk.system.System.log_files "Link to this definition")

    *property* log\_streaming*: [LogStreaming](plugins/log_streaming.html#mavsdk.log_streaming.LogStreaming "mavsdk.log_streaming.LogStreaming")*[¶](#mavsdk.system.System.log_streaming "Link to this definition")

    *property* manual\_control*: [ManualControl](plugins/manual_control.html#mavsdk.manual_control.ManualControl "mavsdk.manual_control.ManualControl")*[¶](#mavsdk.system.System.manual_control "Link to this definition")

    *property* mavlink\_direct*: [MavlinkDirect](plugins/mavlink_direct.html#mavsdk.mavlink_direct.MavlinkDirect "mavsdk.mavlink_direct.MavlinkDirect")*[¶](#mavsdk.system.System.mavlink_direct "Link to this definition")

    *property* mission*: [Mission](plugins/mission.html#mavsdk.mission.Mission "mavsdk.mission.Mission")*[¶](#mavsdk.system.System.mission "Link to this definition")

    *property* mission\_raw*: [MissionRaw](plugins/mission_raw.html#mavsdk.mission_raw.MissionRaw "mavsdk.mission_raw.MissionRaw")*[¶](#mavsdk.system.System.mission_raw "Link to this definition")

    *property* mission\_raw\_server*: [MissionRawServer](plugins/mission_raw_server.html#mavsdk.mission_raw_server.MissionRawServer "mavsdk.mission_raw_server.MissionRawServer")*[¶](#mavsdk.system.System.mission_raw_server "Link to this definition")

    *property* mocap*: [Mocap](plugins/mocap.html#mavsdk.mocap.Mocap "mavsdk.mocap.Mocap")*[¶](#mavsdk.system.System.mocap "Link to this definition")

    *property* offboard*: [Offboard](plugins/offboard.html#mavsdk.offboard.Offboard "mavsdk.offboard.Offboard")*[¶](#mavsdk.system.System.offboard "Link to this definition")

    *property* param*: [Param](plugins/param.html#mavsdk.param.Param "mavsdk.param.Param")*[¶](#mavsdk.system.System.param "Link to this definition")

    *property* param\_server*: [ParamServer](plugins/param_server.html#mavsdk.param_server.ParamServer "mavsdk.param_server.ParamServer")*[¶](#mavsdk.system.System.param_server "Link to this definition")

    *property* rtk*: [Rtk](plugins/rtk.html#mavsdk.rtk.Rtk "mavsdk.rtk.Rtk")*[¶](#mavsdk.system.System.rtk "Link to this definition")

    *property* server\_utility*: [ServerUtility](plugins/server_utility.html#mavsdk.server_utility.ServerUtility "mavsdk.server_utility.ServerUtility")*[¶](#mavsdk.system.System.server_utility "Link to this definition")

    *property* shell*: [Shell](plugins/shell.html#mavsdk.shell.Shell "mavsdk.shell.Shell")*[¶](#mavsdk.system.System.shell "Link to this definition")

    *property* telemetry*: [Telemetry](plugins/telemetry.html#mavsdk.telemetry.Telemetry "mavsdk.telemetry.Telemetry")*[¶](#mavsdk.system.System.telemetry "Link to this definition")

    *property* telemetry\_server*: [TelemetryServer](plugins/telemetry_server.html#mavsdk.telemetry_server.TelemetryServer "mavsdk.telemetry_server.TelemetryServer")*[¶](#mavsdk.system.System.telemetry_server "Link to this definition")

    *property* tracking\_server*: [TrackingServer](plugins/tracking_server.html#mavsdk.tracking_server.TrackingServer "mavsdk.tracking_server.TrackingServer")*[¶](#mavsdk.system.System.tracking_server "Link to this definition")

    *property* transponder*: [Transponder](plugins/transponder.html#mavsdk.transponder.Transponder "mavsdk.transponder.Transponder")*[¶](#mavsdk.system.System.transponder "Link to this definition")

    *property* tune*: [Tune](plugins/tune.html#mavsdk.tune.Tune "mavsdk.tune.Tune")*[¶](#mavsdk.system.System.tune "Link to this definition")

    *property* winch*: [Winch](plugins/winch.html#mavsdk.winch.Winch "mavsdk.winch.Winch")*[¶](#mavsdk.system.System.winch "Link to this definition")