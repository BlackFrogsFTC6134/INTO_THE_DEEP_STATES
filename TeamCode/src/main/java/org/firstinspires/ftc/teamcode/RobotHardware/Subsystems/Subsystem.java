package org.firstinspires.ftc.teamcode.RobotHardware.Subsystems;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotParams;

/**
 * Author: Kavish Karia
 * Date create: 12/28/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */

public interface Subsystem {
    /**
     * initialize(HardwareMap hardwareMap,Telemetry telemetry) function is used to initialize the subsystem
     * @param hardwareMap = hardwareMap
     * @param  telemetry = telemetry
     */
    public abstract void initialize(HardwareMap hardwareMap, Telemetry telemetry);

    /**
     * sendTelemetry(Telemetry telemetry) function is used to send telemetry to the DS for debugging
     * @param telemetry = telemetry
     */
    public abstract void sendTelemetry(Telemetry telemetry);

    /**
     * periodic(Gamepad gamepad) function is used to map gamepad inputs to certain actions
     * @param gamepad1 = gamepad1
     * @param gamepad2 = gamepad2
     * @param telemetry = telemetry
     */
    public abstract void periodic(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry);

    /**
     * stopSubsystem(Telemetry telemetry) applies zero power to the subsystem
     * @param telemetry = telemetry
     */
    public abstract void stopSubsystem(Telemetry telemetry);
}

