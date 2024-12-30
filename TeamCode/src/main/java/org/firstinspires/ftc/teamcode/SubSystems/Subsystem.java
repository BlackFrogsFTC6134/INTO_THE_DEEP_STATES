package org.firstinspires.ftc.teamcode.SubSystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public interface Subsystem {
    /**
     * OpMode context for a Mechanism class.
     */
    LinearOpMode opMode = null;

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     *
     * @param hardwareMap robot's hardware map
     * @throws RuntimeException if something goes wrong
     */
    public abstract void initialize(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException;

    /**
     * Manages multiple gamepad inputs and their corresponding mechanism responses
     * Implement when using two gamepads
     * @param gamepad = null
     */
    void update(Gamepad gamepad, Telemetry telemetry);

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     * Where all telemetry.addData() calls should go
     * @param telemetry = null
     */
    public void telemetry(Telemetry telemetry);

    /**
     * stopSubsystem()
     * @param telemetry = null
     */
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException;
}
