package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;


/**
 * Mechanism is an abstract class for all mechanisms on a robot. It contains methods and/or instance
 * variables common to all mechanisms.
 * All robot mechanisms, including the main hardware map, should extend this abstract class.
 *
 * @version 1.0
 * @since 1.0.0-alpha
 *
 * @author Paul Serbanescu (paulserbanescu3@gmail.com)
 */
public abstract class Mechanism {

    /**
     * OpMode context for a Mechanism class.
     */
    protected LinearOpMode opMode;

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     *
     * @param hardwareMap robot's hardware map
     * @throws RuntimeException if something goes wrong
     */
    public abstract void init(HardwareMap hardwareMap);

    /**
     * Manages gamepad inputs and their corresponding mechanism response
     * Implement when using only one gamepad, in slot 1
     * @param gamepad = null
     */
    public void loop(Gamepad gamepad) { }

    /**
     * Manages multiple gamepad inputs and their corresponding mechanism responses
     * Implement when using two gamepads
     * @param gamepad1 = null
     * @param gamepad2 = null
     */
    public void loop(Gamepad gamepad1, Gamepad gamepad2) { }

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     * Where all telemetry.addData() calls should go
     * @param telemetry = null
     */
    public void telemetry(Telemetry telemetry) { }

    @NonNull
    @Override
    public String toString(){
        return String.format(Locale.US, "(%.2f)", 0.0);
    }
}

