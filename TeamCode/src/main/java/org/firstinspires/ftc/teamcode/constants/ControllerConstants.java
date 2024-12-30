package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for Controllers
 */
public interface ControllerConstants {
    /** Deadband Magnitude of all Axes */
    double AXIS_DEADBAND = 0.01;

    /** Wait time for if the Axis has been zero */
    double WAIT_TIME = 0.5;

    /**
     * If true, enables telemetry for all mechanisms.
     */
    boolean CONTROLLER_DEBUG_MODE = false;

    /**
     * If true, enables PID for all mechanisms.
     */
    boolean USE_PID = true;
}