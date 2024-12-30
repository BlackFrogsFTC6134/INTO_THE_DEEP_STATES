package org.firstinspires.ftc.teamcode.constants;

/**
 * Author: Kavish Karia
 * Date create: 12/15/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */
public final class ClawConstants {
    /**
     * The name of the left claw servo configured in Driver Station
     */
    public static final String clawServoLeftName        = "clawleh1";

    /**
     * The name of the right claw servo configured in Driver Station
     */
    public static final String clawServoRightName       = "clawreh0";

    /**
     * The tolerance for the claw position
     */
    public static final double CLAW_POSITION_TOLERANCE  = 0.1;

    public enum ClawState {
        HOLD,
        RELEASED,
        CENTER
    }

    /**
     * The position of the claw when it is grabbed
     */
    public static final double CLAW_OPEN_POSITION = 0.0;

    /**
     * The position of the claw when it is released
     */
    public static final double CLAW_CLOSE_POSITION = 1.0;

    /**
     * The minimum allowable position to which a servo can be moved
     */
    public static final double MIN_CLAW_POS =  0.0;     // Minimum rotational position

    /**
     * The maximum allowable position to which a servo can be moved
     */
    public static final double MAX_CLAW_POS =  1.0;     // Maximum rotational position

    /**
     * If true, enables telemetry for claw. GLOBAL_DEBUG_MODE overrides this value
     */
    public static final boolean CLAW_DEBUG_MODE         = false;

    /**
     * The timeout for the claw servo to open / close in milliseconds.
     */
    public static final double CLAW_TIMEOUT_MS           = 10000;
}
