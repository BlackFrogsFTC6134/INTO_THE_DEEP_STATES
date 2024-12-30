package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Author: Kavish Karia
 * Date create: 12/15/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */
@Config
public final class PivotConstants {
    /**
     * The name of the pivot motor configured in Driver Station
     */
    public static String pivotMotorName = "pivoteh1";

    /**
     * The name of the left pivot servo configured in Driver Station
     */
    public static String pivotServoLeftName = "pivotleh3";

    /**
     * The name of the right pivot servo configured in Driver Station
     */
    public static String pivotServoRightName = "pivotreh2";

    /**
     * If true, enables PID for pivot.
     */
    public static boolean PIVOT_MOTOR_USE_PID             = false;

    /**
     * The KP for the pivot motor
     */
    public static double PIVOT_KP                 = 0.0007;

    /**
     * The KI for the pivot motor
     */
    public static double PIVOT_KI                 = 0.04;

    /**
     * The Kd for the pivot motor
     */
    public static double PIVOT_KD                 = 0.00006;

    /**
     * The KF for the pivot motor
     */
    public static double PIVOT_KF                 = 0.2;

    /**
     * The RPM of the pivot motor
     */
    public static final double PIVOT_MOTOR_RPM                  = 435;

    /**
     * The number of ticks per revolution of the pivot motor
     */
    public static final double PIVOT_MOTOR_TICKS_PER_REV = 384.5; //TODO: Check if this is correct


    /**
     * The timeout for the pivot in milliseconds.
     */
    public static double PIVOT_TIMEOUT_MS = 10000;

    /**
     * The minimum allowable position to which a left PivotSubsystem servo can be moved
     */
    public static double MIN_LEFT_PIVOT_POS =  0.0;

    /**
     * The maximum allowable position to which a left PivotSubsystem servo can be moved
     */
    public static double MAX_LEFT_PIVOT_POS =  1.0;

    /**
     * The minimum allowable position to which a right PivotSubsystem servo can be moved
     */
    public static double MIN_RIGHT_PIVOT_POS              =  0.0;

    /**
     * The maximum allowable position to which a right PivotSubsystem servo can be moved
     */
    public static double MAX_RIGHT_PIVOT_POS              =  1.0;

    /**
     * The power to which the pivot motor is set.
     * Min -1.0, Max 1.0
     * Negative power will reverse the motor and positive power will forward the motor
     */
    public static double PIVOT_MOTOR_POWER                = 1.0;

    /**
     * The velocity of the pivot motor
     */
    public static double PIVOT_MOTOR_VELOCITY                = 1000;

    /**
     * The tolerance for the pivot position
     */
    public static double PIVOT_POSITION_TOLERANCE         = 0.1;

    /**
     * If true, enables telemetry for pivot. GLOBAL_DEBUG_MODE overrides this value
     */
    public static boolean PIVOT_DEBUG_MODE                = false;

    /**
     * INIT_POS is the position of the pivot arm that touches the ground
     */
    public static int INIT_POS                                  = 0;

    /**
     * PIVOT_DOWN_POS is the position that will be used to grab samples from the submersible and finishing position after low chamber
     */
    public static int PIVOT_DOWN_POS                            = 250;

    /**
     * PIVOT_CENTER_POS is the position used to grab the specimen from the human player and the position to finish after hanging the specimen on the high chamber
     */
    public static int PIVOT_CENTER_POS                          = 1100;

    /**
     * PIVOT_LOW_CHAMBER_POS is the position needed to hang the specimen on the low chamber
     */
    public static int PIVOT_LOW_CHAMBER_POS                     = 1000;

    /**
     * PIVOT_HIGH_CHAMBER_POS is the position needed to hang the specimen on the high chamber
     */
    public static int PIVOT_HIGH_CHAMBER_POS                    = 1650;

    /**
     * PIVOT_LOW_BASKET_POS is the position needed to place the sample on the low basket
     */
    public static int PIVOT_LOW_BASKET_POS                      = 1350;

    /**
     * PIVOT_HIGH_BASKET_POS is the position needed to place the sample on the high basket
     */
    public static int PIVOT_HIGH_BASKET_POS                     = 2200;
}
