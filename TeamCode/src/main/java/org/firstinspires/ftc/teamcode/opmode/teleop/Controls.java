package org.firstinspires.ftc.teamcode.opmode.teleop;
import org.firstinspires.ftc.teamcode.utils.GamepadStatic;

public class Controls extends GamepadStatic {
    public static final Input GRAB = Input.RIGHT_BUMPER;
    public static final Input RELEASE = Input.LEFT_BUMPER;
    public static final Input INTAKE = Input.A; // CROSS

    public static final Input PIVOT_HIGH_BASKET = Input.DPAD_UP;
    public static final Input PIVOT_DOWN = Input.DPAD_DOWN;
    public static final Input PIVOT_HIGH_CHAMBER  = Input.DPAD_LEFT;
    public static final Input PIVOT_CENTER = Input.DPAD_RIGHT;

    public static final Input TELE_EXTEND = Input.RIGHT_TRIGGER;
    public static final Input TELE_RETRACT = Input.LEFT_TRIGGER;
    public static final Input WRIST_LEFT = Input.DPAD_LEFT;
    public static final Input WRIST_RIGHT = Input.DPAD_RIGHT;
}
