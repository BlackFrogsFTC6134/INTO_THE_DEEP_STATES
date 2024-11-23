package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadState {
    public boolean a1, b1, x1, y1, dpadUp1, dpadDown1, dpadLeft1, dpadRight1;
    public boolean a2, b2, x2, y2, dpadUp2, dpadDown2, dpadLeft2, dpadRight2;
    public float leftStickX1, leftStickY1, rightStickX1, rightStickY1;
    public float leftStickX2, leftStickY2, rightStickX2, rightStickY2;
    // Add other gamepad inputs as needed

    public void updateFromGamepad(Gamepad gamepad1, Gamepad gamepad2) {
        a1 = gamepad1.a;
        b1 = gamepad1.b;
        x1 = gamepad1.x;
        y1 = gamepad1.y;
        dpadUp1 = gamepad1.dpad_up;
        dpadDown1 = gamepad1.dpad_down;
        dpadLeft1 = gamepad1.dpad_left;
        dpadRight1 = gamepad1.dpad_right;
        leftStickX1 = gamepad1.left_stick_x;
        rightStickX1 = gamepad1.right_stick_x;
        leftStickY1 = gamepad1.left_stick_y;
        rightStickY1 = gamepad1.right_stick_y;

        // Update other gamepad inputs
        a2 = gamepad2.a;
        b2 = gamepad2.b;
        x2 = gamepad2.x;
        y2 = gamepad2.y;
        dpadUp2 = gamepad2.dpad_up;
        dpadDown2 = gamepad2.dpad_down;
        dpadLeft2 = gamepad2.dpad_left;
        dpadRight2 = gamepad2.dpad_right;
        leftStickX2 = gamepad2.left_stick_x;
        rightStickX2 = gamepad2.right_stick_x;
        leftStickY2 = gamepad2.left_stick_y;
        rightStickY2 = gamepad2.right_stick_y;
    }
}
