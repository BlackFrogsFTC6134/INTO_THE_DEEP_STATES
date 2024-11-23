package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClimberSubsystem extends LinearOpMode implements Subsystem {
    private Servo ClimberHookEh4;
    private LEDSubsystem led;

    public ClimberSubsystem(LEDSubsystem led) {
        this.led = led;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        ClimberHookEh4 = hardwareMap.get(Servo.class, "ClimberHookEh4");
        ClimberHookEh4.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        ClimberHookEh4.setPosition(position);
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // No continuous update needed for servo
    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        // No stop action needed for servo
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}