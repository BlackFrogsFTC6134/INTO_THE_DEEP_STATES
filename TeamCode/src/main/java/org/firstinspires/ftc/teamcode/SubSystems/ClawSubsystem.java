package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends LinearOpMode implements Subsystem {
    private Servo clawRight, clawLeft;
    private LEDSubsystem led;
    private static final double POSITION_TOLERANCE = 0.02;
    private double targetPositionOpen = 0.5;
    private double targetPositionClosed = 0.0;

    public ClawSubsystem(LEDSubsystem led) {
        this.led = led;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

        led = new LEDSubsystem();
        led.initialize(hardwareMap);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        clawRight = hardwareMap.get(Servo.class, "clawreh0");
        clawLeft = hardwareMap.get(Servo.class, "clawleh1");

        clawRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
/*
        telemetry.addData("Claws initialized: ", "successfully");
        telemetry.update(); */
    }

    public void openClaw() {
        clawRight.setPosition(targetPositionOpen);
        clawLeft.setPosition(targetPositionOpen); /*
        if(isAtTargetPosition(targetPositionOpen)){
            telemetry.addData("Claw position: ", "Open");
        } */
    }

    public void closeClaw() {
        clawRight.setPosition(targetPositionClosed);
        clawLeft.setPosition(targetPositionClosed);  /*
        if(isAtTargetPosition(targetPositionClosed)){
            telemetry.addData("Claw position: ", "Closed");
        }  */
    }

    public boolean isAtTargetPosition(double targetPosition) {
        return Math.abs(clawRight.getPosition() - targetPosition) <= POSITION_TOLERANCE &&
                Math.abs(clawLeft.getPosition() - targetPosition) <= POSITION_TOLERANCE;
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // Update logic if needed
        /*
        telemetry.addData("Left claw position: ", clawLeft.getPosition());
        telemetry.addData("Right claw position: ", clawRight.getPosition());
        telemetry.update();*/
    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        openClaw();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
