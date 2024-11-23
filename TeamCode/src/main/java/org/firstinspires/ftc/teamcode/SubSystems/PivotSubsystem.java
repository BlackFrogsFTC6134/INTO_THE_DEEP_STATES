package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PivotSubsystem extends LinearOpMode implements Subsystem {
    private DcMotorEx pivotMotor;
    private Servo pivotreh2;
    private Servo pivotleh3;
    private LEDSubsystem led;

    public PivotSubsystem(LEDSubsystem led) {
        this.led = led;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

        led = new LEDSubsystem();
        led.initialize(hardwareMap);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivoteh1");
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pivotreh2 = hardwareMap.get(Servo.class, "pivotreh2");
        pivotreh2.setDirection(Servo.Direction.FORWARD);

        pivotleh3 = hardwareMap.get(Servo.class, "pivotleh3");
        pivotleh3.setDirection(Servo.Direction.FORWARD);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }

    public void setMotorPosition(HardwareMap hardwareMap, int position) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(1.0);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
    }

    public void setServoPosition(HardwareMap hardwareMap, double position) {
        pivotreh2.setPosition(position);
        pivotleh3.setPosition(position);
    }

    public void setPivotMotorPosition(HardwareMap hardwareMap,int position, double power) {
        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(power);
    }

    public void setPivotMotorPower(HardwareMap hardwareMap, double power){
        pivotMotor.setPower(power);
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // Update logic if needed
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setPower(-gamepad2.right_stick_y);
    }

    public void resetEncoder(HardwareMap hardwareMap){
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1815));
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }
    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        pivotMotor.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}