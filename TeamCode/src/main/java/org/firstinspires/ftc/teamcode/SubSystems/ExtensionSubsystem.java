package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtensionSubsystem extends LinearOpMode implements Subsystem {
    private DcMotorEx extensionMotor1, extensionMotor2;
    private LEDSubsystem led;

    public ExtensionSubsystem(LEDSubsystem led) {
        this.led = led;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, "extensioneh0");
        extensionMotor1.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor2 = hardwareMap.get(DcMotorEx.class, "2extensionEh2");
        extensionMotor2.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPosition(int position) {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1865));
        extensionMotor1.setTargetPosition(position);
        extensionMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor1.setPower(1.0);
        sleep(4500);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
    }

    public void resetEncoder(){
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1815));
        extensionMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // Update logic if needed
        extensionMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor1.setPower(-gamepad2.left_stick_y);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        extensionMotor1.setPower(0);
//        ledSubsystem.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK); // Optional: Turn off LEDs when stopped.
    }
}
