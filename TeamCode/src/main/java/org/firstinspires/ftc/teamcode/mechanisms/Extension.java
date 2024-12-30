package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;

public class Extension implements Subsystem {
    private DcMotorEx extensionMotor1, extensionMotor2;
    private LED led;
    private final Telemetry telemetry;

    public Extension(HardwareMap hardwareMap, LED led, Telemetry telemetry) {
        this.led = led;
        this.telemetry = telemetry;

        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

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

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

    }

    public void setPosition(int position) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1865));
        extensionMotor1.setTargetPosition(position);
        extensionMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor1.setPower(1.0);
        sleep(4500);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1845));
    }

    public void resetEncoder() throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1815));
        extensionMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    @Override
    public void update(Gamepad gamepad2, Telemetry telemetry) {
        // Update logic if needed
        //extensionMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //extensionMotor1.setPower(-gamepad2.left_stick_y);
    }

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     * Where all telemetry.addData() calls should go
     *
     * @param telemetry = null
     */
    @Override
    public void telemetry(Telemetry telemetry) {

    }

    @Override
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException {
        extensionMotor1.setPower(0);
//        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK); // Optional: Turn off LEDs when stopped.
    }
}
