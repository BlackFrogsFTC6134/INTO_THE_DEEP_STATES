package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED implements Subsystem {
    private RevBlinkinLedDriver Blinkin12;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;
    private int sleepTime = 0; //In msecs
    private Telemetry telemetry;

    public LED(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
       Blinkin12 = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin12");
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        Blinkin12.setPattern(pattern);
        currentPattern = pattern; // Store the current pattern
    }

    // Method to get the current pattern
    public RevBlinkinLedDriver.BlinkinPattern getCurrentPattern() {
        return currentPattern;
    }

    @Override
    public void update(Gamepad gamepad, Telemetry telemetry) {

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
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        sleep(sleepTime);
    }
}
