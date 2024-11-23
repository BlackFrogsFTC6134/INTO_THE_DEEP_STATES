package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDSubsystem implements Subsystem {
    private RevBlinkinLedDriver Blinkin12;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    @Override
    public void initialize(HardwareMap hardwareMap) {
       Blinkin12 = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin12");
       currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK; // Default pattern
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        Blinkin12.setPattern(pattern);
        currentPattern = pattern; // Store the current pattern
    }

    // Method to get the current pattern
    public RevBlinkinLedDriver.BlinkinPattern getCurrentPattern() {
        return currentPattern;
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {

    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        Blinkin12.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
