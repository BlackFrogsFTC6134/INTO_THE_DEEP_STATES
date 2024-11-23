package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem implements Subsystem {
    private DistanceSensor LeftFrontDistanceSensor;
    private DistanceSensor RightFrontDistanceSensor;
    private LEDSubsystem led;
    public double[] DistanceValues;

    public DistanceSensorSubsystem(LEDSubsystem led) {
        this.led = led;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        LeftFrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "LeftFrontDistanceCH12Cbus3-3");
        RightFrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "RightFrontDistanceEH12Cbus1-1");
    }

    public double getLeftDistance() {
        DistanceValues[0] = LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH);
        return LeftFrontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double getRightDistance() {
        DistanceValues[1] = RightFrontDistanceSensor.getDistance(DistanceUnit.INCH);
        return RightFrontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void update(HardwareMap hardwareMap, Gamepad gamepadState1, Gamepad gamepadState2) {
        // No continuous update needed for distance sensors
    }

    @Override
    public void stopSubsystem(HardwareMap hardwareMap) {
        // No stop action needed for distance sensors
    }
}
