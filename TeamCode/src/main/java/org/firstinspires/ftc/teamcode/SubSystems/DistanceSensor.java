package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor implements Subsystem {
    private com.qualcomm.robotcore.hardware.DistanceSensor LeftFrontDistanceSensor;
    private com.qualcomm.robotcore.hardware.DistanceSensor RightFrontDistanceSensor;
    private LED led;
    public double[] DistanceValues;
    private final Telemetry telemetry;

    public DistanceSensor(HardwareMap hardwareMap, LED led, Telemetry telemetry) {
        this.led = led;
        this.telemetry = telemetry;

        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        LeftFrontDistanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "LeftFrontDistanceCH12Cbus3-3");
        RightFrontDistanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "RightFrontDistanceEH12Cbus1-1");
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
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
    public void update(Gamepad gamepad, Telemetry telemetry) {
        // No continuous update needed for distance sensors
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
    public void stopSubsystem(Telemetry telemetry) {
        // No stop action needed for distance sensors
    }
}
