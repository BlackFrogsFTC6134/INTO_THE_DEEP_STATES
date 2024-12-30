package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;

public class Climber implements Subsystem {
    private Servo ClimberHookEh4;
    private LED led;
    private final Telemetry telemetry;

    public Climber(HardwareMap hardwareMap, LED led, Telemetry telemetry) {
        this.led = led;
        this.telemetry = telemetry;

        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        ClimberHookEh4 = hardwareMap.get(Servo.class, "ClimberHookEh4");
        ClimberHookEh4.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

    }

    public void setPosition(double position) {
        ClimberHookEh4.setPosition(position);
    }

    @Override
    public void update(Gamepad gamepad2, Telemetry telemetry) {
        // No continuous update needed for servo
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
    public void stopSubsystem(Telemetry telemetry) throws InterruptedException{
        // No stop action needed for servo
    }
}