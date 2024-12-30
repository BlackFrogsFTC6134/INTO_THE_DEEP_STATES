package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.LED;

@TeleOp(name="Demo_Blinkin_LED", group="Concept")

public class Demo_Blinkin_LED extends LinearOpMode {
    private LED led;

    public Demo_Blinkin_LED () throws InterruptedException {
        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            if (gamepad2.b) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                sleep(250);
            }

            if (gamepad2.a) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                sleep(250);
            }

            if (gamepad2.x) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                sleep(250);
            }

            if (gamepad2.y) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                sleep(250);
            }
        }
    }
}
