package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ClawSubsystem;

@TeleOp (name = "ClawSubsystem Dev", group = "dev")
public class ClawDev extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     */

    private ClawSubsystem clawSubsystem = new ClawSubsystem(this);

    @Override
    public void runOpMode() throws InterruptedException {

        clawSubsystem.initialize(hardwareMap, telemetry);

    waitForStart();

    while(opModeIsActive() && !isStopRequested()){
        clawSubsystem.update(gamepad2, telemetry);
        clawSubsystem.telemetry(telemetry);
        sleep(50);
    }
    }
}
