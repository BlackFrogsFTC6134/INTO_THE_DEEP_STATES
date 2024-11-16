package org.firstinspires.ftc.teamcode.states.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Demo_BFR_ClawControl", group="Demo")
public class BFR_Demo_ClawControl extends LinearOpMode {

    private Servo clawServo;
    private static final double CLAW_OPEN_POSITION = 1.0;   /// Fully open position, (ready to pick sample)
    private static final double CLAW_CLOSED_POSITION = 0.0; // Fully closed position, (holding sample)
    private static final long SERVO_MOVE_DELAY_MS = 0;    // Delay to allow servo to move

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the claw servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions
        clawServo.setPosition(CLAW_OPEN_POSITION);

        telemetry.addLine(">> clawServo: Initialized");
        telemetry.addData("Claw Position ", CLAW_OPEN_POSITION);
        telemetry.addData("gamepad2.left_bumper", "Opens The claw");
        telemetry.addData("gamepad2.right_bumper", "Closes the claw");
       // telemetry.addData("gamepad2.right_trigger", "Fine control the claw");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                closeClaw();
            } else if (gamepad2.left_bumper) {
                openClaw();
            }
/*
            // Optionally, use triggers for fine control
            double triggerPosition = gamepad2.right_trigger * (CLAW_OPEN_POSITION - CLAW_CLOSED_POSITION) + CLAW_CLOSED_POSITION;
            clawServo.setPosition(triggerPosition);
*/
            telemetry.addData("Claw Position", clawServo.getPosition());
        }
    }

    private void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    private void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }
}