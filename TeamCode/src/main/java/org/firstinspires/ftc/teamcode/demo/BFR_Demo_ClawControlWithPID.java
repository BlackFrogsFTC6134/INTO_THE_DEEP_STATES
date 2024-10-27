package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Demo_BFR_ClawControlWithPID", group="Demo")
public class BFR_Demo_ClawControlWithPID extends LinearOpMode {

    private Servo clawServo;
    private static final double CLAW_OPEN_POSITION = 1.0;   /// Fully open position, (ready to pick sample)
    private static final double CLAW_CLOSED_POSITION = 0.0; // Fully closed position, (holding sample)
    private PIDController pidController;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.05;

    private double integralClaw = 0;
    private double previousErrorClaw = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the claw servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions
        clawServo.setPosition(CLAW_OPEN_POSITION);

        // Initialize PID Controller with appropriate gains
        pidController = new PIDController(0.1, 0.01, 0.05);

        telemetry.addLine(">> clawServo: Initialized");
        telemetry.addData("continuousIntakeServo1 Position ", CLAW_OPEN_POSITION);
        telemetry.addData("gamepad2.left_bumper", "Opens The claw");
        telemetry.addData("gamepad2.right_bumper", "Closes the claw");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.right_bumper) {
                moveClawToPosition(CLAW_CLOSED_POSITION);
            } else if (gamepad2.left_bumper) {
                moveClawToPosition(CLAW_OPEN_POSITION);
            }

            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();
        }
    }

    private void moveClawToPosition(double targetPosition) {
       /* double currentPosition = clawServo.getPosition();
        double power = pidController.calculate(currentPosition, targetPosition);
        clawServo.setPosition(currentPosition + power); */

        double currentPosition = clawServo.getPosition();
        double error = targetPosition - currentPosition;

        double dt = timer.seconds();
        timer.reset();

        integralClaw += error * dt;
        double derivative = (error - previousErrorClaw) / dt;

        double output = kP * error + kI * integralClaw + kD * derivative;

        clawServo.setPosition(currentPosition + output);

        previousErrorClaw = error;

        //To move claw without PID enable below code and disable above code
        //clawServo.setPosition(targetPosition);
        // rightClawServo.setPosition(1 - targetPosition); // Assuming right servo is mirrored
    }
}