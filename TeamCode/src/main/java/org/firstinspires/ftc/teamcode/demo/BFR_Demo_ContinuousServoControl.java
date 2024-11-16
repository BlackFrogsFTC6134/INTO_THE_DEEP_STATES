package org.firstinspires.ftc.teamcode.states.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Demo_BFR_ContinuousServoControl", group="Demo")
public class BFR_Demo_ContinuousServoControl extends LinearOpMode {

    private CRServo continuousIntakeServo1 = null;
    private CRServo continuousIntakeServo2 = null;
    private Servo intakeRotationServo = null;

    private static final double CONTINUOUS_INTAKE_SERVO_FORWARD_POSITION = 1.0;  // Full speed forward
    private static final double CONTINUOUS_INTAKE_SERVO_REVERSE_POSITION = 0.0;  // Full speed reverse
    private static final double CONTINUOUS_INTAKE_SERVO_CENTER_POSITION = 0.5;  // Center position

    double intakeRotationServoPosition = 0;

    @Override
    public void runOpMode() {
        // Initialize the servo
        continuousIntakeServo1 = hardwareMap.get(CRServo.class, "continuousIntakeServo1");
        continuousIntakeServo2 = hardwareMap.get(CRServo.class, "continuousIntakeServo2");
        intakeRotationServo =    hardwareMap.get(Servo.class, "intakeRotation");

        telemetry.addData("Status", "Continuous intake servos initialized");
        //telemetry.addData("Continuous input servo 1 Position", continuousIntakeServo1.getPosition());
        //telemetry.addData("Continuous input servo 2 Position", continuousIntakeServo2.getPosition());

        telemetry.addData("gamepad1.dpad_up", "Sets full speed forward");
        telemetry.addData("gamepad1.dpad_down", "Sets full speed reverse");
        telemetry.addData("gamepad1.right_stick_y", "Fine tune intakeRotationServo");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        while (opModeIsActive()) {
            while (gamepad1.dpad_up) {
                continuousIntakeServo1.setPower(1.0);
                continuousIntakeServo2.setPower(-1.0);
            }
            while (gamepad1.dpad_down) {
                continuousIntakeServo1.setPower(-1.0);
                continuousIntakeServo2.setPower(1.0);
           }

            continuousIntakeServo1.setPower(0.0);
            continuousIntakeServo2.setPower(0.0);
            }

            // Optionally, use triggers for fine control
            intakeRotationServoPosition = gamepad1.right_stick_y;
            intakeRotationServo.setPosition(intakeRotationServoPosition);
        }
    }