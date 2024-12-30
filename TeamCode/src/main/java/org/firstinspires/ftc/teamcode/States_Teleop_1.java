package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ClawSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Extension;
import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.mechanisms.PivotSubsystem;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "States_Teleop_1", group = "TeleOp")
public class States_Teleop_1 extends LinearOpMode {
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

    double NormalDrivetrainPower = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime;
        runtime = new ElapsedTime();

        LED led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, led, distanceSensor1, distanceSensor2, telemetry);
        drivetrain.initialize(hardwareMap, telemetry);

        ClawSubsystem clawSubsystem = new ClawSubsystem(this);
        clawSubsystem.initialize(hardwareMap, telemetry);

        PivotSubsystem PivotSubsystem = new PivotSubsystem(hardwareMap,this);
        PivotSubsystem.initialize(hardwareMap, telemetry);

        Extension extension = new Extension(hardwareMap, led, telemetry);
        extension.initialize(hardwareMap, telemetry);

        Climber climber = new Climber(hardwareMap, led, telemetry);
        climber.initialize(hardwareMap, telemetry);

        Servo clawRight = hardwareMap.get(Servo.class, "clawreh0");
        Servo clawLeft = hardwareMap.get(Servo.class, "clawleh1");

        clawRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        distanceSensor1 = new DistanceSensor(hardwareMap, led, telemetry);
        distanceSensor1.initialize(hardwareMap, telemetry);

        distanceSensor2 = new DistanceSensor(hardwareMap, led, telemetry);
        distanceSensor2.initialize(hardwareMap, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            runtime.reset();

            // Run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double axial = -gamepad1.left_stick_y * NormalDrivetrainPower; // Forward/Backward control
                double lateral = gamepad1.left_stick_x * NormalDrivetrainPower; // Left/Right control
                double yaw = -gamepad1.right_stick_x * NormalDrivetrainPower; // Rotation control

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));
                //drivetrain.drive(axial, lateral, yaw);

                if (gamepad2.right_stick_button) {
                    climber.setPosition(0.0);
                }

                if (gamepad2.left_stick_button) {
                    climber.setPosition(0.1);
                }
                // Viper Slide  Full Up and Mid way
                // 400= 1 rotation Full up is 6 rotations
                if (gamepad2.start && gamepad2.dpad_up) {
                    extension.setPosition(400 * 6);
                } else if (gamepad2.start && gamepad2.dpad_right) {
                    extension.setPosition(400 * 3);
                } else if (gamepad2.start && gamepad2.dpad_down) {
                    extension.setPosition(0);
                } else if (gamepad2.start && gamepad2.dpad_left) {
                    // Reset 0 (home set)
                    extension.resetEncoder();
                } else {
                    extension.update(gamepad2, telemetry);
                }

                if (gamepad2.back && gamepad2.dpad_up) {
                    //PivotSubsystem.setMotorPosition((int) (400 * 19.2));
                } else if (gamepad2.back && gamepad2.dpad_right) {
                    //PivotSubsystem.setMotorPosition((int) (400 * 10.0));

                } else if (gamepad2.back && gamepad2.dpad_down) {
                    //PivotSubsystem.setMotorPosition(0);
                } else if (gamepad2.back && gamepad2.dpad_left) {
                    // Reset 0 (home set)
                    //PivotSubsystem.resetEncoder();
                } else {
                    PivotSubsystem.telemetry(telemetry);
                }

                if (gamepad1.left_bumper) {
                    drivetrain.increaseDrivePower();
                }

                if (gamepad1.a) {

                }
                if (gamepad1.b) {

                }
                if (gamepad1.x) {

                }

                if (gamepad2.b) {
                    clawSubsystem.releaseClaw();
                }

                if (gamepad2.a) {
                    clawSubsystem.grabClaw();
                }

                if (gamepad2.x) {
                    //PivotSubsystem.setServoPosition(0.26);
                }
                // SPEC OPS Pressing Y ALONE Moves PivotSubsystem Down
                if (gamepad2.y) {
                    //PivotSubsystem.setServoPosition(0.1);
                }

                // SPEC OPS Pressing RightBumper ALONE Moves PivotSubsystem UP
                // SPEC OPS Pressing LeftBumper ALONE Moves PivotSubsystem Down
                if (gamepad2.right_bumper) {
                    //PivotSubsystem.setPivotMotorPower(1.0);

                } else if (gamepad2.left_bumper) {
                    //PivotSubsystem.setPivotMotorPower(-1.0);
                } else {
                    //PivotSubsystem.setPivotMotorPower(0.0);
                }
                // SPEC OPS Pressing Right Bumper ALONE Moves Elevator Up
                // SPEC OPS Pressing Left Bumper ALONE Moves Elevator Down

            }
        }

        telemetry.addData("Status", "Robot stopped");
        telemetry.update();

        drivetrain.stopSubsystem(telemetry);
        led.stopSubsystem(telemetry);
    }
}