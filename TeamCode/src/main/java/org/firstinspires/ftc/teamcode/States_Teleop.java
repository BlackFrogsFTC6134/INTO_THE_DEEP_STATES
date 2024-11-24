package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadState;
import org.firstinspires.ftc.teamcode.SubSystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.LEDSubsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name = "States_Teleop", group = "TeleOp")
public class States_Teleop extends LinearOpMode {
    private DrivetrainSubsystem drivetrain;
    private ClawSubsystem claw;
    private PivotSubsystem pivot;
    private ExtensionSubsystem extension;
    private LEDSubsystem led;
    private ClimberSubsystem climber;
    private DistanceSensorSubsystem distanceSensor1;
    private DistanceSensorSubsystem distanceSensor2;
    private GamepadState gamepadState = new GamepadState();
    private Gamepad Gamepad1, Gamepad2;

    double NormalDrivetrainPower = 0.8;
            ;
    @Override
    public void runOpMode() throws InterruptedException{
        ElapsedTime runtime;
        runtime = new ElapsedTime();

        led = new LEDSubsystem();
        led.initialize(hardwareMap);

        drivetrain = new DrivetrainSubsystem(led , distanceSensor1, distanceSensor2);
        drivetrain.initialize(hardwareMap);

        claw = new ClawSubsystem(led);
        claw.initialize(hardwareMap);

        pivot = new PivotSubsystem(led);
        pivot.initialize(hardwareMap);

        extension = new ExtensionSubsystem(led);
        extension.initialize(hardwareMap);

        climber = new ClimberSubsystem(led);
        climber.initialize(hardwareMap);

/*
        distanceSensor1 = new DistanceSensorSubsystem(led);
        distanceSensor1.initialize(hardwareMap);

        distanceSensor2 = new DistanceSensorSubsystem(led);
        distanceSensor2.initialize(hardwareMap);
*/
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
            Gamepad1 = gamepad1;
            Gamepad2 = gamepad2;

            gamepadState.updateFromGamepad(Gamepad1, Gamepad2);

            double axial = -Gamepad1.left_stick_y* NormalDrivetrainPower; // Forward/Backward control
            double lateral = Gamepad1.left_stick_x*NormalDrivetrainPower; // Left/Right control
            double yaw = Gamepad1.right_stick_x*NormalDrivetrainPower; // Rotation control

            telemetry.addData("axial", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("yaw", yaw);
            telemetry.update();

            drivetrain.drive(hardwareMap, axial, lateral, yaw);

            if (Gamepad2.right_stick_button) {
                climber.setPosition(0.0);

            }
            if (Gamepad2.left_stick_button) {
                climber.setPosition(0.1);
            }
            // Viper Slide  Full Up and Mid way
            // 400= 1 rotation Full up is 6 rotations
            if (Gamepad2.start && Gamepad2.dpad_up) {
                extension.setPosition(400 * 6);
            } else if (Gamepad2.start && Gamepad2.dpad_right) {
                extension.setPosition(400 * 3);
            } else if (Gamepad2.start && Gamepad2.dpad_down) {
                extension.setPosition(0);
            } else if (Gamepad2.start && Gamepad2.dpad_left) {
                // Reset 0 (home set)
                extension.resetEncoder();
            } else {
                extension.update(hardwareMap, Gamepad1, Gamepad2);
            }
            if (Gamepad2.back && Gamepad2.dpad_up) {
                pivot.setMotorPosition(hardwareMap, (int) (400 * 19.2));
            } else if (Gamepad2.back && Gamepad2.dpad_right) {
                pivot.setMotorPosition(hardwareMap, (int) (400 * 10.0));

            } else if (Gamepad2.back && Gamepad2.dpad_down) {
                pivot.setMotorPosition(hardwareMap, (int) (0));
            } else if (Gamepad2.back && Gamepad2.dpad_left) {
                // Reset 0 (home set)
                pivot.resetEncoder(hardwareMap);
            } else {
                 pivot.update(hardwareMap,Gamepad1, Gamepad2);
            }

            if (gamepad1.left_bumper) {
                drivetrain.increaseDrivePower(hardwareMap, gamepadState);
            } else {
                drivetrain.decreaseDrivePower(hardwareMap, gamepadState);
            }
            // Directional Flipping Section
            /*
            if (Gamepad1.a) {
                drivetrain.makeRearOfRobotFront(hardwareMap, gamepadState);
            }
            if (Gamepad1.b) {
                // When Holding B button ALONE Right Side of robot becomes front
                drivetrain.makeRightOfRobotFront(hardwareMap, gamepadState);
            }
            if (Gamepad1.x) {
                // When Holding X button ALONE Left Side of robot becomes front
                drivetrain.makeLeftOfRobotFront(hardwareMap, gamepadState);
            }
            */

            // SPEC OPS Pressing B Alone opens and closes claw
            if (Gamepad2.b) {
                claw.openClaw();
            }

            // SPEC OPS Pressing A Alone opens and closes claw
            if (Gamepad2.a) {
                claw.closeClaw();
            }

            // SPEC OPS Pressing X ALONE Moves Wrist Up
            if (Gamepad2.x) {
                pivot.setServoPosition(hardwareMap, 0.26);
            }
            // SPEC OPS Pressing Y ALONE Moves Wrist Down
            if (Gamepad2.y) {
                pivot.setServoPosition(hardwareMap, 0.1);
            }

            // SPEC OPS Pressing RightBumper ALONE Moves Pivot UP
            // SPEC OPS Pressing LeftBumper ALONE Moves Pivot Down
            if (Gamepad2.right_bumper) {
                pivot.setPivotMotorPower(hardwareMap, 1.0);

            } else if (Gamepad2.left_bumper) {
                pivot.setPivotMotorPower(hardwareMap, -1.0);
            } else {
                pivot.setPivotMotorPower(hardwareMap, 0.0);
            }
            // SPEC OPS Pressing Right Bumper ALONE Moves Elevator Up
            // SPEC OPS Pressing Left Bumper ALONE Moves Elevator Down

        }
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();

        claw.stopSubsystem(hardwareMap);
        pivot.stopSubsystem(hardwareMap);
        extension.stopSubsystem(hardwareMap);
        climber.stopSubsystem(hardwareMap);
        drivetrain.stopSubsystem(hardwareMap);
        led.stopSubsystem(hardwareMap);
    }
}