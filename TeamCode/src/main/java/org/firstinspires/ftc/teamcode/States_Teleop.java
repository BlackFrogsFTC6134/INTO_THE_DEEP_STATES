package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ClawSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Extension;
import org.firstinspires.ftc.teamcode.mechanisms.PivotSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

@TeleOp (name = "States_Teleop", group = "TeleOp")
public class States_Teleop extends LinearOpMode {
    private Drivetrain drivetrain;
    private ClawSubsystem clawSubsystem;
    private PivotSubsystem pivotSubsystem;
    private Extension extension;
    private LED led;
    private Climber climber;
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    //private GamepadState gamepadState = new GamepadState();

    private Servo clawRight, clawLeft;

    double NormalDrivetrainPower = 0.8;

    boolean gamepad2_b_pressed = false;
    boolean gamepad2_a_pressed = false;

    @Override
    public void runOpMode() throws InterruptedException{
        ElapsedTime runtime;
        runtime = new ElapsedTime();

        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        drivetrain = new Drivetrain(hardwareMap, led , distanceSensor1, distanceSensor2, telemetry);
        drivetrain.initialize(hardwareMap, telemetry);

        clawSubsystem = new ClawSubsystem(this);
        clawSubsystem.initialize(hardwareMap, telemetry);

        pivotSubsystem = new PivotSubsystem(hardwareMap,this);
        pivotSubsystem.initialize(hardwareMap, telemetry);

        extension = new Extension(hardwareMap, led, telemetry);
        extension.initialize(hardwareMap, telemetry);

        climber = new Climber(hardwareMap, led, telemetry);
        climber.initialize(hardwareMap, telemetry);

        clawRight = hardwareMap.get(Servo.class, "clawreh0");
        clawLeft = hardwareMap.get(Servo.class, "clawleh1");

        clawRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        distanceSensor1 = new DistanceSensor(hardwareMap, led, telemetry);
        distanceSensor1.initialize(hardwareMap, telemetry);

        distanceSensor2 = new DistanceSensor(hardwareMap, led, telemetry);
        distanceSensor2.initialize(hardwareMap, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        //    Gamepad1 = gamepad1;
          //  Gamepad2 = gamepad2;

            //gamepadState.updateFromGamepad(gamepad1, gamepad2);

            double axial = -gamepad1.left_stick_y* NormalDrivetrainPower; // Forward/Backward control
            double lateral = gamepad1.left_stick_x*NormalDrivetrainPower; // Left/Right control
            double yaw = gamepad1.right_stick_x*NormalDrivetrainPower; // Rotation control

            drivetrain.drive(axial, lateral, yaw);

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
                //pivotSubsystem.setMotorPosition((int) (400 * 19.2));
            } else if (gamepad2.back && gamepad2.dpad_right) {
                //pivotSubsystem.setMotorPosition((int) (400 * 10.0));

            } else if (gamepad2.back && gamepad2.dpad_down) {
                //pivotSubsystem.setMotorPosition((int) (0));
            } else if (gamepad2.back && gamepad2.dpad_left) {
                // Reset 0 (home set)
                //pivotSubsystem.resetEncoder();
            } else {
                 pivotSubsystem.telemetry(telemetry);
            }

            if (gamepad1.left_bumper) {
                drivetrain.increaseDrivePower();
            } else {
                //drivetrain.decreaseDrivePower(gamepadState);
            }
            // Directional Flipping Section
            /*
            if (gamepad1.a) {
                drivetrain.makeRearOfRobotFront(hardwareMap, gamepadState);
            }
            if (gamepad1.b) {
                // When Holding B button ALONE Right Side of robot becomes front
                drivetrain.makeRightOfRobotFront(hardwareMap, gamepadState);
            }
            if (gamepad1.x) {
                // When Holding X button ALONE Left Side of robot becomes front
                drivetrain.makeLeftOfRobotFront(hardwareMap, gamepadState);
            }
            */

            // SPEC OPS Pressing B Alone opens and closes clawSubsystem
            if (gamepad2.b) {
                gamepad2_b_pressed = true;
                if(gamepad2_b_pressed){
                    //clawSubsystem.openClaw();
                    clawRight.setPosition(0.5);
                    clawLeft.setPosition(0.5);
                    gamepad2_b_pressed = false;
                }
            }

            // SPEC OPS Pressing A Alone opens and closes clawSubsystem
            if (gamepad2.a) {
                gamepad2_a_pressed = true;
                if(gamepad2_a_pressed){
                    //clawSubsystem.closeClaw();
                    clawRight.setPosition(0);
                    clawLeft.setPosition(0);
                    gamepad2_a_pressed = false;
                }
            }

            // SPEC OPS Pressing X ALONE Moves PivotSubsystem Up
            if (gamepad2.x) {
                //pivotSubsystem.setServoPosition(0.26);
            }
            // SPEC OPS Pressing Y ALONE Moves PivotSubsystem Down
            if (gamepad2.y) {
                //pivotSubsystem.setServoPosition(0.1);
            }

            // SPEC OPS Pressing RightBumper ALONE Moves PivotSubsystem UP
            // SPEC OPS Pressing LeftBumper ALONE Moves PivotSubsystem Down
            if (gamepad2.right_bumper) {
                //pivotSubsystem.setPivotMotorPower(1.0);

            } else if (gamepad2.left_bumper) {
                //pivotSubsystem.setPivotMotorPower(-1.0);
            } else {
                //pivotSubsystem.setPivotMotorPower(0.0);
            }
            // SPEC OPS Pressing Right Bumper ALONE Moves Elevator Up
            // SPEC OPS Pressing Left Bumper ALONE Moves Elevator Down

        }

        telemetry.addData("Status", "Robot stopped");
        telemetry.update();

        drivetrain.stopSubsystem(telemetry);
        led.stopSubsystem(telemetry);
    }
}