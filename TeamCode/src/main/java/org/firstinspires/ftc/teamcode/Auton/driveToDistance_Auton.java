package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadState;
import org.firstinspires.ftc.teamcode.SubSystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.drive.OpModeEx;

@Autonomous(name = "driveToDistance_Auton", group = "Linear OpMode")
public class driveToDistance_Auton extends LinearOpMode {

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

    private OpModeEx opMode;
    private static final double INCHES_TO_DRIVE = 23.5;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime;
        runtime = new ElapsedTime();

        led = new LEDSubsystem();
        led.initialize(hardwareMap);

        drivetrain = new DrivetrainSubsystem(led, distanceSensor1, distanceSensor2);
        drivetrain.initialize(hardwareMap);

        claw = new ClawSubsystem(led);
        claw.initialize(hardwareMap);

        pivot = new PivotSubsystem(led);
        pivot.initialize(hardwareMap);

        extension = new ExtensionSubsystem(led);
        extension.initialize(hardwareMap);

        climber = new ClimberSubsystem(led);
        climber.initialize(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "Waiting for user to press start");

        if (opModeIsActive()) {
                driveToDistance("DriveForward", INCHES_TO_DRIVE);
                sleep(1000);


                driveToDistance("DriveBackward", INCHES_TO_DRIVE);
                sleep(1000);


                driveToDistance("strafeDriveLeft", INCHES_TO_DRIVE);
                sleep(1000);



                driveToDistance("strafeDriveRight", INCHES_TO_DRIVE);
                sleep(1000);



                driveToDistance("turnDriveLeft", INCHES_TO_DRIVE);
                sleep(1000);



                driveToDistance("turnDriveRight", INCHES_TO_DRIVE);
                sleep(1000);



        }
        drivetrain.stopSubsystem(hardwareMap);
    }


    private void driveToDistance(String direction, double inches) {
        switch (direction) {
            case "DriveForward":
                drivetrain.DriveForward(inches);
                break;
            case "DriveBackward":
                drivetrain.DriveBackward(inches);
                break;
            case "strafeDriveLeft":
                drivetrain.strafeDriveLeft(inches);
                break;
            case "strafeDriveRight":
                drivetrain.strafeDriveRight(inches);
                break;
            case "turnDriveLeft":
                drivetrain.turnDriveLeft(inches);
                break;
            case "turnDriveRight":
                drivetrain.turnDriveRight(inches);
                break;
        }
    }
}