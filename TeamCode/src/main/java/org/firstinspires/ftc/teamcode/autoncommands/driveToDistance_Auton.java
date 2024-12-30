package org.firstinspires.ftc.teamcode.autoncommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "driveToDistance_Auton", group = "Auton")
public class driveToDistance_Auton extends LinearOpMode {

    private MecanumDrive drive;
    int sleepTime = 3000;
    double timeoutS = 10000;
    private static final double INCHES_TO_DRIVE = 23.5;

    public enum Auton_Command {
        DRIVE_FORWARD,
        DRIVE_BACKWARD,
        STRAFE_DRIVE_LEFT,
        STRAFE_DRIVE_RIGHT,
        TURN_DRIVE_LEFT,
        TURN_DRIVE_RIGHT,
        NONE
    }

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum StartPos{
        LEFT,
        RIGHT,
        DEFAULT
    }

    public enum AutoStrategy{
        PID_DRIVE,
        TIMED_DRIVE,
        DISTANCE_DRIVE,
        ROAD_RUNNER,
        DO_NOTHING
    }

    public static class AutoChoices {
        public double delay = 0.0;
        public Alliance alliance = Alliance.RED_ALLIANCE;
        public StartPos startPos = StartPos.LEFT;
        public AutoStrategy strategy = AutoStrategy.DO_NOTHING;
        public double xTarget = 0.0;
        public double yTarget = 0.0;
        public double turnTarget = 0.0;
        public double driveTime = 0.0;
        public double drivePower = 0.0;
    }

    public static final AutoChoices autoChoices = new AutoChoices();

    public void robotinit(){
        //
        // Create autonomous command according to chosen strategy.
        //

        autoChoices.strategy = AutoStrategy.DISTANCE_DRIVE;

        switch (autoChoices.strategy)
        {
            case PID_DRIVE:
                //autoCommand = new CmdPidDrive();
                break;

            case DISTANCE_DRIVE:
                //drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                break;

            case DO_NOTHING:
            default:
                //autoCommand = null;
                break;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        LED led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.initialize(hardwareMap, telemetry);
        drive.debugMode = true;

        // Initialize Dashboard and Camera Stream
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose


        /*
        ClawSubsystem claw = new ClawSubsystem(led, telemetry);
        claw.initialize(hardwareMap);

        WristSubsystem pivot = new WristSubsystem(led, telemetry);
        pivot.initialize(hardwareMap);

        ExtensionSubsystem extension = new ExtensionSubsystem(led, telemetry);
        extension.initialize(hardwareMap);

        ClimberSubsystem climber = new ClimberSubsystem(led, telemetry);
        climber.initialize(hardwareMap);

        // Wait for driver to press START)
        // Abort this loop is started or stopped.
        while (opModeInInit()) {

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status: ", "Ready to drive ...");
        }*/

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            telemetry.addData("Auton started: ", "");
            telemetry.update();
            sleep(5000);
            autonDriveWithoutRoadrunner(hardwareMap, Auton_Command.DRIVE_FORWARD, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
             sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap, Auton_Command.DRIVE_BACKWARD, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
                sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap,Auton_Command.STRAFE_DRIVE_LEFT, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
                sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap,Auton_Command.STRAFE_DRIVE_RIGHT, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
                sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap,Auton_Command.TURN_DRIVE_LEFT, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
                sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap,Auton_Command.TURN_DRIVE_RIGHT, INCHES_TO_DRIVE, INCHES_TO_DRIVE, timeoutS);
            sleep(sleepTime);

            autonDriveWithoutRoadrunner(hardwareMap,Auton_Command.TURN_DRIVE_RIGHT, INCHES_TO_DRIVE, INCHES_TO_DRIVE,timeoutS);
            sleep(sleepTime);
        }
        //drive.stopSubsystem(hardwareMap);
    }

    private void autonDriveWithoutRoadrunner(HardwareMap hardwareMap, Auton_Command command, double leftInches, double rightInches,double timeoutS) throws InterruptedException {
        switch (command) {
            case DRIVE_FORWARD:
               // telemetry.addData("Driving forward: ", inches);
                drive.DriveForward(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
                break;

            case DRIVE_BACKWARD:
                drive.DriveBackward(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
                break;
            case STRAFE_DRIVE_LEFT:
                drive.strafeDriveLeft(hardwareMap,telemetry, leftInches, rightInches, timeoutS);
                break;
            case STRAFE_DRIVE_RIGHT:
                drive.strafeDriveRight(hardwareMap,telemetry, leftInches, rightInches, timeoutS);
                break;
            case TURN_DRIVE_LEFT:
                drive.turnDriveLeft(hardwareMap,telemetry, leftInches, rightInches, timeoutS);
                break;
            case TURN_DRIVE_RIGHT:
                drive.turnDriveRight(hardwareMap,telemetry, leftInches, rightInches, timeoutS);
                break;
            default:
                case NONE:
                    break;
        }
    }
}