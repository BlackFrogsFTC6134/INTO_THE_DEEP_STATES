package org.firstinspires.ftc.teamcode.autoncommands;

import static org.firstinspires.ftc.teamcode.constants.DriveConstants.USE_MOTOR_ENCODER;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Auton")

public class States_Auton extends LinearOpMode {
    private MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    int sleepTime = 2000;

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

    private final ElapsedTime     runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;

    private static final double INCHES_TO_DRIVE = 23.5;
    Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        LED led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);
        initialPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose

        drive = new MecanumDrive(hardwareMap, initialPose);
        drive.initialize(hardwareMap, telemetry);
        drive.debugMode = true;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Note: Reverse movement is obtained by setting a negative speed)??
        DriveRobot(hardwareMap, DRIVE_SPEED,  23.5,  23.5, 5.0);  // S1: Forward with timeout
        DriveRobot(hardwareMap, -DRIVE_SPEED,  23.5,  23.5, 5.0);  // S2 Reverse with timeout
        DriveRobot(hardwareMap, TURN_SPEED,   12, -12, 4.0);  // S3: Turn Right with timeout
        DriveRobot(hardwareMap, DRIVE_SPEED, -12, 12, 4.0);  // S4: Turn right with timeout
/*
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(sleepTime);  // pause to display final telemetry message. */
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void DriveRobot(HardwareMap hardwareMap, double speed,
                           double leftInches, double rightInches,
                           double timeoutS) {
        double newLeftTarget;
        double newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            //drive.DriveForward(hardwareMap, leftInches, rightInches);

            drive.SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);

            // reset the timeout time and start motion.
            runtime.reset();
            drive.setPower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (drive.isBusy())) {
/*
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", leftInches, rightInches);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            drive.getRobotCurrentXPosition(), drive.getRobotCurrentYPosition());
                telemetry.update(); */
            }

            // Stop all motion;
            drive.resetPower();

            // Turn off RUN_TO_POSITION
            if(USE_MOTOR_ENCODER) {
                drive.setRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            } else {
                drive.setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
            sleep(sleepTime);   // optional pause after each move.
        }
    }
}
