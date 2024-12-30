package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.RobotParams;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * Author: Kavish Karia
 * Date create: 12/28/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */

@TeleOp(name="Subsystems Example")
public class SubsystemExample extends LinearOpMode {
    MecanumDrive drive;
    RobotHardware robot = RobotHardware.getInstance();
    //Arm arm = robot.arm;
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        robot.initialize(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()) {
           //arm.periodic(gamepad2);
        }
    }
}
