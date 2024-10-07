package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "BFR_TeleOp")
@Disabled
public class BFR_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //DcMotor Viper1;
        //DcMotor Viper2;
        //DcMotor Actuator1;
        //DcMotor Actuator2;
        //Servo claw;
        //Servo intake;
        DcMotor leftFront; 
        DcMotor leftBack; 
        DcMotor rightFront; 
        DcMotor rightBack; 
        float X_movement;
        float Y_movement;
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        //Viper1 = hardwareMap.get(DcMotorEx.class, "Viper1");
        //Viper2 = hardwareMap.get(DcMotorEx.class, "Viper2");
        //Actuator1 = hardwareMap.get(DcMotorEx.class, "Viper2");
        //Actuator2 = hardwareMap.get(DcMotorEx.class, "Viper2");
        //claw = hardwareMap.get(Servo.class, "claw");
        //intake = hardwareMap.get(Servo.class, "intake");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //WaitForStart();

        while (opModeIsActive()){
            X_movement = -gamepad1.left_stick_y;
            Y_movement = -gamepad1.left_stick_x;
            drive  = -gamepad1.left_stick_y  * 0.25;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  * 0.25;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x * (1/3);
        }


    }

}
