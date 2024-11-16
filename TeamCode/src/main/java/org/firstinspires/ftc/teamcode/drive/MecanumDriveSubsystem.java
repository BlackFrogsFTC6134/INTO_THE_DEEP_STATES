package org.firstinspires.ftc.teamcode.states.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDriveSubsystem implements DriveSubsystem {
        private DcMotorEx leftFront, leftBack, rightFront, rightBack;

        @Override
        public void initialize(HardwareMap hardwareMap) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        /*
        Zero Power Behavior: Set the zero power behavior of the motors to BRAKE or FLOAT
        depending on your design requirements. BRAKE will hold the motors in place when no power is applied,
        while FLOAT allows the motors to spin freely.
        */

            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            if (DriveConstants.RUN_USING_ENCODER) {
                leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            } else {
                leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
        }


        @Override
        public void drive(double forward, double strafe, double turn) {
            double leftFrontPower = forward + strafe + turn;
            double leftRearPower = forward - strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double rightRearPower = forward + strafe - turn;

            setDrivePowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
        }

        @Override
        public void setDrivePowers(double leftFront, double leftBack, double rightFront, double rightBack) {
            this.leftFront.setPower(leftFront);
            this.leftBack.setPower(leftBack);
            this.rightFront.setPower(rightFront);
            this.rightBack.setPower(rightBack);
        }

        @Override
        public void periodic() {
            // Implement any periodic checks or updates
        }

        @Override
        public void stop() {
            setDrivePowers(0, 0, 0, 0);
        }
    }

