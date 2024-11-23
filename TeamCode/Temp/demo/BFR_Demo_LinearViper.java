package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Demo_BFR_LinearViper", group="Demo")
public class BFR_Demo_LinearViper extends LinearOpMode {

    private DcMotorEx linearViper = null;
    private DcMotorEx rotateViper = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    private double linearViperPower = 0;
    private double rotateViperPower = 0;

    public double LINEAR_VIPER_SLOW_SCALE = 0.8;
    public double LINEAR_VIPER_NORMAL_SCALE = 0.8;
    public boolean LINEAR_VIPER_FULL_POWER = true;

    public double ROTATE_VIPER_SLOW_SCALE = 0.7;
    public double ROTATE_VIPER_NORMAL_SCALE = 1.0;
    public boolean ROTATE_VIPER_FULL_POWER = true;

    public int LINEAR_VIPER_TARGET_POSITION_UP = 2000; // Target position for moving up (in encoder ticks)
    public int LINEAR_VIPER_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        linearViper = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        rotateViper = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        linearViper.setDirection(DcMotorEx.Direction.FORWARD);
        rotateViper.setDirection(DcMotorEx.Direction.FORWARD);

        linearViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearViper.setPower(linearViperPower);
        rotateViper.setPower(rotateViperPower);

        //linearViper.setVelocity(100);
        //rotateViper.setVelocity(100);

        linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine(">> linear & rotate vipers: Initialized");
        telemetry.addData("LINEAR_VIPER_POWER", (LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE));
        telemetry.addData("ROTATE_VIPER_POWER", (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE));
        telemetry.addData("gamepad2.x", "Move linear viper up");
        telemetry.addData("gamepad2.b", "Move linear viper down");
        telemetry.addData("gamepad2.left_stick_x", "Fine control linear viper power");
        telemetry.addData("gamepad2.left_stick_y", "Fine control rotate viper power");
        telemetry.update();

        LINEAR_VIPER_TARGET_POSITION_DOWN = linearViper.getCurrentPosition();
        telemetry.addData(">> Linear viper position (ticks/rev): ", linearViper.getCurrentPosition());

        waitForStart();

        while (opModeIsActive()) {
            rotateViperPower = -gamepad2.right_stick_y * (ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE);
            linearViperPower =  gamepad2.right_stick_x * (LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE);
            linearViper.setPower(linearViperPower);

            rotateViper.setPower(ROTATE_VIPER_FULL_POWER ? ROTATE_VIPER_NORMAL_SCALE : ROTATE_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

            // Control to move the linear actuator down (select either Position control or time control)
            if (gamepad2.b) {
                moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_DOWN);


            }
            // Control to move the slide down
            if (gamepad2.x) {
                moveLinearViperToPosition(LINEAR_VIPER_TARGET_POSITION_UP);
            }

            telemetry.addData(">> Linear viper power: ", "%.2f", linearViperPower);
            telemetry.addData(">> Linear viper position (ticks/rev): ", linearViper.getCurrentPosition());

            telemetry.addLine("***************");
            telemetry.addData(">> Rotate viper power: ", "%.2f", rotateViperPower);
            telemetry.addData(">> Rotate viper position (ticks/rev): ", rotateViper.getCurrentPosition());

            telemetry.update();
        }
    }

    // Function to move the viper to the specified position
    public void moveLinearViperToPosition(int targetPosition) {
        linearViper.setTargetPosition(targetPosition);
        linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearViper.setVelocity(200);
        linearViper.setPower(LINEAR_VIPER_FULL_POWER ? LINEAR_VIPER_NORMAL_SCALE : LINEAR_VIPER_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && linearViper.isBusy()) {
            telemetry.addData(">> Linear viper power: ", "%.2f", linearViperPower);
            telemetry.addData(">> Linear viper position (ticks/rev): ", linearViper.getCurrentPosition());

            telemetry.addLine("***************");
            telemetry.addData(">> Rotate viper power: ", "%.2f", rotateViperPower);
            telemetry.addData(">> Rotate viper position (ticks/rev): ", rotateViper.getCurrentPosition());

            telemetry.update();
        }

        // Stop all motion
       linearViper.setPower(0);

        // Reset motor mode
        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}