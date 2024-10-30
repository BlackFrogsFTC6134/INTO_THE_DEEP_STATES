package org.firstinspires.ftc.teamcode.demo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="BFR_Demo_ViperSlideHoldPosition", group="Demo")
public class BFR_Demo_ViperSlideHoldPosition extends LinearOpMode {

    private DcMotorEx linearViper;
    private DcMotorEx rotateViper;

    private static final int EXTENDED_VIPER_POSITION = 300; // Example target position
    private static final int RETRACTED_VIPER_POSITION = 0;   // Original position
    private static final double linearViperPower = 1.0;     // Constant power for linear movement
    private static double rotateViperPower = 1.0;     // Constant power for rotate movement

    private int currentViperTargetPosition ;

    @Override
    public void runOpMode() {
        // Initialize the motor
        linearViper = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        rotateViper = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        // Set motor direction and zero power behavior
        linearViper.setDirection(DcMotorEx.Direction.FORWARD);
        linearViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotateViper.setDirection(DcMotorEx.Direction.FORWARD);
        rotateViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset encoder and set to RUN_USING_ENCODER mode initially
        linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Linear & rotate viper motors initialized");
        telemetry.addData("gamepad2.b", "Viper goes to extended position");
        telemetry.addData("gamepad2.x", "Viper goes to retracted position");

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            if (gamepad2.b) { // Stick moved up
                currentViperTargetPosition = EXTENDED_VIPER_POSITION;
                moveToPosition(EXTENDED_VIPER_POSITION, linearViperPower);
            } else if (gamepad2.x) { // Stick moved down
                currentViperTargetPosition = RETRACTED_VIPER_POSITION;
                moveToPosition(RETRACTED_VIPER_POSITION, linearViperPower);
            }

            rotateViperPower = -gamepad2.right_stick_y;
            rotateViper.setPower(rotateViperPower);

            telemetry.addData("Target Viper Position", currentViperTargetPosition);
            telemetry.addData("Current Viper Position", linearViper.getCurrentPosition());
            telemetry.update();

        }
    }

    private void moveToPosition(int targetPosition, double power) {
        linearViper.setPower(power);
        linearViper.setTargetPosition(targetPosition);
        linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && linearViper.isBusy()) {
            // Wait until the motor reaches the target position
            idle();
            telemetry.addData("Current Viper Position", linearViper.getCurrentPosition());
        }

        // Stop the motor once the target is reached
        linearViper.setPower(0);

        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
