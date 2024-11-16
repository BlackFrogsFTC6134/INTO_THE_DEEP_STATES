package org.firstinspires.ftc.teamcode.states.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Demo_BFR_LinearActuator", group="Demo")
public class BFR_Demo_LinearActuator extends LinearOpMode {

    private DcMotorEx linearActuator = null;
    private DcMotorEx rotateActuator = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double linearActuatorPower = 0;
    double rotateActuatorPower = 0;

    public static final double LINEAR_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double LINEAR_ACTUATOR_NORMAL_SCALE = 1.0;
    public static boolean LINEAR_ACTUATOR_FULL_POWER = true;

    public static final double ROTATE_ACTUATOR_SLOW_SCALE = 0.7;
    public static final double ROTATE_ACTUATOR_NORMAL_SCALE = 1.0;
    public static final boolean ROTATE_ACTUATOR_FULL_POWER = true;

    // Define the target position and time limit
    private static final int LINEAR_ACTUATOR_TARGET_POSITION_UP = 100; // Target position for moving up (in encoder ticks)
    private static final int LINEAR_ACTUATOR_TARGET_POSITION_DOWN = 0;   // Target position for moving down (in encoder ticks)

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        linearActuator = hardwareMap.get(DcMotorEx.class, "Linear_Actuator");
        rotateActuator = hardwareMap.get(DcMotorEx.class, "Rotate_Actuator");

        linearActuator.setDirection(DcMotorEx.Direction.FORWARD);
        rotateActuator.setDirection(DcMotorEx.Direction.FORWARD);

        linearActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateActuator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearActuator.setPower(0);
        rotateActuator.setPower(0);

        //linearActuator.setVelocity(100);
        //rotateActuator.setVelocity(0);

        linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine(">> linear & rotate actuators: Initialized");
        telemetry.addData("ROTATE_ACTUATOR_POWER", (ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE: ROTATE_ACTUATOR_SLOW_SCALE));
        telemetry.addData("gamepad2.y", "Move linear actuator up");
        telemetry.addData("gamepad2.a", "Move linear actuator down");
        telemetry.addData("gamepad2.left_stick_x", "Fine control linear actuator power");
        telemetry.addData("gamepad2.left_stick_y", "Fine control rotate actuator power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            rotateActuatorPower = -gamepad2.left_stick_y * (ROTATE_ACTUATOR_FULL_POWER ? ROTATE_ACTUATOR_NORMAL_SCALE: ROTATE_ACTUATOR_SLOW_SCALE);
            linearActuatorPower =  gamepad2.left_stick_x * ( LINEAR_ACTUATOR_FULL_POWER? LINEAR_ACTUATOR_NORMAL_SCALE : LINEAR_ACTUATOR_SLOW_SCALE);

            rotateActuator.setPower(rotateActuatorPower); // Set power to full (you can adjust this as needed)
            linearActuator.setPower(linearActuatorPower); // Set power to full (you can adjust this as needed)

            // Control to move the linear actuator up (select either Position control or time control)
            if (gamepad2.a) {
                moveLinearActuatorToPosition(LINEAR_ACTUATOR_TARGET_POSITION_DOWN);
                //     moveForSpecifiedTime(SLIDE_TIME_DOWN);
            }

            if (gamepad2.y) {
                moveLinearActuatorToPosition(LINEAR_ACTUATOR_TARGET_POSITION_UP);
                //  moveForSpecifiedTime(SLIDE_TIME_UP)
            }

            telemetry.addData(">> Linear actuator power: ", "%.2f", linearActuatorPower);
            telemetry.addData(">> Linear actuator position (ticks/rev): ", linearActuator.getCurrentPosition());

            telemetry.addLine("***************");
            telemetry.addData(">> Rotate actuator power: ", "%.2f", rotateActuatorPower);
            telemetry.addData(">> Rotate actuator position (ticks/rev): ", rotateActuator.getCurrentPosition());

            telemetry.update();
        }
    }

    // Function to move the linear actuator to the specified position
    private void moveLinearActuatorToPosition(int targetPosition) {
        linearActuator.setTargetPosition(targetPosition);
        linearActuator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(LINEAR_ACTUATOR_FULL_POWER ? LINEAR_ACTUATOR_NORMAL_SCALE : LINEAR_ACTUATOR_SLOW_SCALE); // Set power to full (you can adjust this as needed)

        while (opModeIsActive() && linearActuator.isBusy()) {
            telemetry.addData(">> Linear actuator power: ", "%.2f", linearActuatorPower);
            telemetry.addData(">> Linear actuator position (ticks/rev): ", linearActuator.getCurrentPosition());

            telemetry.addLine("***************");
            telemetry.addData(">> Rotate actuator power: ", "%.2f", rotateActuatorPower);
            telemetry.addData(">> Rotate actuator position (ticks/rev): ", rotateActuator.getCurrentPosition());

            telemetry.update();
        }

        // Stop all motion
        linearActuator.setPower(0);

        if(targetPosition == LINEAR_ACTUATOR_TARGET_POSITION_DOWN){
            linearActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Reset motor mode
        linearActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}