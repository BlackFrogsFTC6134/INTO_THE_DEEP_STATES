package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Auton_Test {
    @Autonomous(name="Viper: Sample to Observation Zone", group="IntoTheDeep")
    public class ViperAutonomous extends LinearOpMode {


        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private Servo gripper = null;
        private DistanceSensor distanceSensor = null;
        private ElapsedTime runtime = new ElapsedTime();
        private static final double WHEEL_DIAMETER_INCHES = 4.0; // Wheel diameter in inches
        private static final double TICKS_PER_REV = 537.6; // For GoBilda 20:1 motors,
        // adjust if using different
        private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        @Override
        public void runOpMode() {
            // Initialize hardware
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            gripper = hardwareMap.get(Servo.class, "clawServo");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_Distance");


            // Set motor directions
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);


            // Initialize servos
            gripper.setPosition(0.0); // Adjust for initial gripper position (open or closed)

            waitForStart();
            runtime.reset();

            if (opModeIsActive()) {
                // Step 1: Move to the Spike Mark (Adjust distance as needed)
                moveForward(30); // Moves the robot forward to Spike Mark position

                // Step 2: Pick up the sample
                pickUpSample();

                // Step 3: Move to the Observation Zone (Adjust distance and direction as needed)
                moveBackward(10);
                turnRight(90); // Adjust angle for Observation Zone
                moveForward(20); // Move into Observation Zone area

                // Step 4: Drop the sample
                dropSample();

                // Optional: Back up slightly after dropping the sample
                moveBackward(5);

                telemetry.addData("Status", "Task Completed");
                telemetry.update();
            }
        }


        private void moveForward(double distanceInches) {
            // Convert distance to motor ticks, assuming robot has encoders
            int ticks = (int)(distanceInches * TICKS_PER_INCH); // Define TICKS_PER_INCH for your robot
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);


            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDrive.setPower(0.5); // Adjust speed as needed
            rightDrive.setPower(0.5);


            while (opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()) {
                telemetry.addData("Path", "Moving Forward");
                telemetry.update();
            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }


        private void moveBackward(double distanceInches) {
            moveForward(-distanceInches); // Reuse moveForward method for simplicity
        }


        private void turnRight(int degrees) {
            // Implement a turn method here or use IMU for more accuracy
            // Adjust power and timing for a 90-degree turn, or use encoders for accuracy
        }


        private void pickUpSample() {
            // Assume gripper closes at position 1.0 to pick up
            gripper.setPosition(1.0);
            sleep(1000); // Wait for gripper to close
            telemetry.addData("Gripper", "Closed to pick up sample");
            telemetry.update();
        }


        private void dropSample() {
            // Assume gripper opens at position 0.0 to release
            gripper.setPosition(0.0);
            sleep(1000); // Wait for gripper to release
            telemetry.addData("Gripper", "Opened to drop sample");
            telemetry.update();
        }
    }


}
