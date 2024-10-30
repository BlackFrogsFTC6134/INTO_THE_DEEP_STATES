package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ArcTanCalculator", group="Concept")
public class ArcTanCalculator extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Example values to calculate arctangent
        double[] testValues = {0, 1, -1, 0.5, -0.5, 2, -2};

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            for (double value : testValues) {
                double result = Math.atan(value);
                double resultDegrees = Math.toDegrees(result);

                telemetry.addData("Input", value);
                telemetry.addData("Arctangent (radians)", "%.4f", result);
                telemetry.addData("Arctangent (degrees)", "%.2f", resultDegrees);
                telemetry.addLine();
            }

            // Calculate arctangent of y/x (atan2)
            double x = 3;
            double y = 4;
            double atan2Result = Math.atan2(y, x);
            double atan2ResultDegrees = Math.toDegrees(atan2Result);

            telemetry.addData("atan2(y,x) where y=4, x=3", "");
            telemetry.addData("Result (radians)", "%.4f", atan2Result);
            telemetry.addData("Result (degrees)", "%.2f", atan2ResultDegrees);

            telemetry.update();

            // Pause for better readability
            sleep(10000);
        }
    }
}
