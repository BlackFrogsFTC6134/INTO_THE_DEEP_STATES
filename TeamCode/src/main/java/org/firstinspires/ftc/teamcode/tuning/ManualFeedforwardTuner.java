package org.firstinspires.ftc.teamcode.tuning;

//@Config

/*public class ManualFeedforwardTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final DriveView view = new DriveView(hardwareMap);

        final TimeProfile profile = new TimeProfile(Profiles.constantProfile(
                DISTANCE, 0, view.maxVel, view.minAccel, view.maxAccel).baseProfile);

        Mode mode = Mode.TUNING_MODE;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        double startTs = System.nanoTime() / 1e9;

        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE: {
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    for (int i = 0; i < view.forwardEncsWrapped.size(); i++) {
                        double v = view.forwardEncsWrapped.get(i).getPositionAndVelocity().velocity;
                        telemetry.addData("v" + i, view.inPerTick * v);
                    }

                    double ts = System.nanoTime() / 1e9;
                    double t = ts - startTs;
                    if (t > profile.duration) {
                        movingForwards = !movingForwards;
                        startTs = ts;
                    }

                    DualNum<Time> v = profile.get(t).drop(1);
                    if (!movingForwards) {
                        v = v.unaryMinus();
                    }
                    telemetry.addData("vref", v.get(0));

                    double power = view.feedforward().compute(v) / view.voltageSensor.getVoltage();
                    view.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

                    break;
                }
                case DRIVER_MODE: {
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        startTs = System.nanoTime() / 1e9;
                    }

                    view.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            -gamepad1.left_stick_y,
                                            -gamepad1.left_stick_x
                                    ),
                                    -gamepad1.right_stick_x
                            )
                    );

                    break;
                }
            }

            telemetry.update();
        }
    }
}*/
