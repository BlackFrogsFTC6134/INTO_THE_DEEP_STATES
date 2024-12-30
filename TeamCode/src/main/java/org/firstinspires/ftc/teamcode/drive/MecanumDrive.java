package org.firstinspires.ftc.teamcode.drive;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.constants.DriveConstants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SubSystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.SubSystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.LED;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.TwoDeadWheelLocalizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

// FTC dashboard http://192.168.43.1:8080/dash
// Example MecanumDrive - https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/SampleMecanumDrive.java

@Config
public final class MecanumDrive extends LinearOpMode implements DrivetrainSubsystem {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP; //Blackfrog_Setting for 2023 season
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT; //Blackfrog_Setting for 2023 season

        // drive model parameters
        public double inPerTick = 1; // Blackfrog_Setting default = 1 141.24/5583.75 https://rr.brott.dev/docs/v1-0/tuning. Refer to ForwardPushTest
        public double lateralInPerTick = 1; // Blackfrog_Setting def https://rr.brott.dev/docs/v1-0/tuning. Refer to LateralPushTest
        public double trackWidthTicks = 0; // Blackfrog_Setting https://rr.brott.dev/docs/v1-0/tuning. Refer to either Drive Encoders or Dead Wheels section

        // feedforward parameters (in tick units)
        public double kV = 1.0 / rpmToVelocity(MAX_RPM); // Blackfrog_Setting 0.00392616896265144
        public double kA = 0; // Blackfrog_Setting 0.001995
        public double kStatic = 0; // Blackfrog_Setting 1.1890544370408778

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0;
        public double lateralGain = 0;
        public double headingGain = 0; // shared with turn

        public double axialVelGain = 0;
        public double lateralVelGain = 0;
        public double headingVelGain = 0; // shared with turn


        public Params(RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                      RevHubOrientationOnRobot.UsbFacingDirection usbDirection,
                      double in_Per_Tick, double lateral_In_PerTick, double track_Width_Ticks,
                      double KV, double KA, double KStatic,
                      double max_Wheel_Vel, double min_Profile_Accel, double max_Profile_Accel,
                      double max_Ang_Vel, double max_Ang_Accel,
                      double axial_Gain, double lateral_Gain, double heading_Gain,
                      double axial_Vel_Gain, double lateral_Vel_Gain, double heading_Vel_Gain) {
            logoFacingDirection = logoDirection;
            usbFacingDirection = usbDirection;
            inPerTick = in_Per_Tick;
            lateralInPerTick = lateral_In_PerTick;
            trackWidthTicks = track_Width_Ticks;
            kV = KV;
            kA = KA;
            kStatic = KStatic;
            maxWheelVel = max_Wheel_Vel;
            minProfileAccel = min_Profile_Accel;
            maxProfileAccel = max_Profile_Accel;
            maxAngVel = max_Ang_Vel;
        }
    }

    //private final String[] motor_names = {DriveConstants.leftFront, DriveConstants.leftBack, DriveConstants.rightFront, DriveConstants.rightBack};

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public DcMotorEx[] motors = new DcMotorEx[]{leftFront, leftBack, rightBack, rightFront};
    public final VoltageSensor voltageSensor;
    public final LazyImu lazyImu;
    public final Localizer localizer;
    FtcDashboard dashboard;

    boolean showDrivetrainTelem = true;
    boolean showElevatorTelem = true;
    boolean showIntakeTelem = true;
    boolean showOtherTelem = true;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public LED led;

    public DistanceSensor distanceSensor1, distanceSensor2;
    public boolean debugMode;

    public enum ROBOT {
        DEFAULT,
        ROBOT_1,
        ROBOT_2
    }

    public ROBOT robot = ROBOT.ROBOT_1;
    public Pose2d pose;

    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // Setting power reduction number Multiplying by (0.8) reduces the Maximum speed of the robot for more control
    // Setting power Increase number Multiplying by (0.2) Increases the Maximum speed of the robot for more control
    double DRIVE_SPEED = 0.25;
    double DRIVE_SPEED_INCREASED = DRIVE_SPEED + 0.15;
    double PowerReduction = DRIVE_SPEED - 0.15;
    double MAX_MOTOR_VELOCITY = 2400; //Blackfrog_Setting. Needed when motor is configured as RUN_TO_POSITION

    double lefFrontPower = DRIVE_SPEED;
    double leftBackPower = DRIVE_SPEED;
    double rightFrontPower = -DRIVE_SPEED;
    double rightBackPower = -DRIVE_SPEED;

    int sleepTime = 0;
    //Telemetry telemetry;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    //private final List<DcMotorEx> motors;

 // Params was here

    public static Params PARAMS = new Params(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,

            1,
            1,
            0,

            1.0 / rpmToVelocity(MAX_RPM),
            0.0,
            0.0,

            50,
            -30,
            50,

            Math.PI,
            Math.PI,

            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
    );

    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        this.pose = initialPose;

        switch (robot) {
            case DEFAULT:
                PARAMS = new Params(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,

                        1,
                        1,
                        0,

                        1.0 / rpmToVelocity(MAX_RPM),
                        0.0,
                        0.0,

                        50,
                        -30,
                        50,

                        Math.PI,
                        Math.PI,

                        0.0,
                        0.0,
                        0.0,

                        0.0,
                        0.0,
                        0.0
                );
                break;
            case ROBOT_1:
                PARAMS = new Params(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,

                        0.0347222222222222,
                        1,
                        0,

                        1.0 / rpmToVelocity(MAX_RPM),
                        0.0,
                        0.0,

                        50,
                        -30,
                        50,

                        Math.PI,
                        Math.PI,

                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                );
                break;
            case ROBOT_2:
                PARAMS = new Params(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,

                        0.03779289493575207860922146636432,
                        0.000048902012913635846,
                        271.57673251529997,

                        1.2359209955161816,
                        0.004420301924236487,
                        0.00000035,

                        50,
                        -30,
                        50,

                        Math.PI,
                        Math.PI,

                        1,
                        3,
                        2.5,
                        0.1,
                        1,
                        0.1
                );
               break;
        }

        final IMU imu;

        // For BulkCaching read - https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
      /*
        leftFront = hardwareMap.get(DcMotorEx.class, "lfrtch0");
        leftBack = hardwareMap.get(DcMotorEx.class, "lrrech1");
        rightFront = hardwareMap.get(DcMotorEx.class, "rftch2");
        rightBack = hardwareMap.get(DcMotorEx.class, "rrrch3");
*/
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftBack = hardwareMap.get(DcMotorEx.class, leftBackMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        rightBack = hardwareMap.get(DcMotorEx.class, rightBackMotorName);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // TODO: reverse encoders if needed
        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
        leftBack.setDirection(DcMotorEx.Direction.REVERSE); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
        rightFront.setDirection(DcMotorEx.Direction.FORWARD); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
        rightBack.setDirection(DcMotorEx.Direction.FORWARD); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.

        //motors = Arrays.asList(leftFront, leftBack, rightFront, rightBack);

        if (USE_DEAD_WHEEL_ODOMETRY) {
            USE_MOTOR_ENCODER = false; //Blackfrog_Setting. False if using deadwheel odometry
        }

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //Blackfrog_Setting.

            if (USE_MOTOR_ENCODER) {
                motor.setMode(RUN_USING_ENCODER); // You can still provide a power level in RUN_USING_ENCODER mode, but this is not recommended, as it will limit your target speed significantly. Setting a velocity from RUN_WITHOUT_ENCODER mode will automatically switch the motor to RUN_USING_ENCODER mode
                if (USE_MOTOR_ENCODER && MOTOR_VELO_PID != null) { //Blackfrog_Setting.
                    setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
                    //leftFront.setPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
                }
            } else {
                motor.setMode(RUN_WITHOUT_ENCODER); //Blackfrog_Setting. The RUN_WITHOUT_ENCODER motor mode is very straightforward, you simply set a power in the range of -1.0 to 1.0. However, if you try to set a velocity, the motor will automatically be switched into RUN_USING_ENCODER mode
            }
        }

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = lazyImu.get();

        if (showDrivetrainTelem) {
            telemetry.addLine("========== DRIVETRAIN INITIALIZED ==========");
            telemetry.addData("Initial pose: ", initialPose);
            telemetry.addData("Selected Robot: ", robot);
            telemetry.addData("logoFacingDirection: ", PARAMS.logoFacingDirection);
            telemetry.addData("MOTOR_NAMES: ", leftFrontMotorName, leftBackMotorName, rightFrontMotorName, rightBackMotorName);
        }

        if (USE_TWO_DEAD_WHEEL_ODOMETRY && !USE_MOTOR_ENCODER) {
            localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);
            //telemetry.addData("Localizer selected:", "Two deadwheels");
        } else if (USE_THREE_DEAD_WHEEL_ODOMETRY && !USE_MOTOR_ENCODER) {
            localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);
            //telemetry.addData("Localizer selected:", "Three deadwheels");
        } else if (USE_goBILDA_4_BAR_POD && !USE_MOTOR_ENCODER) {
            localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);

        } else {
            localizer = new DriveLocalizer();
           // telemetry.addData("Localizer selected:", "Default localizer");
        }

      //  telemetry.update();
/*
        if (debugMode) {
            sleep(sleepTime);
        }
*/

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    @Override
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
      // setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        led = new LED(hardwareMap, telemetry);
        led.initialize(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        if (debugMode) {
            sleepTime = 2000;
        } else {
            sleepTime = 0;
        }
    }

    @Override
    public void update(Telemetry telemetry) {

    }

    @Override
    public void stopSubsystem(Telemetry telemetry) {
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
    }

    public void increaseDrivePower() {
        axial = -(gamepad1.left_stick_y * DRIVE_SPEED_INCREASED);
        lateral = -(gamepad1.left_stick_x * DRIVE_SPEED_INCREASED);
        yaw = gamepad1.right_stick_x * DRIVE_SPEED_INCREASED;
        setDrivePowers((new PoseVelocity2d(new Vector2d(axial, lateral), yaw)));
/*
        telemetry.addData("increaseDrivePower", "");
        telemetry.addData("axial:", axial);
        telemetry.addData("lateral:", lateral);
        telemetry.addData("yaw:", yaw);
       // telemetry.update();

 */
    }

    public void decreaseDrivePower() {
        axial = -(gamepad1.left_stick_y * PowerReduction);
        lateral = -(gamepad1.left_stick_x * PowerReduction);
        yaw = gamepad1.right_stick_x * PowerReduction;
        setDrivePowers((new PoseVelocity2d(new Vector2d(axial, lateral), yaw)));
/*
        telemetry.addData("decreaseDrivePower", "");
        telemetry.addData("axial:", axial);
        telemetry.addData("lateral:", lateral);
        telemetry.addData("yaw:", yaw);
       // telemetry.update();

 */
    }

    public void SetTargetPosition(HardwareMap hardwareMap, Telemetry telemetry, double leftInches, double rightInches, double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();

        if(USE_MOTOR_ENCODER) {

          /*  To use RUN_TO_POSITION mode, you need to do the following things in this order:
            Set a target position (in ticks)
            Switch to RUN_TO_POSITION mode
            Set the maximum velocity

            You should reset the encoders (switch to STOP_AND_RESET_ENCODER mode) during initialization when you use RUN_TO_POSITION mode
          */

            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            int newLeftTarget = (int) (getRobotCurrentXPosition() + (int) (leftInches * COUNTS_PER_INCH));
            int newRightTarget = (int) getRobotCurrentYPosition() + (int) (rightInches * COUNTS_PER_INCH);

            //int ticks = (int) (leftInches * TICKS_PER_INCH);

            leftFront.setTargetPosition(newLeftTarget);
            leftBack.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            rightBack.setTargetPosition(newRightTarget);

            setRunMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            leftFront.setVelocity(MAX_MOTOR_VELOCITY);
            leftBack.setVelocity(MAX_MOTOR_VELOCITY);
            rightFront.setVelocity(MAX_MOTOR_VELOCITY);
            rightBack.setVelocity(MAX_MOTOR_VELOCITY);

            //setPower(DRIVE_SPEED, DRIVE_SPEED);

        } else {

            /*RUN_WITHOUT_ENCODER Mode
            Use this mode when you don’t want the Control Hub to attempt to use the encoders to
            maintain a constant speed. You can still access the encoder values, but your actual
            motor speed will vary more based on external factors such as battery life and friction.
            In this mode, you provide a power level in the -1 to 1 range,
                where -1 is full speed backwards,
                0 is stopped, and
                1 is full speed forwards.
            Reducing the power reduces both torque and speed. This mode is a good choice for
            drivetrain motors driven by joysticks on the gamepad.
*/
            // Get the current position and heading from the Pinpoint
            double currentX = odo.getPosX();
            double currentY = odo.getPosY();
            double currentHeading = odo.getHeading();

            // Use this information for your robot's movement logic
            // Determine new target position, and pass to motor controller
            int newLeftTarget = (int) (getRobotCurrentXPosition() + (int) (leftInches * COUNTS_PER_INCH));
            int newRightTarget = (int) getRobotCurrentYPosition() + (int) (rightInches * COUNTS_PER_INCH);

            double deltaX = newLeftTarget - currentX;
            double deltaY = newRightTarget - currentY;

            // Calculate motor powers based on the position difference
            // This is a simple example and may need to be adjusted for your robot
            double leftPower = deltaY + deltaX;
            double rightPower = deltaY - deltaX;

            //setPower(leftPower, rightPower);
            setPower(lefFrontPower, leftBackPower, rightFrontPower, rightBackPower);
        }

        runtime.reset();

        //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        while ((runtime.seconds() <= timeoutS) && isBusy()) {
            telemetry.addData("Moving forward, current position: ", leftFront.getCurrentPosition());
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
        }

        if(USE_MOTOR_ENCODER) {
            leftFront.setVelocity(0);
            leftBack.setVelocity(0);
            rightFront.setVelocity(0);
            rightBack.setVelocity(0);
        } else {
            resetPower(); //When using RUN_TO_POSITION the motor will continue to hold its position even after it has reached its target, unless you set the velocity or power to zero, or switch to a different motor mode.
        }

        //Initialize the drivetrain to what it was set during initialization
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void DriveForward(HardwareMap hardwareMap, Telemetry telemetry, double leftInches, double rightInches, double timeoutS) {
        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            lefFrontPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            leftBackPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            rightFrontPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            rightBackPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
        }
        SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
    }

    public void DriveBackward(HardwareMap hardwareMap, Telemetry telemetry, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            leftBack.setDirection(DcMotorEx.Direction.FORWARD);
            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            lefFrontPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            leftBackPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            rightFrontPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            rightBackPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
        }
        SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
    }

    public void strafeDriveLeft(HardwareMap hardwareMap,Telemetry telemetry, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            lefFrontPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            leftBackPower = - DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            rightFrontPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            rightBackPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
        }
        SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
    }

    public void strafeDriveRight(HardwareMap hardwareMap,Telemetry telemetry, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.FORWARD);
            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            lefFrontPower = -DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            leftBackPower =  DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
            rightFrontPower = - DRIVE_SPEED; // -ve power means motor is REVERSE moving when using RUN_WITHOUT_ENCODER
            rightBackPower = DRIVE_SPEED; // +ve power means motor is FORWARD moving when using RUN_WITHOUT_ENCODER
        }
        SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
    }

    public void turnDriveLeft(HardwareMap hardwareMap,Telemetry telemetry, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            leftBack.setDirection(DcMotorEx.Direction.FORWARD);
            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
        }
    }

    public void turnDriveRight(HardwareMap hardwareMap,Telemetry telemetry, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1785));

        if(USE_MOTOR_ENCODER) {
            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            SetTargetPosition(hardwareMap, telemetry, leftInches, rightInches, timeoutS);
        }
    }

    public void getRobotCurrentPosition(){
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Robot current position: ",  "%7d :%7d",odo.getPosX(),odo.getPosY());

        //telemetry.update();
    }

    public double getRobotCurrentXPosition(){
        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Robot current X position: ",  "%7d", odo.getPosX());

       // telemetry.update();
        return odo.getPosX();
    }

    public double getRobotCurrentYPosition(){
        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Robot current Y position: ",  "%7d", odo.getPosY());

      //  telemetry.update();
        return odo.getPosY();
    }

    public double[] getMotorVelocities() {
        return new double[]{leftFront.getVelocity(), leftBack.getVelocity(), rightFront.getVelocity(), rightBack.getVelocity()};
    }

    public boolean isBusy() {
        return leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy();
    }

    public void setRunMode(DcMotorEx.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void setPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void resetPower() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void resetVelocity() {
        leftFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightFront.setVelocity(0);
        rightBack.setVelocity(0);
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /*
    public Action followTrajectory(Trajectory t) {
        return new followTrajectory()
    }

    public Action turn(double angle) {
       // return new TodoAction();
    }

    public Action moveToPoint(double x, double y) {
       // return new TodoAction();
    }

    public SleepAction SleepDrive(double sleepTime) {
        return new TodoAction();
    }
*/
    public void setPIDFCoefficients(DcMotorEx.RunMode runMode, PIDFCoefficients coefficients) {
        assert voltageSensor != null;
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / voltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

 // Params was here

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFrontEncoder, leftBackEncoder, rightBackEncoder, rightFrontEncoder;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            leftFrontEncoder = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBackEncoder = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightFrontEncoder = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));
            rightBackEncoder = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            leftFrontEncoder.setDirection(DcMotorEx.Direction.FORWARD); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
            leftBackEncoder.setDirection(DcMotorEx.Direction.FORWARD); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
            rightFrontEncoder.setDirection(DcMotorEx.Direction.REVERSE); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
            rightBackEncoder.setDirection(DcMotorEx.Direction.REVERSE); // Blackfrog_Setting. If you’re using drive encoders, the ticks recorded should increase in a positive direction.
            initialized = false;
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFrontEncoder.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBackEncoder.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBackEncoder.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFrontEncoder.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }
/*
    Pose2D ftcPose = new Pose2D(10, 20, Math.PI/2); // Your FTC/ROS pose
    Pose2d roadRunnerPose = new Pose2d(
            ftcPose.getX(DistanceUnit.MM),
            ftcPose.getY(DistanceUnit.MM),
            ftcPose.getHeading(AngleUnit.DEGREES)
    );

    public static Pose2d convertPose2DToPose2d(Pose2D pose2D) {
        return new Pose2d(pose2D.getX(DistanceUnit.MM), pose2D.getY(DistanceUnit.MM), new Rotation2d(pose2D.getHeading(DistanceUnit.MM)));
    }
*/

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private double beginTs = -1;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kStatic,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kStatic,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
