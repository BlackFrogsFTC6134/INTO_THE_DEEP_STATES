package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class RobotParams {
    /**
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_312_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
        public static final double MOTOR_5203_312_MAX_RPM       = 312.0;
        public static final double MOTOR_5203_312_MAX_VEL_PPS   =
                MOTOR_5203_312_ENC_PPR * MOTOR_5203_312_MAX_RPM / 60.0;     // 2795.9872 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_435_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
        public static final double MOTOR_5203_435_MAX_RPM       = 435.0;
        public static final double MOTOR_5203_435_MAX_VEL_PPS   =
                MOTOR_5203_435_ENC_PPR * MOTOR_5203_435_MAX_RPM / 60.0;     // 2787.9135 pps
    }   //class Gobilda

    /**
     * This enum specifies different drive modes.
     */
    public enum DriveMode
    {
        TankMode,
        ArcadeMode,
        HolonomicMode
    }   //enum DriveMode

    /**
     * This enum specifies all the drive orientation modes:
     * - ROBOT: Robot centric driving mode.
     * - FIELD: Field centric driving mode.
     * - INVERTED: Inverted driving mode (i.e. robot front becomes robot rear and vice versa).
     */
    public enum DriveOrientation
    {
        ROBOT, FIELD, INVERTED;

        public static DriveOrientation nextDriveOrientation(DriveOrientation driveOrientation)
        {
            DriveOrientation nextDriveOrientation;

            switch (driveOrientation)
            {
                case ROBOT:
                    nextDriveOrientation = FIELD;
                    break;

                case FIELD:
                    nextDriveOrientation = INVERTED;
                    break;

                default:
                case INVERTED:
                    nextDriveOrientation = ROBOT;
                    break;
            }

            return nextDriveOrientation;
        }   //nextDriveOrientation

    }   //enum DriveOrientation

    /**
     * This specifies the type of Drive Base Odometry is in use.
     */
    public enum OdometryType
    {
        Motor_Odometry,
        Two_DeadWheels,
        Three_DeadWheels    //e.g. SparkFun OTOS
    }   //enum OdometryType

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final Pose2d[] APRILTAG_POSES          = new Pose2d[] {
                new Pose2d(0.0, 0.0, 0.0),   // TagId 1
                new Pose2d(0.0, 0.0, 0.0),   // TagId 2
                new Pose2d(0.0, 0.0, 0.0),   // TagId 3
                new Pose2d(0.0, 0.0, 0.0)    // TagId 4
        };
    }   //class Game
    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             =
                Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcTeam";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.1;      // in msec
        public static final String ROBOT_CODEBASE               = "DWARAKA_1";
        public static final double ROBOT_LENGTH                 = 12.0;
        public static final double ROBOT_WIDTH                  = 12.0;

        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.HolonomicMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.6;
        public static final double DRIVE_NORMAL_SCALE           = 0.8;
        public static final double TURN_SLOW_SCALE              = 0.6;
        public static final double TURN_NORMAL_SCALE            = 0.8;
    }   //class Robot

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.MecanumRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showSubsystems              = true;
        // Vision
        public static final boolean useVision                   = false;
        public static final boolean useWebCam                   = false;    // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useAprilTagVision           = false;
        public static final boolean useColorBlobVision          = false;
        public static final boolean useLimelightVision          = false;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        // Drive Base
        public static final boolean useDriveBase                = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
    }   //class Preferences

    //
    // Robot Parameters.
    //

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams
    {
        static double INCHES_PER_MM = 0.03937007874015748;

        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public MecanumParams()
        {
            String robotName = "DWARAKA-1";
            // Robot Dimensions
            double robotLength = Robot.ROBOT_LENGTH;
            double robotWidth = Robot.ROBOT_WIDTH;
            double wheelBaseLength = (24.0 * 14)*INCHES_PER_MM;
            double wheelBaseWidth = 16.0;
            // IMU
            String imuName = "imu";
            RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

            /*
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"leftFront", "rightFront", "leftBack", "rightBack"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.MotorOdometry;

            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};
            xOdWheelIndices = new int[] {0};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};
            yOdWheelIndices = new int[] {1, 2};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};

            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count

            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);

            // Miscellaneous
            //blinkinName = "blinkin"; */
        }   //MecanumParams
    }   //class MecanumParams


    //
    // Subsystems.
    //

}   //class RobotParams
