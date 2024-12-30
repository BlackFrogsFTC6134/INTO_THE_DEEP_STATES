package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.RobotHardware.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.RobotHardware.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.RobotHardware.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.RobotParams;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * Author: Kavish Karia
 * Date create: 12/28/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */

@TeleOp(name = "RobotHardware")
public class RobotHardware extends LinearOpMode {
//    public Arm arm;
    Pivot pivot;
    Telemetry telemetry;

    ArrayList<Subsystem> subsystems = new ArrayList<>();

    private static RobotHardware instance;

    public RobotHardware() {
       //arm = new Arm();
       //subsystems.add(arm);

        pivot = new Pivot();
        subsystems.add(pivot);
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initialize(hardwareMap, telemetry);

        //pivot.initialize(hardwareMap, telemetry);
        while(opModeIsActive()){

        }
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }

        return instance;
    };

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        for(Subsystem subsystem : subsystems) {
            subsystem.initialize(hardwareMap, telemetry);
        }
    }

    public void sendTelemetry(Telemetry telemetry){
        for(Subsystem subsystem : subsystems) {
            subsystem.sendTelemetry(telemetry);
        }
    }
}
