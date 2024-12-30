package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.constants.Constants.GLOBAL_DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.constants.PivotConstants.PIVOT_DEBUG_MODE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PivotSubsystem;
import org.firstinspires.ftc.teamcode.utils.Mechanism; /**
 * Author: Kavish Karia
 * Date create: 12/15/2024
 * FTC Team: 6134 - Black Frog
 * Comments:
 */

/**
 * pivotServosBackCMD() command is used to set the both pivot servos to the up position.
 */
public class pivotServosBackCMD extends Mechanism {
    private final PivotSubsystem pivotSubsystem;
    private final ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;

    public pivotServosBackCMD(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
    }

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     *
     * @param hardwareMap robot's hardware map
     * @throws RuntimeException if something goes wrong
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        pivotSubsystem.pivotServosBack();
        if (GLOBAL_DEBUG_MODE || PIVOT_DEBUG_MODE) {
            pivotSubsystem.telemetry(telemetry);
        }
    }
}
