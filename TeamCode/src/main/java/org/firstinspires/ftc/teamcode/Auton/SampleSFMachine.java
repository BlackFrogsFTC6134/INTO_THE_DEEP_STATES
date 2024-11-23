package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import com.sfdev.assembly.state.*;

public class SampleSFMachine extends LinearOpMode {

    enum States {
        FIRST,
        SECOND,
        THIRD
    }

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                StateMachine machine = new StateMachineBuilder()
                .state(States.FIRST)
                .onEnter( () -> {
                    System.out.println( "Entering the first state" );
                })
                .transition( () ->  gamepad1.x ) // transition when gamepad1.x is clicked
                .onExit( () -> System.out.println("Exiting!") ) // setting check2 to false

                .state(States.SECOND)
                .onEnter( () -> System.out.println( "Entering the second state" ) )
                .transition( () -> gamepad1.b) // if check2 is false transition

                .state(States.THIRD)
                .onEnter( () -> System.out.println( "In the third state " ) )
                .build();
    }
}
