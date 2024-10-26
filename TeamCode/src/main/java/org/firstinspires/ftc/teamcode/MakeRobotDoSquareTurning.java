package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Make Robot Do Square Turning")
public class MakeRobotDoSquareTurning extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        E e = new E(this);
        e.encoderDrive(20.0, e.DRIVE_SPEED,  0,  24, 0,24);
        e.encoderDrive(20.0, e.DRIVE_SPEED,  24,  0, 24,0);
        e.turnWithGyro(180,6);
        e.encoderDrive(20.0, e.DRIVE_SPEED,  0,  24, 0,24);
        e.turnWithGyro( 90,3);
        e.encoderDrive(20.0, e.DRIVE_SPEED,  0,  24, 0,24);

    }
}
