package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Make Robot Move Backwards Mechanum")
@Disabled
public class MakeRobotMoveBackwardsMechanum extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MoveRobotBackwardsInches(24000);
    }
}

