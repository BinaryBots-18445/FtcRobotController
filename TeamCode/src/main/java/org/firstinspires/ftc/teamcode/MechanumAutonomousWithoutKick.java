package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Mecanum Autonomous Without Kick")
public class MechanumAutonomousWithoutKick extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MakeRobotStrafeLeft(48);
        e.MakeRobotStrafeRight(18);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(14);
        e.MoveRobotBackwardsInches(54);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(10);
        e.MoveRobotBackwardsInches(54);
        e.MakeRobotStrafeRight(132);
        e.MoveRobotBackwardsInches(14);
    }
}
//try 1: 2/4
//try 2: 2/4
//try 3: 2/4
//try: 2/4