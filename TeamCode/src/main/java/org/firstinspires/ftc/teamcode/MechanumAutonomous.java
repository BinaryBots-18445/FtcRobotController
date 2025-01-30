package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Mecanum Autonomous")
public class MechanumAutonomous extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MakeRobotStrafeLeft(48);
        e.MakeRobotStrafeRight(24);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(18);
        e.MoveRobotBackwardsInches(54);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(10);
        e.MoveRobotBackwardsInches(52);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(6);
        e.MoveRobotBackwardsInches(48);
        e.MakeRobotStrafeRight(132);
    }
}
