package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Mecanum Autonomous Without Parking And Kick")
public class MechanumAutonomousWithoutParkingAndKick extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MakeRobotStrafeLeft(48);
        e.MakeRobotStrafeRight(18);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(14);
        e.MoveRobotBackwardsInches(48);
        e.MoveRobotForwardInches(58);
        e.MakeRobotStrafeLeft(10);
        e.MoveRobotBackwardsInches(54);

    }
}
