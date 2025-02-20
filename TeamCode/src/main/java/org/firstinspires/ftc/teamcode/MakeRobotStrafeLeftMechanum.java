package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Make Robot Strafe Left Mechanum")
@Disabled
public class MakeRobotStrafeLeftMechanum extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MoveRobotForwardInches(4);
        e.MakeRobotStrafeLeft(42);
    }
}
