package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Make Robot Strafe Left Mechanum")
public class MakeRobotStrafeLeftMechanum extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        MechanumDrive e = new MechanumDrive(this);
        e.MakeRobotStrafeLeft(2400);
    }
}
