package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Make Robot Strafe Left")
public class FourWheelAutonomous extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {
        FourWheelDrive e = new FourWheelDrive(this);
        e.MakeRobotStrafeLeft(24);
    }
}
