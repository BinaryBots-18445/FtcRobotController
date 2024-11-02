package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Turn Robot 90 Degrees")
@Disabled
public class TurnRobot90Degrees extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        E e = new E(this);
        e.turnRobotDegrees(83);
    }
}
