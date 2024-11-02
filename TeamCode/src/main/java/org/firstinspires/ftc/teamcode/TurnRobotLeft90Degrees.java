package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Turn Robot 90 Left Degrees")
public class TurnRobotLeft90Degrees extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        E e = new E(this);
        e.turnWithGyro(-90,0.1);
    }
}
