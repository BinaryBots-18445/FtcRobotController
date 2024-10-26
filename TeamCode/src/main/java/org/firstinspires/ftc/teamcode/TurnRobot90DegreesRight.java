package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Turn Robot Right 90 Degrees")
public class TurnRobot90DegreesRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        E e = new E(this);
        e.turnWithGyro(90,3);
    }
}
