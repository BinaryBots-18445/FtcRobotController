package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Turn Robot 180 Degrees")
public class TurnRobot180Degrees extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);
        E e = new E(this);
        e.turnWithGyro(179,6);

    }
}
