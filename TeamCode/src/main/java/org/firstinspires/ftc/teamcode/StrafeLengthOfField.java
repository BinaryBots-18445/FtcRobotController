package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Make Robot Strafe Length Of Field")
public class StrafeLengthOfField extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        E e = new E(this);

        e.encoderDrive(20.0, e.DRIVE_SPEED,  120,  0, 120,0);


    }
}
