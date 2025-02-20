package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test", group="Robot")
public class Test extends LinearOpMode {
    public void runOpMode() {
        DcMotor frontLeft = null;
        frontLeft = hardwareMap.get(DcMotor.class, "HT");
        frontLeft.setPower(0.5);
        sleep(5000);
        frontLeft.setPower(-0.5);
        sleep(5000);
        }
    }

