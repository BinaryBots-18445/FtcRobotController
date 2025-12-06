package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Grey's Auto", group="Robot")
public class GreysAuto extends LinearOpMode {
    MechanumDrive md;
    HardwareControl hc;
    public void runOpMode() {
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);

        boolean run_once = true;
        while(opModeIsActive()) {

            if(run_once)
            {
                for (int i = 0; i < 3; i++) {
                    if(i == 0) {
                        hc.shooter.setPower(-0.72);
                        sleep(7000);
                        hc.agitator.setPower(1);
                        hc.feeder.setPower(-1);
                        sleep(1500);
                        hc.feeder.setPower(0);
                    }if (i >= 1 && i <=3) {
                        hc.shooter.setPower(-0.72);
                        sleep(5000);
                        hc.agitator.setPower(1);
                        hc.feeder.setPower(-1);
                        sleep(1500);
                        hc.feeder.setPower(0);
                    }
                    }

                }
                hc.shooter.setPower(0);
                
                md.MoveRobotForwardInches(30);
            }
            run_once = false;
        }
    }

