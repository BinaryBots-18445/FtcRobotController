package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name="Close Auto", group="Robot")
public class RedFarAuto extends LinearOpMode {
    MechanumDrive md;
    HardwareControl hc;

    public void runOpMode() {
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);

        boolean run_once = true;
        while(opModeIsActive()) {
            telemetry.addData("Shooter Velocity", ((DcMotorEx) hc.shooter).getVelocity());
            telemetry.update();
            if(run_once)
            {
                for (int i = 0; i < 3; i++) {

                    ((DcMotorEx) hc.shooter).setVelocity(1350);
                    sleep(3000);
                    hc.agitator.setPower(1);
                    hc.feeder.setPower(-1);
                    sleep(1250);
                    hc.feeder.setPower(0);

                }

            }
            hc.shooter.setPower(0);

            md.MoveRobotBackwardsInches(60);

        }
        run_once = false;
    }
}

