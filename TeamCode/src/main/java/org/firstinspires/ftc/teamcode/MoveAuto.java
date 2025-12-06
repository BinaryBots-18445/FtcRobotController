package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous (name="MoveAuto2025-2026", group="Robot")

public class MoveAuto extends LinearOpMode {
    public void shoot(double inches){
        hc.shooter.setPower(inches);
    }
    MechanumDrive md;
    HardwareControl hc;
    public void shootArtifacts(){
        hc.shooter.setPower(-0.7);
        sleep(5000);
        hc.agitator.setPower(1);
        hc.feeder.setPower(-1);
        sleep(1500);
        hc.feeder.setPower(0);
    }

    public void runOpMode() {
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);
        md.MoveRobotBackwardsInches(54);
        while(opModeIsActive()){
            shootArtifacts();
        }
    }

}



