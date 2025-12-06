package org.firstinspires.ftc.teamcode;

import android.service.autofill.LuhnChecksumValidator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ClassUtil;



@Autonomous (name = "Coleton's Auto", group = "Robot")
public class ColetonsAuto extends LinearOpMode {
    public void shootArtifacts(double inches){
        hc.shooter.setPower(inches);
        System.out.println("Shooter Power: " + hc.shooter.getPower());
    }
    MechanumDrive md;
    HardwareControl hc;
    static double RPM = 6000;

    public void shootArtifacts(){
        hc.shooter.setPower(-0.9);
        double launchingPower = hc.shooter.getPower();
        System.out.println("Shooter Power: " + launchingPower);
        double rotations = RPM * launchingPower;
        System.out.println("RPM: " + rotations);
        sleep(5000);
        hc.agitator.setPower(1);
        double agitatorPower = hc.agitator.getPower();
        System.out.println("Agitator Power: " + agitatorPower);
        hc.feeder.setPower(-1);
        double feederPower = hc.feeder.getPower();
        System.out.println("Feeder Power: " + feederPower);
        sleep(1500);
        hc.feeder.setPower(0);
    }

    public void runOpMode() {
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);
        System.out.println("Moving");
        md.MoveRobotForwardInches(30);
        while(opModeIsActive()) {
            shootArtifacts();
        }
    }
}
