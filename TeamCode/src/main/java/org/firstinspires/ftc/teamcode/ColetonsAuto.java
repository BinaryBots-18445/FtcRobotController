package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous (name = "Coleton's Auto", group = "Robot")
public class ColetonsAuto extends LinearOpMode {
    MechanumDrive md;
    HardwareControl hc;
    static double RPM = 6000;

    public void shootArtifacts(){
        System.out.println("Preparing Launcher");
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
        System.out.println("Shot Artifact");
    }

    public void runOpMode() {
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);
        System.out.println("Moving");
        md.MoveRobotForwardInches(30);
        while(opModeIsActive()) {
            shootArtifacts();
            System.out.println(RPM * 0.9);
        }
    }
}
