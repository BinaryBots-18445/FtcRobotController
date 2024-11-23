package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MechanumDrive extends DrivetrainBase{
    public void MoveRobotBackwardsInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, -inches, -inches, -inches, -inches);
    }
    public void MakeRobotStrafeRight(double inches){
        encoderDrive(20.0, DRIVE_SPEED, -inches, inches, -inches, inches);
    }
    public void MakeRobotStrafeLeft(double inches){
        encoderDrive(20.0, DRIVE_SPEED, inches, -inches, inches, -inches);
    }

    public void MoveRobotForwardInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, inches, inches, inches, inches);
    }
    public MechanumDrive(LinearOpMode opMode) {
        super(opMode);
    }
}
