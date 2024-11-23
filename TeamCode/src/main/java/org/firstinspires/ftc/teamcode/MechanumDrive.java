package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public void MoveRobotForwardOrBackwardsPercent(double speed){
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setPower(speed);
        right.setPower(speed);
        back.setPower(speed);
        left.setPower(speed);
    }
    public void MakeRobotStrafeLeftAndRight(double speed){
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setPower(speed);
        right.setPower(-speed);
        back.setPower(speed);
        left.setPower(-speed);
    }
    public void MakeRobotTurnLeftAndRight(double speed){
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setPower(-speed);
        right.setPower(-speed);
        back.setPower(speed);
        left.setPower(speed);
    }
    public MechanumDrive(LinearOpMode opMode) {
        super(opMode);
    }
}
