package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MechanumDrive extends DrivetrainBase{
    public void MoveRobotBackwardsInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, -inches, -inches, -inches, -inches);
    }
    public void MakeRobotStrafeRight(double inches){
        encoderDrive(20.0, DRIVE_SPEED, inches, -inches, inches, -inches);
    }
    public void MakeRobotStrafeLeft(double inches){
        encoderDrive(20.0, DRIVE_SPEED, -inches, inches, -inches, inches);
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
//    forward is going backwards and forward, right is strafing right and left, turning is turning right and left
//    give forward positive number = go forward, if give negative = go backwards
//    giving rightward a positive number makes it strafe right, if give negative = strafe left
//    give clockwise positive = turn right, if give negative = turn left
    public void MoveRobot(double forward, double rightward, double clockwise){
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (forward > 0) {
            front.setPower(forward);
            right.setPower(forward);
            back.setPower(forward);
            left.setPower(forward);
        }
        if (forward < 0) {
            front.setPower(-forward);
            right.setPower(-forward);
            back.setPower(-forward);
            left.setPower(-forward);
        }
        if (rightward < 0) {
            front.setPower(rightward);
            right.setPower(-rightward);
            back.setPower(rightward);
            left.setPower(-rightward);
        }
        if (rightward > 0) {
            front.setPower(-rightward);
            right.setPower(rightward);
            back.setPower(-rightward);
            left.setPower(rightward);
        }
        if (clockwise > 0) {
            front.setPower(clockwise);
            right.setPower(-clockwise);
            back.setPower(clockwise);
            left.setPower(-clockwise);
        }
        if (clockwise < 0) {
            front.setPower(-clockwise);
            right.setPower(clockwise);
            back.setPower(-clockwise);
            left.setPower(clockwise);
        }
    }
    public MechanumDrive(LinearOpMode opMode) {
        super(opMode);
    }
}
