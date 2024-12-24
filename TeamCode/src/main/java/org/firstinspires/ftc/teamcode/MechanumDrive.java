package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        encoderDrive(20.0, DRIVE_SPEED, inches, -inches, -inches, -inches);
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
        double denominator = Math.max(Math.abs(forward) + Math.abs(rightward) + Math.abs(clockwise), 1);
        double frontLeftPower = (forward + rightward - clockwise) / denominator;
        double backLeftPower = (forward - rightward - clockwise) / denominator;
        double frontRightPower = (forward - rightward + clockwise) / denominator;
        double backRightPower = (forward + rightward + clockwise) / denominator;

        left.setPower(frontLeftPower);
        back.setPower(backLeftPower);
        front.setPower(-frontRightPower);
        right.setPower(backRightPower);
    }
    public MechanumDrive(LinearOpMode opMode) {
        super(opMode);
        front.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
    }
}
