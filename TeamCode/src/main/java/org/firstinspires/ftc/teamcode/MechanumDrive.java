package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MechanumDrive extends DrivetrainBase{

    public void turnWithGyro(double degrees, double tolerance) {
        if (opMode.opModeIsActive()) {
            double Kp = 1.0 / degrees;
            double minPower = 0.05;
//            double Ki = -0.5 * Kp;
//            double accError = 0;
            double Kd = 25 * Kp;
            double Perror = 0;
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setPower((TURN_SPEED));
            frontright.setPower((-TURN_SPEED));
            backleft.setPower((-TURN_SPEED));
            backright.setPower((TURN_SPEED));
            while (opMode.opModeIsActive()) {
//                double heading = -1 * getGyroHeading();
//                double error = degrees - heading;
////                accError += error;
//                double power = Kp * error + Kd * (error - Perror);
//                if (Math.abs(power) < minPower){
//                    if(power > 0){
//                        power = minPower;
//                    }
//                    else{
//                        power = -minPower;
//                    }
//                }
//                Perror = error;
//                if (degrees < 0) {
//                    spinTurnWithPower(-power);
//                } else {
//                    spinTurnWithPower(power);
//                }

//
//                if (Math.abs(error) < tolerance) {
//                    frontleft.setPower(0);
//                    frontright.setPower(0);
//                    backleft.setPower(0);
//                    backright.setPower(0);
//                    break;
//
//                }
//
//
//                opMode.telemetry.addData(
//                        "heading",
//                        "%2f",
//                        heading
//                );

                opMode.telemetry.addData(
                        "velocity",
                        "%2f %2f %2f %2f",
                        frontleft.getVelocity(),
                        frontright.getVelocity(),
                        backleft.getVelocity(),
                        backright.getVelocity());
                opMode.telemetry.addData(
                        "zero power behavior",
                        "%s %s %s %s,",
                        frontleft.getZeroPowerBehavior().name(),
                        frontright.getZeroPowerBehavior().name(),
                        backleft.getZeroPowerBehavior().name(),
                        backright.getZeroPowerBehavior().name());

                opMode.telemetry.update();

            }


            opMode.telemetry.addData("Robot Stopped", "");
        }

    }
    public void MoveRobotBackwardsInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, inches * 0.8, inches * 0.8, inches * 0.8, inches);
    }
    public void spinTurnWithPower(double power) {
        frontleft.setPower((power));
        frontright.setPower((-power));
        backleft.setPower((-power));
        backright.setPower((power));
    }
    public void MakeRobotStrafeLeft(double inches){
        encoderDrive(20.0, DRIVE_SPEED, inches, inches, -inches, inches);
    }
    public void MakeRobotStrafeRight(double inches){
        encoderDrive(20.0, DRIVE_SPEED, -inches, -inches, inches, -inches);
    }

    public void MoveRobotForwardInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, -inches, -inches, -inches, -inches);
    }
    public void MoveRobotForwardOrBackwardsPercent(double speed){
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);
    }
    public void MakeRobotStrafeLeftAndRight(double speed){
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setPower(speed);
        frontright.setPower(-speed);
        backleft.setPower(speed);
        backright.setPower(-speed);
    }
    public void MakeRobotTurnLeftAndRight(double speed){
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setPower(-speed);
        frontright.setPower(-speed);
        backleft.setPower(speed);
        backright.setPower(speed);
    }
    //    forward is going backwards and forward, right is strafing right and left, turning is turning right and left
//    give forward positive number = go forward, if give negative = go backwards
//    giving rightward a positive number makes it strafe right, if give negative = strafe left
//    give clockwise positive = turn right, if give negative = turn left
    //denominator is making sure the gamepad values won't exceed 1 (aka 100%)
    public void MoveRobot(double forward, double strafing, double clockwise, double speedMultiplier){
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafing) + Math.abs(clockwise), 1);

        double frontLeftPower = (forward + strafing - clockwise) / denominator;
        double backLeftPower = (forward - strafing - clockwise) / denominator;
        double frontRightPower = (forward - strafing + clockwise) / denominator;
        double backRightPower = (forward + strafing + clockwise) / denominator;

        backright.setPower(backRightPower * speedMultiplier);
        backleft.setPower(backLeftPower * speedMultiplier);
        frontleft.setPower(frontLeftPower * speedMultiplier);
        frontright.setPower(frontRightPower * speedMultiplier);
    }
    public MechanumDrive(LinearOpMode opMode) {
        super(opMode);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
    }


}
// # budgies