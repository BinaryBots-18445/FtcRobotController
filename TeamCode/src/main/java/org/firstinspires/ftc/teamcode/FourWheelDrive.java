package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FourWheelDrive extends DrivetrainBase{

    public void TurnRobotDegrees(double degrees) {
        encoderDrive(20.0, TURN_SPEED, robotDegreesToWheelInches(degrees), robotDegreesToWheelInches(-degrees), robotDegreesToWheelInches(-degrees), robotDegreesToWheelInches(degrees));
    }
    public void MoveRobotForwardInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, 0, inches, 0, inches);
    }
    public void MoveRobotBackwardsInches(double inches) {
        encoderDrive(20.0, DRIVE_SPEED, 0, -inches, 0, -inches);
    }
    public void MakeRobotStrafeRight(double inches){
        encoderDrive(20.0, DRIVE_SPEED, inches, 0, inches, 0);
    }
    public void MakeRobotStrafeLeft(double inches){
        encoderDrive(20.0, DRIVE_SPEED, -inches, 0, -inches, 0);
    }
    public void turnWithGyro(double degrees, double tolerance) {
        if (opMode.opModeIsActive()) {
            double Kp = 1.0 / degrees;
            double minPower = 0.05;
//            double Ki = -0.5 * Kp;
//            double accError = 0;
            double Kd = 25 * Kp;
            double Perror = 0;
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front.setPower((TURN_SPEED));
            right.setPower((-TURN_SPEED));
            back.setPower((-TURN_SPEED));
            left.setPower((TURN_SPEED));
            while (opMode.opModeIsActive()) {
                double heading = -1 * getGyroHeading();
                double error = degrees - heading;
//                accError += error;
                double power = Kp * error + Kd * (error - Perror);
                if (Math.abs(power) < minPower){
                    if(power > 0){
                        power = minPower;
                    }
                    else{
                        power = -minPower;
                    }
                }
                Perror = error;
                if (degrees < 0) {
                    spinTurnWithPower(-power);
                } else {
                    spinTurnWithPower(power);
                }


                if (Math.abs(error) < tolerance) {
                    front.setPower(0);
                    right.setPower(0);
                    back.setPower(0);
                    left.setPower(0);
                    break;

                }


                opMode.telemetry.addData(
                        "heading",
                        "%2f",
                        heading
                );

                opMode.telemetry.addData(
                        "velocity",
                        "%2f %2f %2f %2f",
                        front.getVelocity(),
                        right.getVelocity(),
                        back.getVelocity(),
                        left.getVelocity());
                opMode.telemetry.addData(
                        "zero power behavior",
                        "%s %s %s %s,",
                        front.getZeroPowerBehavior().name(),
                        right.getZeroPowerBehavior().name(),
                        back.getZeroPowerBehavior().name(),
                        left.getZeroPowerBehavior().name());

                opMode.telemetry.update();

            }


            opMode.telemetry.addData("Robot Stopped", "");
        }

    }



    public void spinTurnWithPower(double power) {
        front.setPower((power));
        right.setPower((-power));
        back.setPower((-power));
        left.setPower((power));
    }

    public FourWheelDrive(LinearOpMode opMode) {
        super(opMode);

    }
}
