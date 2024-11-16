package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Pushing the left  stick forward      makes the robot go forward
 * Pushing the left  stick backward     makes the robot go backward
 * Pushing the left  stick to the right makes the robot go to the right
 * Pushing the left  stick to the left  makes the robot go to the left
 * Pushing the right stick to the right makes the robot turn clockwise
 * Pushing the right stick to the left  makes the robot turn counter clockwise
 */
@TeleOp(name="TeleOp2024", group="Robot")
public class TeleOp2024 extends OpMode {

    // variables for motors
    // note: motors must be defined as member variables on the class
    //       so that they can be used by every function in the class
    DcMotor front;
    DcMotor right;
    DcMotor back;
    DcMotor left;


    /**
     * This function runs when the driver presses the INIT button on the driver station.
     * This function is called only ONCE.
     * This function initializes the motors so that they can be used in the loop function.
     */
    public void init() {
        // initialize motors
        front   = hardwareMap.get(DcMotor.class, "front");
        right    = hardwareMap.get(DcMotor.class, "right");
        back  = hardwareMap.get(DcMotor.class, "back");
        left   = hardwareMap.get(DcMotor.class, "left");


        // the motors on the left side of the robot need to be reversed
        // because their axles point in the opposite direction as the motors on the right side of the robot
        front.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.REVERSE);
//        left.setDirection(DcMotor.Direction.FORWARD);


        // tell the driver that the robot is ready
        telemetry.addData(">", "Robot Ready. Press Play.");
    }

    /**
     * This function runs when the driver presses the PLAY button on the driver station.
     * This function stops when the driver presses the STOP button on the driver station.
     * This function is called REPEATEDLY.
     */
    public void loop() {
        // get inputs from the gamepad
        double leftY = -gamepad1.left_stick_y;
        double leftX = -gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        // the left and right motors make the robot go backward and forward
        front.setPower(leftX - rightX/2);
        back.setPower(-leftX - rightX/2);
        right.setPower(-leftY + rightX/2);
        left.setPower(leftY + rightX/2);
        // the middle motors make the robot go to the left or right
        // note: turn is too sensitive at default so divide rightX by 2


        // send telemetry message to signify robot running
        // %.2f shows two decimal places
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("rightX", "%.2f", rightX);
    }
}