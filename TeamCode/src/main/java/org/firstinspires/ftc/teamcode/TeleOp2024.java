package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Pushing the left  stick forward      makes the robot go forward
 * Pushing the left  stick backward     makes the robot go backward
 * Pushing the left  stick to the right makes the robot go to the rig ht
 * Pushing the left  stick to the left  makes the robot go to the left
 * Pushing the right stick to the right makes the robot turn clockwise
 * Pushing the right stick to the left  makes the robot turn counter clockwise
 */
@TeleOp(name="TeleOp2024", group="Robot")
public class TeleOp2024 extends LinearOpMode {

    // variables for motors
    // note: motors must be defined as member variables on the class
    //       so that they can be used by every function in the class

    DcMotor arm1;
    DcMotor arm2;
    DcMotor arm3;
    MechanumDrive e;




    /**
     * This function runs when the driver presses the INIT button on the driver station.
     * This function is called only ONCE.
     * This function initializes the motors so that they can be used in the loop function.
     */
    public void runOpMode() {
        e = new MechanumDrive(this);
        // initialize motors
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        arm3 = hardwareMap.get(DcMotor.class, "arm3");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//
//
//        // the motors on the left side of the robot need to be reversed
//        // because their axles point in the opposite direction as the motors on the right side of the robot
//        front.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.FORWARD);
//        back.setDirection(DcMotor.Direction.REVERSE);
//
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.REVERSE);


        // tell the driver that the robot is ready
        telemetry.addData(">", "Robot Ready. Press Play.");


    /**
     * This function runs when the driver presses the PLAY button on the driver station.
     * This function stops when the driver presses the STOP button on the driver station.
     * This function is called REPEATEDLY.
     */
    waitForStart();
    while(opModeIsActive()){
        // get inputs from the gamepad
        double leftY = gamepad1.left_stick_y;
        double leftX = -gamepad1.left_stick_x * 1.1;
        double rightX = gamepad1.right_stick_x;

        e.MoveRobot(leftY,leftX,rightX);
        double upButton1 = gamepad2.left_stick_y;
        double downButton1 = gamepad2.right_stick_y;

        if (upButton1 > 0) {
            arm1.setPower(1);
            arm3.setPower(1);
        }
        //down button is a on the game pad and up button is x on the game pad. when you press a both arms go down. when you press x both arms go up faster than they go down.
        //the if statements check if the two buttons on the gamepad are pressed and do the actions assigned to the button.
        else if (upButton1 < 0){
            arm1.setPower(-1);
            arm3.setPower(-1);

        }else{
            arm1.setPower(0);
            arm2.setPower(0);
            arm3.setPower(0);
        }
        if (downButton1 > 0){
            arm2.setPower(1);
        }
        else if (downButton1 < 0){
            arm2.setPower(-1);
        }
        else {
            arm1.setPower(0);
            arm2.setPower(0);
            arm3.setPower(0);
        }
        // send telemetry message to signify robot running
        // %.2f shows two decimal places
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("rightX", "%.2f", rightX);
        }
    }
}

