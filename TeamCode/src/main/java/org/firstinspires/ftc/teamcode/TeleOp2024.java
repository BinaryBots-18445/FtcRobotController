package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    DcMotor climber;
    DcMotor slide;
    Servo claw;
    DcMotor arm4;
    MechanumDrive e;

    int slideUpPosition = 6000;
    int slideDownPosition = 0;




    /**
     * This function runs when the driver presses the INIT button on the driver station.
     * This function is called only ONCE.
     * This function initializes the motors so that they can be used in the loop function.
     */
    public void runOpMode() {
        e = new MechanumDrive(this);
        // initialize motors
        climber = hardwareMap.get(DcMotor.class, "climber");
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        arm4 = hardwareMap.get(DcMotor.class, "arm4");

        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



//
//        // the motors on the left side of the robot need to be reversed
//        // because their axles point in the opposite direction as the motors on the right side of the robot
//        front.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.FORWARD);
//        back.setDirection(DcMotor.Direction.REVERSE);
//
        climber.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);


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
        boolean upDpad = gamepad2.dpad_up;
        boolean downDpad = gamepad2.dpad_down;
        boolean clawOpen = gamepad2.a;
        boolean clawClose = gamepad2.b;
        // when you press the left bumper, the slide goes down
        // when you let go of the left bumper, the slide stops
        boolean leftBumper = gamepad2.left_bumper;
        // kk
        float leftTrigger = gamepad2.left_trigger;

        if (upButton1 > 0) {
            climber.setPower(1);

        }
        //down button is a on the game pad and up button is x on the game pad. when you press a both arms go down. when you press x both arms go up faster than they go down.
        //the if statements check if the two buttons on the gamepad are pressed and do the actions assigned to the button.
        else if (upButton1 < 0){
            climber.setPower(-1);


        }else{
            climber.setPower(0);
//            arm2.setPower(0);

        }
        if (clawOpen){
            claw.setPosition(180);
        }
        if (clawClose){
            claw.setPosition(-180);
        }

        // slide stuff

        // go up to the high bar, even after letting the button go
        // if the slide isn't already at the high bar, move it up
        // if the slide is all the way up, this will not do anything
        if (upDpad && slide.getTargetPosition() != slideUpPosition && slide.getCurrentPosition() < slideUpPosition) {
            slide.setTargetPosition(slideUpPosition);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }
        // go all the way down, even after letting the button go
        // if the slide isn't already down, move it down
        else if (downDpad && slide.getTargetPosition() != slideDownPosition && slide.getCurrentPosition() > slideDownPosition) {
            slide.setTargetPosition(slideDownPosition);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(-1);
        }
        // go up while the button is pressed (manual mode)
        else if (leftTrigger > 0) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(leftTrigger);
        }
        // go down while the button is pressed (manual mode)
        else if (leftBumper) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(-1);
        }
        // if the slide isn't all the way up or all the way down,
        // and none of the buttons are pressed,
        // and the slide is in manual mode,
        // turn off the slide
        else if (slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            slide.setPower(0);
        }

        if (slide.getCurrentPosition() > slideUpPosition && slide.getPower() > 0) {
            slide.setPower(0);
        }

        if (slide.getCurrentPosition() < slideDownPosition && slide.getPower() < 0) {
            slide.setPower(0);
        }

        //

        // send telemetry message to signify robot running
        // %.2f shows two decimal places
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("rightX", "%.2f", rightX);
        telemetry.addData("current position", "%d", slide.getCurrentPosition());
        telemetry.addData("target position", "%d", slide.getTargetPosition());
        telemetry.addData("power", "%.2f", slide.getPower());
        telemetry.addData("current mode", "%s", slide.getMode().name());
        telemetry.update();
        }
    }
}

