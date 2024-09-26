package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Coin", group="Robot")

public class Coin extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;

    double clawOffset = 0;

    //public static final double MID_SERVO   =  0.5 ;
   // public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    //public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    //public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        // Define and Initialize Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "LF");
        backLeft  = hardwareMap.get(DcMotor.class, "LB");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backRight  = hardwareMap.get(DcMotor.class, "RB");
        //leftArm    = hardwareMap.get(DcMotor.class, "left_arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        //rightClaw = hardwareMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        double lefty;
        double leftx;
        double right;
        boolean limitSwitchOne;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        lefty = gamepad1.left_stick_y;
        leftx = gamepad1.left_stick_x;
        right = gamepad1.right_stick_x;

        frontLeft.setPower(-lefty - leftx + right);
        frontRight.setPower(lefty + leftx + right);
        backLeft.setPower(-lefty - leftx - right);
        backRight.setPower(lefty + leftx - right);
        // Use gamepad left & right Bumpers to open and close the claw
        //if (gamepad1.right_bumper)
        //    clawOffset += CLAW_SPEED;
        //else if (gamepad1.left_bumper)
        //    clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        //leftClaw.setPosition(MID_SERVO + clawOffset);
        //rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        //if (gamepad1.y)
        //    leftArm.setPower(ARM_UP_POWER);
        //else if (gamepad1.a)
        //    leftArm.setPower(ARM_DOWN_POWER);
        //else
        //    leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", lefty);
        telemetry.addData("left",  "%.2f", leftx);
        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
    }
}
