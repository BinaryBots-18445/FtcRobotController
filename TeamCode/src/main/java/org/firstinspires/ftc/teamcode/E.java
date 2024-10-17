package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class E extends LinearOpMode {

    // E is basically strife drivbing + eventual other things probably <<33
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor front = null;
    private DcMotor right = null;
    private DcMotor back = null;
    private DcMotor left = null;

    //counts per motor rev means the number the encoder gives you when the shaft of the motor completes one full turn/revolution
    //https://www.andymark.com/products/neverest-classic-40-gearmotor

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // No External Gearing.
    //https://www.revrobotics.com/DUO-Omni-Wheels/ 90mm convert to inches
    static final double     WHEEL_DIAMETER_INCHES   = 90.0/25.4;     // For figuring circumference

    //Since the goal was to reach n degrees, and the total number of degrees is 360, we divided 360
    //by the number of sideways wheels, or 9. Then,  we divided that number by 90. We multiplied that by
    //the diameter (90 mm) converted to inches, (90/25.4) multiplied by pi and divided by 9
    //n/(360/9)*(90.0/25.4*pi)/9

 //to do a 180 degree, you have to substitute 180 in as n in this equation(n/(360/9)*(90.0/25.4*pi)/9
    //and that turns out to roughly 5.6 inches for a 180 degree turn
    //8.5 inches is the diameter of the circle that the wheels make
    //the distance from one point to another in the circle is 6 inches
    //find circumference and then divided by 4 to find the curve for 90 degrees
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        back = hardwareMap.get(DcMotor.class, "back");
        left = hardwareMap.get(DcMotor.class, "left");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        front.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        back.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);

        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // note from raymond: maybe we should use RUN_TO_POSITION instead?
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        Telemetry.Item item = telemetry.addData(
            "Starting position",
            "%7d %7d %7d %7d",
                front.getCurrentPosition(),
                right.getCurrentPosition(),
                back.getCurrentPosition(),
                left.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Path", "Starting");
        telemetry.update();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //the full wheel is 360 degrees, so therefore, the interval in between each vaguely triangle things is 40 degrees,
        //so to get 90 degrees, you have to rotate the wheel 2 1/4 sideways wheels.

        encoderDrive(20.0, DRIVE_SPEED,  0,  5, 0,5);
        // S1: Forward 47 Inches with 5 Sec timeout
        //(DRIVE_SPEED,   0, 0, 12, 0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  moves on the robots current position, based on encoder counts
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(
        double timeoutS,
        double speed,
        double frontInches,
        double rightInches,
        double backInches,
        double leftInches
    ) {
        int newFrontTarget;
        int newRightTarget;
        int newBackTarget;
        int newLeftTarget;

        telemetry.addData("Running If", "1");
        telemetry.update();

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newFrontTarget = front.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackTarget = back.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
            newLeftTarget = left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            front.setTargetPosition(newFrontTarget);
            right.setTargetPosition(newRightTarget);
            back.setTargetPosition(newBackTarget);
            left.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            front.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            back.setPower(Math.abs(speed));
            left.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            do { // || means or, and is &&
                    //(left.isBusy() && front.isBusy() && back.isBusy() && right.isBusy()))

//                telemetry.addData(
//                    "Timeout",
//                    "%7d %7d",
//                    runtime.seconds(),
//                    timeoutS);

                telemetry.addData(
                    "Are motors busy?",
                    "%b %b %b %b",
                    front.isBusy(),
                    right.isBusy(),
                    back.isBusy(),
                    left.isBusy());

                telemetry.addData(
                    "Target position",
                    "%7d %7d %7d %7d",
                    front.getTargetPosition(),
                    right.getTargetPosition(),
                    back.getTargetPosition(),
                    left.getTargetPosition());

                telemetry.addData(
                    "Current position",
                    "%7d %7d %7d %7d",
                    front.getCurrentPosition(),
                    right.getCurrentPosition(),
                    back.getCurrentPosition(),
                    left.getCurrentPosition());

                telemetry.update();
            } while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (right.isBusy() || left.isBusy()));

            // Stop all motion;
            front.setPower(0);
            right.setPower(0);
            back.setPower(0);
            left.setPower(0);

            // Turn off RUN_TO_POSITION
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }
}
