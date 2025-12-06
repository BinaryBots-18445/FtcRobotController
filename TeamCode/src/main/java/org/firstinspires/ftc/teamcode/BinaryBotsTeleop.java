package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * Pushing the left  stick forward      makes the robot go forward
 * Pushing the left  stick backward     makes the robot go backward
 * Pushing the left  stick to the right makes the robot go to the right
 * Pushing the left  stick to the left  makes the robot go to the left
 * Pushing the right stick to the right makes the robot turn clockwise
 * Pushing the right stick to the left  makes the robot turn counter clockwise
 */


@TeleOp(name="BinaryBotsTeleop2025-2026", group="Robot")
public class BinaryBotsTeleop extends LinearOpMode {

    // variables for motors
    // note: motors must be defined as member variables on the class
    //       so that they can be used by every function in the class

    MechanumDrive md;
    HardwareControl hc;
    AprilTagDetection myAprilTagDetection;

    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true; // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private static final int DESIRED_TAG_ID = 21 ;
    double myAprilTagDetections;
    boolean targetFound = false;
    VisionPortal visionPortal;



    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(736.342,736.342,371.36,277.445)
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
                if (detection.id == DESIRED_TAG_ID) {
                    // This is the tag we want to move towards
                    targetFound = true;
                    // Set here so that we know which tag is detected.
                    desiredTag = detection;

                    telemetry.addData("April tag found: ",  detection.id);
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", 2, desiredTag.ftcPose.bearing, 2));

                    md.MoveRobot(desiredTag.ftcPose.range, 0,0,1);
                    telemetry.addLine(String.format("\n==== (ID %d) %s", desiredTag.id, desiredTag.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", desiredTag.ftcPose.pitch, desiredTag.ftcPose.roll, desiredTag.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.elevation));        // Add "key" information to telemetry
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                    telemetry.addLine("RBE = Range, Bearing & Elevation");


//

                    sleep(1000);
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    targetFound = false;
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            }
      //  }
    }


    /**
     * This function runs when the driver presses the INIT button on the driver station.
     * This function is called only ONCE.
     * This function initializes the motors so that they can be used in the loop function.
     */
    public void runOpMode() {
        initAprilTag();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        md = new MechanumDrive(this);
        hc = new HardwareControl(this);

        // tell the driver that the robot is ready
        telemetry.addData(">", "Robot Ready. Press Play.");


        /**
         * This function runs when the driver presses the PLAY button on the driver station.
         * This function stops when the driver presses the STOP button on the driver station.
         * This function is called REPEATEDLY.
         */
        boolean slowMode = false;
        boolean shooterSlow = false;
        boolean aPressedLast = gamepad1.a;
        boolean shooterButton = gamepad2.a;
        boolean agitatorButton = gamepad2.b;
        boolean feederButton = gamepad2.left_bumper;
        boolean x2PressedLast = gamepad2.x;
        boolean rb2PressedLast = gamepad2.right_bumper;
        boolean y2PressedLast = gamepad2.y;
        boolean shooterStart = false;
        boolean aStart = false;
        boolean b2pressedLast = gamepad2.b;
        boolean a2PressedLast = gamepad2.a;
        boolean fStart = false;
        boolean lb2PressedLast = gamepad2.left_bumper;
        double leftY = gamepad1.left_stick_y;
        double leftX = -gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double shooterSpeed = 0.7;
        while(opModeIsActive()) {
//            telemetryAprilTag();
            //telemetry.update();
            sleep(20);
            leftY = gamepad1.left_stick_y;
            leftX = -gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;
            shooterButton = gamepad2.a;
            agitatorButton = gamepad2.b;
            if (gamepad1.a && !aPressedLast) {
                slowMode = !slowMode; //toggle
            }
            aPressedLast = gamepad1.a;


            double speedMultiplier = slowMode ? 0.5 : 1.0;
            double shooterMultiplier = shooterSlow ? 0.7 : 0.9;
            md.MoveRobot(leftY, leftX, rightX, speedMultiplier);
            telemetry.addData("current speed", speedMultiplier);
            telemetry.addData("shooter speed", shooterSpeed);
            telemetry.update();
            if (gamepad2.right_bumper && !rb2PressedLast) {
                shooterSpeed = 0.7;
            }
            rb2PressedLast = gamepad2.right_bumper;
            if (gamepad2.y && !y2PressedLast) {
                shooterStart = !shooterStart;
            }

            if (gamepad2.x && !x2PressedLast) {
                shooterSpeed = 0.8;
            }
            x2PressedLast = gamepad2.right_bumper;
            if (gamepad2.left_bumper && !lb2PressedLast) {
                shooterSpeed = 0.9;
            }
            lb2PressedLast = gamepad2.right_bumper;
            y2PressedLast = gamepad2.y;
            if (shooterStart) {
                hc.shooter.setPower(-shooterSpeed);
            }else{
                hc.shooter.setPower(0);

            }
            if (gamepad2.b && !b2pressedLast){
                aStart = !aStart;
            }
            b2pressedLast = gamepad2.b;
            if (aStart){
                hc.agitator.setPower(1);
            }else{
                hc.agitator.setPower(0);
            }
            if (gamepad2.a && !a2PressedLast){
                fStart = !fStart;
            }
            a2PressedLast = gamepad2.a;
            if (fStart){
                hc.feeder.setPower(-1);
            }else{
                hc.feeder.setPower(0);
            }
        }
            telemetry.update();
        }
    }


