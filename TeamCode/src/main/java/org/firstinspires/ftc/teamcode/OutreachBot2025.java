package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OutreachBot2025{
    protected DcMotorEx front = null;
    protected DcMotorEx right = null;
    protected DcMotorEx back = null;
    protected DcMotorEx left = null;
    LinearOpMode opMode;

    public OutreachBot2025(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.setMsTransmissionInterval(50);
        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front = opMode.hardwareMap.get(DcMotorEx.class, "front");
        right = opMode.hardwareMap.get(DcMotorEx.class, "right");
        back = opMode.hardwareMap.get(DcMotorEx.class, "back");
        left = opMode.hardwareMap.get(DcMotorEx.class, "left");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        front.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.FORWARD);
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

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}
