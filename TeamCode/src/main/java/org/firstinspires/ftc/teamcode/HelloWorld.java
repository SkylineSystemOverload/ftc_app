package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="HelloWorld", group="Linear Opmode")
@Disabled
public class HelloWorld extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        double X2 , Y1 , X1 , threshold = 15;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Create "deadzone" for Y1/Ch3
            if(Math.abs(gamepad1.left_stick_y) > threshold)
                Y1 = gamepad1.left_stick_y;
            Y1 = 0;
//Create "deadzone" for X1/Ch4
            if(Math.abs(gamepad1.left_stick_x) > threshold)
                X1 = gamepad1.left_stick_x;
            else
                X1 = 0;
//Create "deadzone" for X2/Ch1
            if(Math.abs(gamepad1.right_stick_x) > threshold)
                X2 = gamepad1.right_stick_x;
            else
                X2 = 0;

//Remote Control Commands
            motor2.setPower(Y1 - X2 - X1);
            motor4.setPower(Y1 - X2 + X1);
            motor1.setPower(Y1 + X2 + X1);
            motor3.setPower(Y1 + X2 - X1);

            /*//drive forwards methods for auto later
            public void DriveForward(double power)
            {
                motor1.setPower(power);
                motor2.setPower(power);
                motor3.setPower(power);
                motor4.setPower(power);
            }

            public void Drivebackward(double -power)
            {
                motor1.setPower(power);
                motor2.setPower(power);
                motor3.setPower(power);
                motor4.setPower(power);
            }
            //Drive Forward
            DriveForward(1);
            sleep(4000);*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", motor1, motor2, motor3, motor4);
            telemetry.update();
        }
    }
}
