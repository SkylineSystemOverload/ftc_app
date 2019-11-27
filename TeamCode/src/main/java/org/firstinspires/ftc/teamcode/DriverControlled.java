package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverControlled", group="Test")

public class DriverControlled extends OpMode{

    // DEFINE robot
    RobotHardware robot = new RobotHardware();

    // CONSTANTS

    // RUN ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    }

    //LOOP ON init()
    @Override
    public void init_loop() {
    }

    //RUN ONCE ON start()
    @Override
    public void start() {
    }

    //LOOP ON start()
    @Override
    public void loop() {
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1rightStickX = gamepad1.right_stick_x;
        double G1leftStickX = gamepad1.left_stick_x;
        boolean G1a = gamepad1.a;
        boolean G1y = gamepad1.y;
        boolean G1x = gamepad1.x;
        boolean G1b = gamepad1.b;
        double G1LT = gamepad1.left_trigger;
        double G1RT = gamepad1.right_trigger;
        boolean G1dpad_right = gamepad1.dpad_right;
        boolean G1dpad_left = gamepad1.dpad_left;
        boolean G1dpad_up = gamepad1.dpad_up;
        boolean G1dpad_down = gamepad1.dpad_down;

        //mecanum drive
        robot.motor2.setPower((G1rightStickY) - (G1rightStickX) - (G1leftStickX));
        robot.motor4.setPower((G1rightStickY) - (G1rightStickX) + (G1leftStickX));
        robot.motor1.setPower((G1rightStickY) + (G1rightStickX) + (G1leftStickX));
        robot.motor3.setPower((G1rightStickY) + (G1rightStickX) - (G1leftStickX));


        //finger in
        if (G1dpad_down)
            robot.motor6.setPower(1);
        else if (G1dpad_up)
            //finger out
            robot.motor6.setPower(-1);
        else
            robot.motor6.setPower(0);


        //arm up
        if (G1LT > 0)
            robot.motor5.setPower(G1LT);
            //arm down
        else if (G1RT > 0)
            robot.motor5.setPower(-G1RT);
        else
            robot.motor5.setPower(0);



       /* //close finger
       if (G1a == true) {
          robot.motor7.setPower(1);
       }
       //open finger
       else if (G1b == true) {
          robot.motor7.setPower(-1);
       }
       else {
          robot.motor7.setPower(0);
       }
       */

        robot.servo1.setPosition(0.2);
        robot.servo2.setPosition(0.3);

        telemetry.addData("motor1 Power", robot.motor1.getPower());

        telemetry.addData("motor2 Power", robot.motor2.getPower());

        telemetry.addData("motor3 Power", robot.motor3.getPower());

        telemetry.addData("motor4 Power", robot.motor4.getPower());

        telemetry.addData("motor5 Power", robot.motor5.getPower());

        telemetry.addData("motor6 Power", robot.motor6.getPower());

        telemetry.addData("motor7 Power", robot.motor7.getPower());

        telemetry.addData("motor8 Power", robot.motor8.getPower());
    }

    // RUN ONCE ON stop()
    @Override
    public void stop() {
    }
}
