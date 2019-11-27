package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class RobotHardware {
    //  INSTANTIATE MOTORS AND SERVOS
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    public DcMotor motor5;
    public DcMotor motor6;
    public DcMotor motor7;
    public DcMotor motor8;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;
    public Servo servo5;
    public Servo servo6;
    public Servo servo7;
    public Servo servo8;

    //INSTANTIATE SENSORS
    public GyroSensor gyroSensor;
    public ColorSensor colorSensor;

    //CREATE THE HARDWARE MAP
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // DEFINE MOTORS AND SERVOS
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor5 = hardwareMap.get(DcMotor.class, "motor5");
        motor6 = hardwareMap.get(DcMotor.class, "motor6");
        motor7 = hardwareMap.get(DcMotor.class, "motor7");
        motor8 = hardwareMap.get(DcMotor.class, "motor8");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        servo6 = hardwareMap.get(Servo.class, "servo6");
        servo7 = hardwareMap.get(Servo.class, "servo7");
        servo8 = hardwareMap.get(Servo.class, "servo8");

        //DEFINE SENSORS
        gyroSensor = hardwareMap.get(GyroSensor.class, "gyroSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        //SET MOTOR POWERS
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor5.setPower(0);
        motor6.setPower(0);
        motor7.setPower(0);
        motor8.setPower(0);

        //SET MOTOR MODES
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //...RunMode.RUN_TO_POSITION
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor7.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor8.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SET MOTOR zeroPowerBehavior
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //...ZeroPowerBehavior.FLOAT
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor8.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SET MOTOR DIRECTIONS
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        //SET SERVO POSITION

        //CALIBRATE SENSORS
        gyroSensor.calibrate();

    }
}
