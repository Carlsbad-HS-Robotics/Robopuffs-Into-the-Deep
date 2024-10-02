package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotHardware {

    public LinearOpMode teleOp;

    //TODO what is IMU again??
    public IMU imu;
    public HardwareMap hardwareMap;

    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    } //RobotHardware functions

    //Hardware declarations
    public DcMotor liftMotor;

    //Wheels
    public DcMotor frontLeftMotor; // port 3
    public DcMotor frontRightMotor; // port 0
    public DcMotor backLeftMotor; // port 2
    public DcMotor backRightMotor; // port 1

    public Servo spinServo; //port 0



    //FUNCTIONS
    public void initMotor(DcMotor motor, boolean forward) {

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setDirection(DcMotor.Direction.FORWARD);
        if (!forward) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
        motor.setPower(0);

    }  //initializes a power motor
    public void initialize() {

        //Wheels
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        initMotor(frontLeftMotor, true);
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        initMotor(frontRightMotor, false);
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        initMotor(backLeftMotor, true);
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        initMotor(backRightMotor, false);

        //Lift Motor
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        initMotor(liftMotor, true);

        //Servo
        spinServo = hardwareMap.get(Servo.class, "spinServo");
        spinServo.setDirection(Servo.Direction.REVERSE);
        spinServo.scaleRange(0, 1);

    } // initializes all hardware

    public void robotCentricDrive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        double speedModifier = 0.4;

        frontLeftPower = frontLeftPower - (frontLeftPower*speedModifier);
        frontRightPower = frontRightPower - (frontRightPower*speedModifier);
        backLeftPower = backLeftPower - (backLeftPower*speedModifier);
        backRightPower = backRightPower - (backRightPower*speedModifier);

        //Sets power to motors

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    } //drive from robot POV


} //class RobotHardware