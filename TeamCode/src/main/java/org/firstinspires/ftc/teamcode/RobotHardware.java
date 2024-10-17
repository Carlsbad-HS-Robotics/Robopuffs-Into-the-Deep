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
    //public angleDiff;
    public DcMotor liftMotor; //port 0 E
    //Wheels
    public DcMotor frontLeftMotor; // port 3
    public DcMotor frontRightMotor; // port 0
    public DcMotor backLeftMotor; // port 2
    public DcMotor backRightMotor; // port 1
    public Servo spinServo; //port 0

    //TELEOP FUNCTIONS
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
        initMotor(liftMotor, false);

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

        double speedModifier = 0.5;

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

    /*
    public void fieldCentricDrive (double x, double y, double rx) { //Removed ", LinearOpMode teleop" -- if it stopped working that might be why

        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = botHeading+angleDiff;

        // Rotate the movement direction counter to the robot's rotation
        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = -x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        robotCentricDrive(rotX,rotY,rx);

    }

    */ //field centric

    //AUTONOMOUS FUNCTIONS

    int turnTime = 820;
    public void autoStop () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    } //Stop all drive movement

    public void autoMoveSquare(boolean forward, double numMats) {
        int driveTime = (int) (915 * numMats); //amount of time to drive one square (at 0.3 or 0.4 the speed)
        int multiplier = 1;
        if (!forward) {
            multiplier = -multiplier;
        }

        robotCentricDrive(0,multiplier, 0);
        teleOp.sleep(driveTime);
        stopDrive();

    } //drive one space

    public void autoLeft() {

        stopDrive();
        robotCentricDrive(0,0,-0.5);
        teleOp.sleep(turnTime);
        stopDrive();
    } //turn left

    public void autoOdoTurn(boolean left) {

    }

    public void autoRight() {
        stopDrive();
        robotCentricDrive(0,0,0.5);
        teleOp.sleep(turnTime);
        stopDrive();
    } //turn right

    public void stopDrive() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    } //stops all drive movement
    public void stopAll () {
        stopDrive();
        liftMotor.setPower(0);
        spinServo.setPosition(0.5);
    } //stops all motor, servo, etc. movement


} //class RobotHardware