/*
Robopuffs 2024-2025: Into the Deep
Author: Brielle McBarron
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap; //defines each piece of hardware to be coded
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotHardware {
    public LinearOpMode teleOp;

    //TODO what is IMU again??
    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    } //RobotHardware functions

    public double angleDiff = 0;
    public IMU imu;
    public HardwareMap hardwareMap;
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

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    } // initializes all hardware

    public void reInitImu() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        angleDiff = -newHeading;
    } //Reinitialize IMU

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

    public void fieldCentricDrive (double x, double y, double rx) {

        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = botHeading+angleDiff;

        // Rotate the movement direction counter to the robot's rotation
        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = -x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.2;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        robotCentricDrive(rotX,rotY,rx);

    }

    //field centric

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
        if (forward) {
            multiplier = -multiplier;
        }

        robotCentricDrive(0, multiplier,0);
        teleOp.sleep(driveTime);
        stopDrive();

    } //drive one space

    public void autoLeft() {

        stopDrive();
        robotCentricDrive(0,0,-0.5);
        teleOp.sleep(turnTime);
        stopDrive();
    } //turn left

    public void getBotHeadings() {

        //double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentDegreeHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (currentDegreeHeading < 0) {
            currentDegreeHeading += 360;
        }

        //TODO test angle sensing to get left/right directions (+ or -)
        /*
        Left
        Right
        */

        //Angle Displays
        teleOp.telemetry.addData("DEGREES", "");
        teleOp.telemetry.addData("Current Heading: ", currentDegreeHeading);
        //teleOp.telemetry.addData("RADIANS","");
        //teleOp.telemetry.addData("(Radians) Current Heading: ", currentHeading);

        teleOp.telemetry.update();


    }

    public double fixTargetHeading (double target) {
        if (target > 360) {
            target -= 360;
        } else if (target < 0) {
            target += 360;
        }

        return target;
    } //alters headings to be in a 0-360 degree range

    public double fixTargetHeading180 (double target, double nowDir) {
        //assumes that headings are on a 180-0 and -180-0 range
        double diff = Math.abs(target) - Math.abs(180);
        if (target > 180) {
            target = -180 + diff;
        } else if (target < -180) {
            target = 180 - diff;
        }
        return target;

    } // on a -180 to 180 range

    public void autoOdoTurn(boolean left){

        //TODO test this function
        stopAll();
        //calculate goal & current angles
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = currentHeading;
        if (left) {
            targetHeading = currentHeading - 90;
        } else {
            targetHeading = currentHeading + 90;
        }
        teleOp.telemetry.addData("targetHeading before fix: ", targetHeading);
        fixTargetHeading(targetHeading);
        //targetHeading = fixTargetHeading(targetHeading); //TODO uncomment this and remove prev line?
        teleOp.telemetry.addData("targetHeading after fix:", targetHeading);
        //TODO if the targetHeading before/after fix remains unchanged, uncomment other line and comment the other

        //Print angles
        teleOp.telemetry.addLine("Degrees");
        teleOp.telemetry.addData("(Degrees) Current Heading: ", currentHeading);
        teleOp.telemetry.addData("(Degrees) Goal Heading: ", targetHeading);
        teleOp.telemetry.addLine();
        teleOp.telemetry.update();

        teleOp.sleep(1000);

        teleOp.telemetry.addLine("Commencing movement...");
        teleOp.telemetry.update();

        while (currentHeading != targetHeading) {
            robotCentricDrive(0,0,-0.5);
        }

        //TODO if the previous while statement is too specific, try this
        /*
        //if the range is too small, add a few degrees
        double smaller = targetHeading - 0.5;
        double bigger = targetHeading + 0.5;

        while (currentHeading < smaller || currentHeading > bigger) {
            robotCentricDrive(0,0,-0.5);0
        }

         */ //Option 2

        teleOp.telemetry.addLine("Movement Complete.");
        teleOp.telemetry.update();

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