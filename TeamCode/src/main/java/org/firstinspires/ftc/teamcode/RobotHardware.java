/*
Robopuffs 2024-2025: Into the Deep
Author: Brielle McBarron
 */

//Imports: brings in a bunch of different commands and objects we can use
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap; //defines each piece of hardware to be coded
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotHardware {

    //FIRST: define all objects
    public LinearOpMode teleOp;
    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    } //RobotHardware functions
    public IMU imu;
    public double angleDiff = 0;
    //TODO add a 180 difference? to fix field centric
    public HardwareMap hardwareMap;

    //Define all components that we control with their type & name (I also like to put ports for convenience)

    //Wheels
    public DcMotor frontLeftMotor; // port 3
    public DcMotor frontRightMotor; // port 0
    public DcMotor backLeftMotor; // port 2
    public DcMotor backRightMotor; // port 1
    public Servo spinServo; //port 0
    //Servo that spins the wheel for intake
    public DcMotor extendMotor; //port 2 E
    //Motor that extends the slides

    //You can also add variables (like I did below)
    public int matDriveTime = 1150; //ms it takes to travel over one mat (VERY unspecific)

    //****************************************TELEOP FUNCTIONS****************************************
    /*
    Below are a bunch of functions (or methods) created by me (human). You can refer to these throughout this file or in other files
    such as MechTeleop by saying roboHardware.[method name]([parameters])
    */ //DESCRIPTION
    public void initEncoderMotor (DcMotor motor, boolean forward) {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (forward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (!forward) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());

    } //initializes an encoder motor
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

        //Slide Extension Motor
        extendMotor = hardwareMap.get(DcMotor.class, "extendMotor");
        initEncoderMotor(extendMotor, false);
        extendMotor.setTargetPosition(extendMotor.getCurrentPosition());

        //Servo
        spinServo = hardwareMap.get(Servo.class, "spinServo");
        spinServo.setDirection(Servo.Direction.REVERSE);
        spinServo.scaleRange(0, 1);

        //IMU - the IMU (Inertial Measurement Unit) is how the robot senses its direction/ orientation
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); //When initialized, resets the core/base direction as where it's facing

    } // initializes all hardware components
    public void robotCentricDrive(double x, double y, double rx) {
        //                  right stick x, right stick y, left stick x (formerly right stick x but we switched joysticks) (I didn't wanna change the variable names)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); //returns the sum or 1: whichever is bigger

        x = -x;

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
    public void reInitImu() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        angleDiff = -newHeading;
    } //Reinitialize IMU (resets direction that field centric is based off of-- resets which way "front" is)
    public void fieldCentricDrive (double x, double y, double rx) {
        //Removed ", LinearOpMode teleop" -- if it stopped working that might be why

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

    } //from driver POV (front is one direction, no matter which way robot is facing)
    public void getBotHeadings() {

        //double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentDegreeHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (currentDegreeHeading < 0) {
            currentDegreeHeading += 360;
        }

        /*
        Left = +
        Right = -
        */ //Directions

        //Angle Displays
        teleOp.telemetry.addData("DEGREES", "");
        teleOp.telemetry.addData("Current Heading: ", currentDegreeHeading);
        //teleOp.telemetry.update(); //Took this out because update line is in telemetry
    } //Prints robot headings out to the driver hub screen

    public void presetSlideLift(boolean y, boolean x, boolean a) {

        if (y) {
            extendMotor.setTargetPosition(8000);
        } //top
        else if (x) {
            extendMotor.setTargetPosition(4000);
        } //low
        else if (a) {
            extendMotor.setTargetPosition(0);
        } //bottom

        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

    }

    public void powerSlideLift(boolean RB, boolean LB) {
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (RB) {
            extendMotor.setPower(0.5);
        }
        else if (LB) {
            extendMotor.setPower(-0.5);
        }
        else {
            extendMotor.setPower(0);
        }
    }

    //****************************************AUTONOMOUS FUNCTIONS************************************
    public void autoMoveSquare(boolean forward, double numMats) {
        int driveTime = (int) (matDriveTime * numMats); //amount of time to drive one square (at 0.3 or 0.4 the speed)
        //driveTime -= 300;
        teleOp.telemetry.addData("driveTime: ",driveTime);

        //determine direction of movement based on whether it's going 'forward' (true) or backward (forward == false)
        int multiplier = 1; //multiply drive speed by this to make direction positive or negative
        if (forward) {
            multiplier = -multiplier;
        }

        teleOp.telemetry.addData("driveSpeed: ", multiplier);
        teleOp.telemetry.update();

        robotCentricDrive(0, multiplier,0); //tell the robot to drive
        teleOp.sleep(driveTime); //drive for X amount of time
        stopDrive();

    } //drive one space (NOT encoder based)
    //TODO add deadwheels; use encoders
    public void autoMoveSquareSide (boolean left, double numMats) {
        int driveTime = (int) (matDriveTime * numMats); //amount of time to drive one square (at 0.3 or 0.4 the speed)

        //determine direction of movement based on whether it's going 'forward' (true) or backward (forward == false)
        int multiplier = 1; //multiply drive speed by this to make direction positive or negative
        if (!left) {
            multiplier = -multiplier;
        }


        robotCentricDrive(multiplier, 0,0); //tell the robot to drive
        teleOp.sleep(driveTime); //drive for X amount of time
        stopDrive();
    }
    public double fixAngles (double angle) {
        //make everything 0 - 360
        if (angle > 360) {
            angle -= 360;
        } else if (angle < 0) {
            angle += 360;
        }

        return angle;

    } //make an angle based on 0-360 degree range
    public void autoOdoTurn(boolean left, int degrees){

        stopAll();
        //******************************CALCULATIONS**************************************************
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); //get the way robots currently facing
        fixAngles(currentHeading); //make that on a 360 range


        //CALCULATE NEW GOAL HEADING & DIRECTION TO TURN
        double targetHeading = currentHeading;
        double turnPower = -0.4;
        if (left) {
            targetHeading += degrees;
        } else {
            targetHeading -= degrees;
            turnPower = 0.4;
        }
        fixAngles(targetHeading);


        //CALCULATE RANGE BASED OFF OF GOAL HEADING
        double offset = 2;
        double smaller = targetHeading - offset;
        //fixAngles(smaller);
        double bigger = targetHeading + offset;
        //fixAngles(bigger);

        //******************************PRINT ANGLES**************************************************
        int currentSim = (int) currentHeading; //Simplified current heading (not some crazy decimal)
        int goalSim = (int) targetHeading; //Simplified goal heading
        int smallPar = (int) smaller;
        int bigPar = (int) bigger;
        teleOp.telemetry.addData("Current Heading: ", currentSim);
        teleOp.telemetry.addData("Goal Heading: ", goalSim);
        teleOp.telemetry.addLine();
        teleOp.telemetry.addData("Smaller limit:", smallPar);
        teleOp.telemetry.addData("Bigger limit:", bigPar);
        teleOp.telemetry.update();

        teleOp.sleep(2000);

        //******************************START TURNING**************************************************
        boolean angleReached = false;
        while (!angleReached) {
            //prevent infinite loop; get new heading and fix to 0-360
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            fixAngles(currentHeading);

            //RANGE FULFILLMENT
            boolean lessThanBig = false;
            boolean moreThanSmall = false;
            if (currentHeading < bigger) {
                lessThanBig = true;

            }
            if (currentHeading > smaller) {
                moreThanSmall = true;

            }

            //TEST IF AT THE GOAL ANGLE
            if (moreThanSmall && lessThanBig) {
                angleReached = true;
            } else {
                angleReached = false;
            }

            //PRINT ANGLES
            currentSim = (int) currentHeading;
            robotCentricDrive(0,0,turnPower);
            teleOp.telemetry.addData("Goal Heading: ", goalSim);
            teleOp.telemetry.addLine();
            teleOp.telemetry.addLine("(Big) " + bigPar + " > " + currentSim + " (Heading)");
            teleOp.telemetry.addLine("(Small) " + smallPar + " < " + currentSim + " (Heading)");
            teleOp.telemetry.addLine();
            teleOp.telemetry.update();

        }
        stopDrive();

        //Signify end of turn
        teleOp.telemetry.addLine("Movement Complete.");
        teleOp.telemetry.update();
        teleOp.sleep(1000);

    } //turn based on odometry
    public void autoSlideLift (int target) {
        extendMotor.setTargetPosition(target);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);
    }
    public void stopDrive() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    } //stops all drive movement
    public void stopAll () {
        stopDrive();
        extendMotor.setPower(0);
        spinServo.setPosition(0.5);
    } //stops all motor, servo, etc. movement


} //class RobotHardware
