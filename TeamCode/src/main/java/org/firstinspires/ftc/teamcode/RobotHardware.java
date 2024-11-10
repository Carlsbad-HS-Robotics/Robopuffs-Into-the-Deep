/*
Robopuffs 2024-2025: Into the Deep
Author: Brielle McBarron
 */

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
    public LinearOpMode teleOp;

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
    public void initEncoderMotor (DcMotor motor, boolean forward) {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (forward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (!forward) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
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
        imu.resetYaw(); //When initialized, resets the core/base direction as where it's facing

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

    public void autoMoveSquare(boolean forward, double numMats) {
        //TODO make this encoder based so that it'll go off of distance not power; more precision
        int driveTime = (int) (915 * numMats); //amount of time to drive one square (at 0.3 or 0.4 the speed)
        int multiplier = 1;
        if (forward) {
            multiplier = -multiplier;
        }

        robotCentricDrive(0, multiplier,0);
        teleOp.sleep(driveTime);
        stopDrive();

    } //drive one space
    public double fixAngles (double angle) {
        //make everything 0 - 360
        if (angle > 360) {
            angle -= 360;
        } else if (angle < 0) {
            angle += 360;
        }

        return angle;

    }

    public void getBotHeadings() {

        //double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentDegreeHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (currentDegreeHeading < 0) {
            currentDegreeHeading += 360;
        }

        /*
        Left = +
        Right = -
        */

        //Angle Displays
        teleOp.telemetry.addData("DEGREES", "");
        teleOp.telemetry.addData("Current Heading: ", currentDegreeHeading);
        teleOp.telemetry.update();
    }

    public void autoOdoTurn(boolean left){

        stopAll();
        //******************************CALCULATIONS**************************************************
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); //get the way robots currently facing
        fixAngles(currentHeading); //make that on a 360 range


        //CALCULATE NEW GOAL HEADING & DIRECTION TO TURN
        double targetHeading = currentHeading;
        double turnPower = -0.3;
        if (left) {
            targetHeading += 90;
        } else {
            targetHeading -= 90;
            turnPower = 0.3;
        }
        fixAngles(targetHeading);


        //CALCULATE RANGE BASED OFF OF GOAL HEADING
        double offset = 5;
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

    }
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
    public void encoderArm() {

        liftMotor.setPower(0);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(0);

    }

    public void switchToEncoder(DcMotor motor, boolean forward) {
        motor.setPower(0);
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

} //class RobotHardware