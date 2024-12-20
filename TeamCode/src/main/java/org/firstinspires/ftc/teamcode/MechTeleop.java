package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

/*TODO
Test odometry / heading sensors
Figure out math for odometry
get turn figured out
 */

@TeleOp(name="Mechanism TeleOp", group="TeleOps")
public class  MechTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        waitForStart();

        double liftMotorPower = 0.3;

        while (opModeIsActive()) {



            //******************************GAME FUNCTIONS******************************

            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
            roboHardware.getBotHeadings(); //print headings

            //**********ARM CONTROLS**********
            if (gamepad2.right_stick_y > 0) {
                roboHardware.leftLiftMotor.setPower(liftMotorPower);
                roboHardware.rightLiftMotor.setPower(liftMotorPower);
            }       //RS up
            else if (gamepad2.right_stick_y < 0) {
                roboHardware.leftLiftMotor.setPower(-liftMotorPower);
                roboHardware.rightLiftMotor.setPower(-liftMotorPower);
            }  //RS down
            else {
                roboHardware.leftLiftMotor.setPower(0);
                roboHardware.rightLiftMotor.setPower(0);
            } //stop

            //**********INTAKE CONTROLS**********
            if (gamepad2.dpad_down) {
                roboHardware.spinServo.setDirection(Servo.Direction.REVERSE);
                roboHardware.spinServo.setPosition(0);
            }       //intake
            else if (gamepad2.dpad_up) {
                roboHardware.spinServo.setDirection(Servo.Direction.FORWARD);
                roboHardware.spinServo.setPosition(0);
            }    //output
            else {
                roboHardware.spinServo.setPosition(0.5);
            }

            //**********SLIDE CONTROLS**********
            if (gamepad2.left_stick_y > 0) {
                roboHardware.extendMotor.setPower(0.8);
            }       //Slide out
            else if (gamepad2.left_stick_y < 0) {
                roboHardware.extendMotor.setPower(-0.8);
            }  //Slide in
            else {
                roboHardware.extendMotor.setPower(0);
            }

            ////**********RESET IMU**********
            if (gamepad1.right_stick_button) {
                roboHardware.reInitImu();
            }      //RS button


            //******************************TEST FUNCTIONS******************************

            //AUTO FUNCTIONS
            if (gamepad1.dpad_up) {
                roboHardware.autoMoveSquare(true, 1);
            } else if (gamepad1.dpad_down) {
                roboHardware.autoMoveSquare(false, 1);
            } else if (gamepad1.dpad_left) {
                roboHardware.autoMoveSquareSide(true,1);
            } else if (gamepad1.dpad_right) {
                roboHardware.autoMoveSquareSide(false,1);
            }

            //TEST ARM ENCODER
            /*
            //set mode
            if (gamepad2.left_bumper) {
                roboHardware.setArmtoEncoder();
            } //set Arm to Encoder mode
            else if (gamepad2.right_bumper) {
                roboHardware.setArmtoPower();
            } //set Arm to Power mode
            //movement
            int targetPos = 0;
            if (gamepad2.left_stick_y > 0) {
                targetPos = roboHardware.liftMotor.getCurrentPosition();
                //roboHardware.liftMotor.setTargetPosition(targetPos + 50);
                roboHardware.liftMotor.setTargetPosition(100);
                roboHardware.liftMotor.setPower(0.5);
            } //up
            else if (gamepad2.left_stick_y < 0) {
                targetPos = roboHardware.liftMotor.getCurrentPosition();
                //roboHardware.liftMotor.setTargetPosition(targetPos -50);
                roboHardware.liftMotor.setTargetPosition(-100);
                roboHardware.liftMotor.setPower(0.5);
            } //down
            else {
                roboHardware.liftMotor.setPower(0);
            }

             */


        }
    }
}
