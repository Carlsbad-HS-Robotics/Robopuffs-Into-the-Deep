package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mechanism TeleOp", group="TeleOps")
public class  MechTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        waitForStart();

        double liftMotorPower = 0.35; //multiplied by stick value so max value is 0.5 power (0.5 * 1 = 0.5)

        while (opModeIsActive()) {
            //******************************GAME FUNCTIONS******************************
            roboHardware.fieldCentricDrive(-gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
            /*
            if (fieldFailed = false) {
                roboHardware.fieldCentricDrive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
                telemetry.addData("Drive mode:", "field centric");
            } else if (fieldFailed = true) {
                roboHardware.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
                telemetry.addData("Drive mode:", "robot centric");
            } else {
                roboHardware.fieldCentricDrive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
                telemetry.addData("Drive mode:", "field centric");
            }

            if (gamepad1.right_bumper) {
                fieldFailed = true;
            } else if (gamepad1.left_bumper) {
                fieldFailed = false;
            }

             */ //TODO fix field centric
            roboHardware.getBotHeadings(); //print headings

            //**********ARM CONTROLS**********

            if (gamepad2.right_bumper) {
                roboHardware.leftLiftMotor.setTargetPosition(1000);
                roboHardware.rightLiftMotor.setTargetPosition(1000);
            } else if (gamepad2.left_bumper) {
                roboHardware.leftLiftMotor.setTargetPosition(0);
                roboHardware.rightLiftMotor.setTargetPosition(0);
            }

            roboHardware.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.leftLiftMotor.setPower(0.5);
            roboHardware.rightLiftMotor.setPower(0.5);

            if (gamepad2.x) {
                roboHardware.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                roboHardware.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Reset Encoder");
            } //STOP & RESET ENCODER BUTTON



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
            if (gamepad2.y) {
                roboHardware.extendMotor.setTargetPosition(200);
            } else if (gamepad2.b) {
                roboHardware.extendMotor.setTargetPosition(0);
            }
            telemetry.addData("Current Pos:", roboHardware.extendMotor.getCurrentPosition());

            roboHardware.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.extendMotor.setPower(0.5);

            if (gamepad2.x) {
                roboHardware.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Reset Encoder");
            } //STOP & RESET ENCODER BUTTON



            ////**********RESET IMU**********
            if (gamepad1.right_stick_button) {
                roboHardware.reInitImu();
            }      //RS button


            //******************************TEST FUNCTIONS******************************

            //AUTO FUNCTIONS
            /*
            if (gamepad1.dpad_up) {
                roboHardware.autoMoveSquare(true, 1);
            } else if (gamepad1.dpad_down) {
                roboHardware.autoMoveSquare(false, 1);
            } else if (gamepad1.dpad_left) {
                roboHardware.autoMoveSquareSide(true,1);
            } else if (gamepad1.dpad_right) {
                roboHardware.autoMoveSquareSide(false,1);
            }
             */


            telemetry.update(); //final updates for telemetry; displays all data added throughout the teleop loop

        }
    }
}
