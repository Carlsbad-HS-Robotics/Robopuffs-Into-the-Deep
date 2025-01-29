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
            //roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
            //TODO fix field centric
            roboHardware.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
            roboHardware.getBotHeadings(); //print headings

            //**********ARM CONTROLS**********
            //roboHardware.presetArmLifting(gamepad2.right_bumper, gamepad2.left_bumper); //Arm controls

            int increment = 20;
            if (gamepad2.right_stick_y < 0) {
                roboHardware.leftLiftMotor.setTargetPosition(roboHardware.leftLiftMotor.getCurrentPosition() + increment);
                roboHardware.rightLiftMotor.setTargetPosition(roboHardware.rightLiftMotor.getCurrentPosition() + increment);
            } else if (gamepad2.right_stick_y > 0) {
                roboHardware.leftLiftMotor.setTargetPosition(roboHardware.leftLiftMotor.getCurrentPosition() - increment);
                roboHardware.rightLiftMotor.setTargetPosition(roboHardware.rightLiftMotor.getCurrentPosition() - increment);
            } else {
                roboHardware.leftLiftMotor.setTargetPosition(roboHardware.leftLiftMotor.getCurrentPosition());
                roboHardware.rightLiftMotor.setTargetPosition(roboHardware.rightLiftMotor.getCurrentPosition());
            }

            roboHardware.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.leftLiftMotor.setPower(0.5);
            roboHardware.rightLiftMotor.setPower(0.5);

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
            double slideMultiplier = -0.3;
            telemetry.addData("Extension Motor Ticks: " , roboHardware.extendMotor.getCurrentPosition());
            roboHardware.extendMotor.setPower(gamepad2.left_stick_y * slideMultiplier); //Slide in / out

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
