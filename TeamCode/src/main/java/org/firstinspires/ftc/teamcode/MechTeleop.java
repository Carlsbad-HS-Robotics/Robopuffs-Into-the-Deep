package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

@TeleOp(name="Mechanism TeleOp", group="TeleOps")
public class MechTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        waitForStart();

        roboHardware.leftExtendMotor.setTargetPosition(roboHardware.leftExtendMotor.getCurrentPosition());
        roboHardware.rightExtendMotor.setTargetPosition(roboHardware.rightExtendMotor.getCurrentPosition());

        while (opModeIsActive()) {
            //******************************GAME FUNCTIONS******************************

            //**********DRIVE**********
            //roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x); //drive
            roboHardware.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            //**********SLIDES**********
            int increment = 20; //increment by which the motor turns; how many ticks
            if (gamepad2.right_stick_y < 0) {
                roboHardware.leftExtendMotor.setTargetPosition(roboHardware.leftExtendMotor.getCurrentPosition() + increment);
                roboHardware.rightExtendMotor.setTargetPosition(roboHardware.rightExtendMotor.getCurrentPosition() + increment);
            }
            else if (gamepad2.right_stick_y > 0) {
                roboHardware.leftExtendMotor.setTargetPosition(roboHardware.leftExtendMotor.getCurrentPosition() - increment);
                roboHardware.rightExtendMotor.setTargetPosition(roboHardware.rightExtendMotor.getCurrentPosition() - increment);
            }
            else {
                roboHardware.leftExtendMotor.setTargetPosition(roboHardware.leftExtendMotor.getCurrentPosition());
                roboHardware.rightExtendMotor.setTargetPosition(roboHardware.rightExtendMotor.getCurrentPosition());
            }
            roboHardware.leftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.rightExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.leftExtendMotor.setPower(0.5);
            roboHardware.rightExtendMotor.setPower(0.5);

            //roboHardware.presetSlideLift(gamepad2.y, gamepad2.x, gamepad2.a);       // High, Low, Bottom
            telemetry.addData("Current Slides Position:", roboHardware.leftExtendMotor.getCurrentPosition());
            telemetry.addLine();

            //**********INTAKE**********
            if (gamepad2.dpad_down) {
                roboHardware.spinServo.setDirection(Servo.Direction.REVERSE);
                roboHardware.spinServo.setPosition(0);
            }            //intake
            else if (gamepad2.dpad_up) {
                roboHardware.spinServo.setDirection(Servo.Direction.FORWARD);
                roboHardware.spinServo.setPosition(0);
            }         //output
            else {
                roboHardware.spinServo.setPosition(0.5);
            }

            //**********RESET IMU**********
            if (gamepad1.right_stick_button) {
                roboHardware.reInitImu();
            }   //RS button

            //**********ODOMETRY VALUES**********
            roboHardware.odo.update(); //gets new data from odometry pods
            telemetry.addLine("POSITIONS IN MM");
            telemetry.addLine();
            telemetry.addData("Pose2D position: ", roboHardware.odo.getPosition());
            telemetry.addData("X position: ", roboHardware.odo.getPosX()); //X position in mm relative to starting pos
            telemetry.addData("Y position: ", roboHardware.odo.getPosY()); //Y position in mm relative to starting pos
            telemetry.addData("Heading: ", roboHardware.odo.getHeading());
            telemetry.addLine();
            /*
            telemetry.addLine("POSITIONS IN INCHES");
            telemetry.addLine();
            telemetry.addData("Pose2D position: ", roboHardware.odo.getPosition());
            telemetry.addData("X position: ", roboHardware.odo.getPosX()); //X position in mm relative to starting pos
            telemetry.addData("Y position: ", roboHardware.odo.getPosY()); //Y position in mm relative to starting pos
            telemetry.addData("Heading: ", roboHardware.odo.getHeading());
            telemetry.addLine();
             */

            //******************************TEST FUNCTIONS******************************

            //POWER SLIDE CONTROLS
            /*
            //**********POWER SLIDE CONTROLS**********
            double slideMultiplier = 0.4;
            telemetry.addData("Extension Motor Ticks: " , roboHardware.extendMotor.getCurrentPosition());
            roboHardware.extendMotor.setPower(gamepad2.left_stick_y * slideMultiplier); //Slide in & out
             */

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
