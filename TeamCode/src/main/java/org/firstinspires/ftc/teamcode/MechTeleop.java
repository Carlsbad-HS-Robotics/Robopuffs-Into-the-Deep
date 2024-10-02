package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mechanism TeleOp", group="TeleOps")
public class MechTeleop extends LinearOpMode {

    private DcMotor liftMotor;

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        waitForStart();

        roboHardware.frontLeftMotor.setPower(0);
        roboHardware.frontRightMotor.setPower(0);
        roboHardware.backLeftMotor.setPower(0);
        roboHardware.backRightMotor.setPower(0);

        while (opModeIsActive()) {
            roboHardware.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            //lift motor controls
            if (gamepad2.right_stick_y > 0) {
                roboHardware.liftMotor.setPower(0.5);
            }
            else if (gamepad2.right_stick_y < 0) {
                roboHardware.liftMotor.setPower(-0.5);
            }
            else {
                roboHardware.liftMotor.setPower(0);
            }

            //Servo controls
            if (gamepad2.left_bumper) {
                roboHardware.spinServo.setPosition(0.5);
            }
            else if (gamepad2.right_bumper) {
                roboHardware.spinServo.setPosition(0);
            }

            //Set servo direction
            if (gamepad2.dpad_down) {
                roboHardware.spinServo.setDirection(Servo.Direction.REVERSE); // intake
            }
            else if (gamepad2.dpad_up) {
                roboHardware.spinServo.setDirection(Servo.Direction.FORWARD); // outtake
            }
        }
    }
}
