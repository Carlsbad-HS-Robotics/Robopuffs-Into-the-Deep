package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


public class MechTeleop extends LinearOpMode {

    private DcMotor liftMotor;

    @Override
    public void runOpMode() {

        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        //roboHardware.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {

            //RobotHardware.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);


            if (gamepad1.right_bumper == true) {
                liftMotor.setPower(1);
            }
            else if (gamepad1.left_bumper == true) {
                liftMotor.setPower(-1);
            }
            else {
                liftMotor.setPower(0);
            }
        }
    }
}
