package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="Encoder Testing", group="TeleOp");

public class MechTeleop extends LinearOpMode{

    private DcMotor liftMotor;

    @Override
    public void runOpMode() {

        telemetry.addData("Status:", "Initialized!");
        telemetry.update();

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        //RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        //roboHardware.initialize();

        waitForStart();

        //roboHardware.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
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
