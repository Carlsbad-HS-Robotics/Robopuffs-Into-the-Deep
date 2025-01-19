package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Reset Components", group="TeleOps")
public class resetting extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();
        waitForStart();
        //double liftMotorPower = 0.3;
        double liftMotorPower = 0.35;

        while (opModeIsActive()) {

            //******************************GAME FUNCTIONS******************************
            roboHardware.fieldCentricDrive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
            roboHardware.powerArmLifting(gamepad2.right_stick_y);

        }
    }
}
