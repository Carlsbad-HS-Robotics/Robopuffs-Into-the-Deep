package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Odometry Test", group="TeleOps")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();
        waitForStart();
        //double liftMotorPower = 0.3;
        double liftMotorPower = 0.35;

        roboHardware.extendMotor.setTargetPosition(roboHardware.extendMotor.getCurrentPosition());

        while (opModeIsActive()) {

            //******************************GAME FUNCTIONS******************************
            roboHardware.robotCentricDrive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);


            telemetry.update(); //final updates for telemetry; displays all data added throughout the teleop loop

        }
    }
}
