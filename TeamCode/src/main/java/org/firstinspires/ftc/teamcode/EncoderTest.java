package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Encoders", group="TeleOps")
public class EncoderTest extends LinearOpMode {

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

            roboHardware.presetSlideLift(gamepad2.y, gamepad2.x, gamepad2.a);
            telemetry.addData("Current Encoder Position:", roboHardware.extendMotor.getCurrentPosition());



            if (gamepad2.left_stick_button) {
                roboHardware.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Reset Encoder");
            } //STOP & RESET ENCODER BUTTON


            telemetry.update(); //final updates for telemetry; displays all data added throughout the teleop loop

        }
    }
}
