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
            roboHardware.fieldCentricDrive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);

            if (gamepad2.y) {
                roboHardware.extendMotor.setTargetPosition(100);
            } else if (gamepad2.b) {
                roboHardware.extendMotor.setTargetPosition(0);
            }

            roboHardware.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roboHardware.extendMotor.setPower(0.5);

            if (gamepad2.x) {
                roboHardware.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Reset Encoder");
            } //STOP & RESET ENCODER BUTTON



            telemetry.update(); //final updates for telemetry; displays all data added throughout the teleop loop

        }
    }
}
