package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

/*TODO
Test odometry / heading sensors
Figure out math for odometry
get turn figured out
 */

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

        double liftMotorPower = 0.3;

        while (opModeIsActive()) {

            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
            roboHardware.getBotHeadings();

            //lift motor controls
            if (gamepad2.right_stick_y > 0) {
                roboHardware.liftMotor.setPower(liftMotorPower);
            } //up
            else if (gamepad2.right_stick_y < 0) {
                roboHardware.liftMotor.setPower(-liftMotorPower);
            } //down
            else {
                roboHardware.liftMotor.setPower(0);
            } //stop


            //Auto turns
            if (gamepad1.x) {
                roboHardware.autoOdoTurn(true);
            } //left
            else if (gamepad1.b && !gamepad1.start && !gamepad2.start) {
                //make sure to exclude gamepad start because when setting controllers A & B it might accidentally run
                roboHardware.autoOdoTurn(false);
            } //right
            else if (gamepad1.y) {
                roboHardware.autoMoveSquare(true, 1);
            } //forward square
            else if (gamepad1.a && !gamepad1.start && !gamepad2.start) {
                roboHardware.autoMoveSquare(false, 1);
            }


            //Wheel/Servo controls
            if (gamepad2.dpad_down) {
                roboHardware.spinServo.setDirection(Servo.Direction.REVERSE);
                roboHardware.spinServo.setPosition(0);
            } //intake
            else if (gamepad2.dpad_up) {
                roboHardware.spinServo.setDirection(Servo.Direction.FORWARD);
                roboHardware.spinServo.setPosition(0);
            } //export
            else {
                roboHardware.spinServo.setPosition(0.5);
            }

        }
    }
}
