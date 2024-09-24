package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    public LinearOpMode teleOp;

    //TODO what is IMU again??
    public IMU imu;
    public HardwareMap hardwareMap;

    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    } //RobotHardware functions

    public DcMotor liftMotor;


    //FUNCTIONS

    public void initialize() {

        //liftMotor intitialization
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


} //class RobotHardware