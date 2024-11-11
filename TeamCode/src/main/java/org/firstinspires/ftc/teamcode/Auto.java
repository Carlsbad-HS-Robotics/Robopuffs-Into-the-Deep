package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Test Auto", group="Autonomous")
public class Auto extends LinearOpMode {
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        /*
        1. Move left
        2. Stop
        3. Drop Specimen into backstage
        4. Move left
        5. Stop (park)
         */

        /*
        roboHardware.autoMoveSquareSide(true,1.5);
        //TODO add only if we need to get rid of the specimen before parking for points
        roboHardware.stopDrive();
        roboHardware.spinServo.setDirection(Servo.Direction.FORWARD);
        roboHardware.spinServo.setPosition(0);
        sleep(1000);
        roboHardware.stopAll();
        roboHardware.autoMoveSquareSide(true,0.5);
        roboHardware.stopDrive();

         */

        roboHardware.autoMoveSquareSide(true,2);



    } //runOpMode
} //auto