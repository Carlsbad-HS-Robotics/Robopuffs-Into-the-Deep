package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Left Level 1 Ascent", group="Autonomous")
public class Level1Ascent extends LinearOpMode{
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        telemetry.addData("Autonomous: ", "Initialized");
        telemetry.update();

        waitForStart();

        //robot faces outward!
        //********************SCORE SAMPLE********************
        telemetry.addLine("Autonomous running...");
        telemetry.addLine();
        telemetry.addData("Sample Scoring: ", "Incomplete");
        telemetry.addData("Parking:        ", "Incomplete");
        telemetry.update();

        //TODO score in net zone or low basket?
        roboHardware.autoMoveSquare(true,0.5);                          //go forward 0.1 mat
        sleep(300);
        roboHardware.leftLiftMotor.setPower(0.3); //TODO is this the right direction
        sleep(1000); //TODO see if this needs to be longer or shorter
        roboHardware.leftLiftMotor.setPower(0);                                            //raise arm (with preloaded sample)
        roboHardware.autoMoveSquareSide(true,1.5);                        //go left 1.5 mats
        roboHardware.autoOdoTurn(true,45);                                 //turn left 45 degrees
        roboHardware.autoMoveSquareSide(true,0.3);                        //move left 0.5 mats
        //TODO check and test number of mats

        telemetry.addLine("Autonomous running...");
        telemetry.addLine();
        telemetry.addData("Sample Scoring: ", "Complete");
        telemetry.addData("Parking:        ", "Incomplete");
        telemetry.update();


        //********************PARK********************
        roboHardware.autoMoveSquareSide(false,0.3);                         //move right 0.5 mats
        roboHardware.autoOdoTurn(false,45);                                 //turn right 45 degrees
        roboHardware.autoMoveSquare(false,0.5);                         //go backwards 0.5 mats (against barrier)
        roboHardware.autoMoveSquareSide(false,4.5);                        //go left 4.5 mats (parked)
        roboHardware.stopDrive();

        telemetry.addLine("Autonomous running...");
        telemetry.addLine();
        telemetry.addData("Sample Scoring: ", "Complete");
        telemetry.addData("Parking:        ", "Complete");
        telemetry.update();



    } //runOpMode
}
