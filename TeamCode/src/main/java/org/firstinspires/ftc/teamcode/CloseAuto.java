package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Right Auto", group="Autonomous")
public class CloseAuto extends LinearOpMode {
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        roboHardware.autoMoveSquareSide(true,2.2);



    } //runOpMode
} //auto