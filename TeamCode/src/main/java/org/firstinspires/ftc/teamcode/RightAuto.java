package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Right Auto", group="Autonomous")
public class RightAuto extends LinearOpMode {
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        sleep(2000);
        //robot faces outward!
        sleep(1000);
        roboHardware.autoMoveSquareSide(true,2);



    } //runOpMode
} //auto