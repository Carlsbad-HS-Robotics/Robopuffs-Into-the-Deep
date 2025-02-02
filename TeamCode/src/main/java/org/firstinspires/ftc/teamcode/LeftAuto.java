package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Left Auto", group="Autonomous")
public class LeftAuto extends LinearOpMode {
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        //put on the left edge of the left square (align with submersible cage left bottom corner)
        sleep(2000);
        //robot faces outward!
        sleep(1000);
        roboHardware.autoMoveSquareSide(true,3.5);



    } //runOpMode
} //auto