package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous (name="Test Auto", group="Autonomous")
public class Auto extends LinearOpMode {
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();

        waitForStart();

        while (opModeIsActive()) {
            roboHardware.getBotHeadings();
        }

    } //runOpMode
} //auto