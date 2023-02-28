package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


    @Config
    @Autonomous
    public class WristReset extends LinearOpMode {
        public Servo HWrist;



        @Override
        public void runOpMode() {

            HWrist = hardwareMap.servo.get("HWrist");

            waitForStart();
            HWrist.setPosition(1);
            sleep(3000);



        }
    }