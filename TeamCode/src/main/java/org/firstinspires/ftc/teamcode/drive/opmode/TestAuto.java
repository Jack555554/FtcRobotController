package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {
    public DcMotorEx AH;
    public DcMotorEx H1;
    public DcMotorEx H2;
    public Servo VWrist;
    public Servo HWrist;
    public Servo HClaw;
    public Servo HElbow;
    public Servo VLeft;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AH = hardwareMap.get(DcMotorEx.class,"AH");
        H1 = hardwareMap.get(DcMotorEx.class,"H1");
        H2 = hardwareMap.get(DcMotorEx.class,"H2");

        VWrist = hardwareMap.servo.get("VWrist");
        HWrist = hardwareMap.servo.get("HWrist");
        HClaw = hardwareMap.servo.get("HClaw");
        HElbow = hardwareMap.servo.get("HElbow");
        VLeft = hardwareMap.servo.get("VLeft");

        AH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        H1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        H2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        H1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        Trajectory firstPos = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(44,-5.5,Math.toRadians(-77)),Math.toRadians(0))
                .build();

        Trajectory secondPos = drive.trajectoryBuilder(firstPos.end())
                .back(2)
                .build();






        waitForStart();
        HElbow.setPosition(.65);
        sleep(500);
        HWrist.setPosition(.6);
        HClaw.setPosition(0);
        sleep(300);
        VLeft.setPosition(0);
        sleep(100);
        extendVertical(1,-800);
        VWrist.setPosition(0);
        sleep(200);
        extendHorizontal(1,-650);
        extendHorizontal(1,580);
        sleep(1000);
        VLeft.setPosition(1);
        sleep(300);
        VWrist.setPosition(.84);
        extendVertical(1,0);
        HClaw.setPosition(1);
        sleep(300);
        HWrist.setPosition(.6);
        sleep(100);
        HElbow.setPosition(.8);
        sleep(300);
        extendHorizontal(1,100);
        sleep(400);
        HElbow.setPosition(1);
        sleep(400);
        HClaw.setPosition(0);
        HElbow.setPosition(.6);
        sleep(500);
        HWrist.setPosition(.65);
        HClaw.setPosition(0);
        sleep(300);
        VLeft.setPosition(0);
        sleep(100);
        VWrist.setPosition(0);
        sleep(200);
        extendVertical(1,-800);
        extendHorizontal(1,580);
        sleep(500);
        VLeft.setPosition(1);
        sleep(300);
        VWrist.setPosition(.84);
        extendVertical(1,0);
        HClaw.setPosition(1);
        sleep(300);
        HWrist.setPosition(.6);
        sleep(100);
        HElbow.setPosition(.8);
        sleep(300);
        extendHorizontal(1,100);
        sleep(400);
        HElbow.setPosition(1);
        sleep(400);
        HClaw.setPosition(0);
        HElbow.setPosition(.55);
        sleep(500);
        HWrist.setPosition(.7);
        HClaw.setPosition(0);
        sleep(300);
        VLeft.setPosition(0);
        sleep(100);
        VWrist.setPosition(0);
        sleep(200);
        extendVertical(1,-800);
        extendHorizontal(1,570);
        sleep(500);
        VLeft.setPosition(1);
        sleep(300);
        VWrist.setPosition(.84);
        extendVertical(1,0);
        HClaw.setPosition(1);
        sleep(300);
        HWrist.setPosition(.6);
        sleep(100);
        HElbow.setPosition(.8);
        sleep(300);
        extendHorizontal(1,100);
        sleep(400);
        HElbow.setPosition(1);
        sleep(400);
        HClaw.setPosition(0);
        HElbow.setPosition(.5);
        sleep(500);
        HWrist.setPosition(.75);
        HClaw.setPosition(0);
        sleep(300);
        VLeft.setPosition(0);
        sleep(100);
        VWrist.setPosition(0);
        sleep(200);
        extendVertical(1,-800);
        extendHorizontal(1,570);
        sleep(500);
        VLeft.setPosition(1);
        sleep(300);
        VWrist.setPosition(.84);
        extendVertical(1,0);
        HClaw.setPosition(1);
        sleep(300);
        HWrist.setPosition(.6);
        sleep(100);
        HElbow.setPosition(.8);
        sleep(300);
        extendHorizontal(1,100);
        sleep(400);
        HElbow.setPosition(1);
        sleep(400);
        HClaw.setPosition(0);

















    }

    public void extendVertical(double verticalSpeed, int verticalTarget) {
        AH.setTargetPosition(verticalTarget);
        AH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (AH.isBusy()) {
            AH.setPower(verticalSpeed);
        } else {
            AH.setPower(0);
        }
    }

    public void retractVertical(double verticalSpeed, int verticalTarget) {
        AH.setTargetPosition(verticalTarget);
        AH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (AH.isBusy()) {
            AH.setPower(-verticalSpeed);
        } else {
            AH.setPower(0);
        }
    }

    public void extendHorizontal(double speed, int target) {
        H1.setTargetPosition(target);
        H1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (H1.getCurrentPosition() < H1.getTargetPosition()) {
            H1.setPower(speed);
            H2.setPower(speed);
        }
    }

    public void retractHorizontal(double speed, int target) {
        H1.setTargetPosition(target);
        H1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (H1.getCurrentPosition() > H1.getTargetPosition()) {
            H1.setPower(speed);
            H2.setPower(speed);
        }
    }

}
