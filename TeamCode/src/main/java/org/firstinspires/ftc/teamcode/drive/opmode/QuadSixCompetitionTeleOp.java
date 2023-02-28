package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Modes TeleOp - V4", group = "0")
public class QuadSixCompetitionTeleOp extends OpMode  {
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime horizontal_time = new ElapsedTime();

    public ElapsedTime ModeTime = new ElapsedTime();

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotorEx ArmHeight;
    DcMotorEx Horizontal1;
    DcMotor Horizontal2;
    DcMotor Turret;
    Servo VClamp;
    Servo HElbow;
    Servo ServoLeftClawH;
    Servo ClawWristV;
    Servo ClawWristH;

    double PowerDilation = 1;
    public static double HElbow_Position = 0;
    public static double H_Wrist_Position = 0.9;
    public static double HClamp_Position = 0;
    double HorizontalPowerMultiplier = 1;
    double HorizontalTarget = 0;

    boolean G2mode;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        {
            LFMotor = hardwareMap.dcMotor.get("LFMotor");
            RFMotor = hardwareMap.dcMotor.get("RFMotor");
            LBMotor = hardwareMap.dcMotor.get("LBMotor");
            RBMotor = hardwareMap.dcMotor.get("RBMotor");
            Horizontal1 = (DcMotorEx) hardwareMap.dcMotor.get("H1");
            Horizontal2 = hardwareMap.dcMotor.get("H2");
            Turret = hardwareMap.dcMotor.get("Turret");
            ArmHeight = hardwareMap.get(DcMotorEx.class, "AH");
            VClamp = hardwareMap.servo.get("VLeft");
            ServoLeftClawH = hardwareMap.servo.get("HClaw");
            ClawWristH = hardwareMap.servo.get("HWrist");
            ClawWristV = hardwareMap.servo.get("VWrist");
            HElbow = hardwareMap.servo.get("HElbow");
            ArmHeight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Horizontal1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Horizontal2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Horizontal1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setDirection(DcMotor.Direction.REVERSE);
            RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            ArmHeight.setDirection(DcMotorSimple.Direction.REVERSE);
            Horizontal1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmHeight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmHeight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        ClawWristV.setPosition(0.85);
        ClawWristH.setPosition(H_Wrist_Position);
        VClamp.setPosition(0);
        ServoLeftClawH.setPosition(HClamp_Position);
        HElbow.setPosition(HElbow_Position);
    }

    @Override
    public void loop() {
        if (!gamepad2.options) {
            ModeTime.reset();
        }

        if (G2mode) {
            telemetry.addLine("Mode 1");
            telemetry.update();
            if (gamepad2.options && ModeTime.milliseconds() > 500) {
                G2mode = !G2mode;
            }
        }

        if (!G2mode){
            telemetry.addLine("Mode 2");
            telemetry.update();
            if (gamepad2.options && ModeTime.milliseconds() > 500) {
                G2mode = !G2mode;
            }
        }

        telemetry.addData("ArmHeight Current Position: ", ArmHeight.getCurrentPosition());
        telemetry.addData("Horizontal1 Current Position", Horizontal1.getCurrentPosition());
        telemetry.addData("Horizontal1 Current Power: ", Horizontal1.getPower());
        telemetry.addData("Horizontal1 Target Position: ", HorizontalTarget);
        telemetry.addData("gamepad2.y current position", gamepad2.left_stick_y);
        telemetry.addData("Current Vertical Time: ", runtime.time());
        telemetry.addData("Current Horizontal Time: ", horizontal_time.time());
        telemetry.addData("Turret Current: ", Turret.getCurrentPosition());
        telemetry.update();

        Horizontal1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Horizontal2.setPower(Horizontal1.getPower());

        //Gamepad1 Base Drive :
        double lateral = gamepad1.left_stick_x * PowerDilation;
        double longitudinal = -gamepad1.left_stick_y * PowerDilation;
        double turn = -0.5 * gamepad1.right_stick_x * PowerDilation;
        double wheelPower = Math.hypot(lateral, longitudinal);
        double stickAngleRadians = Math.atan2(longitudinal, lateral);
        stickAngleRadians = stickAngleRadians - Math.PI / 4;
        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
        double LFPower = (wheelPower * cosAngleRadians * factor + turn);
        LFMotor.setPower(LFPower);
        double RFPower = (wheelPower * sinAngleRadians * factor - turn);
        RFMotor.setPower(RFPower);
        double LBPower = (wheelPower * sinAngleRadians * factor + turn);
        LBMotor.setPower(LBPower);
        double RBPower = (wheelPower * cosAngleRadians * factor - turn);
        RBMotor.setPower(RBPower);



        //****************************************************************************



        if (gamepad1.left_bumper) {
            PowerDilation = 0.5;
        } else {
            PowerDilation = 0.8;
        }

        //Works
        if (gamepad1.x) {
            ServoLeftClawH.setPosition(0);
            VClamp.setPosition(0);
            ClawWristV.setPosition(0);
            ArmHeight.setTargetPosition(0);
            ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (ArmHeight.getCurrentPosition() < (ArmHeight.getTargetPosition() - 10)) {
                ArmHeight.setPower(1);
            } else if (ArmHeight.getCurrentPosition() > (ArmHeight.getTargetPosition() + 5)) {
                ArmHeight.setPower(-1);
            } else {
                ArmHeight.setPower(0);
            }
        }

        //Works
        if (gamepad1.y) {
            ServoLeftClawH.setPosition(0);
            VClamp.setPosition(0);
            ClawWristV.setPosition(0);
            ArmHeight.setTargetPosition(700);
            ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (ArmHeight.getCurrentPosition() < (ArmHeight.getTargetPosition() - 10)) {
                ArmHeight.setPower(1);
            } else if (ArmHeight.getCurrentPosition() > (ArmHeight.getTargetPosition() + 10)) {
                ArmHeight.setPower(-1);
            } else {
                ArmHeight.setPower(0);
            }
        }

        //Works
        if (gamepad1.b) {
            ServoLeftClawH.setPosition(0);
            VClamp.setPosition(0);
            ClawWristV.setPosition(0);
            ArmHeight.setTargetPosition(1550);
            ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (ArmHeight.getCurrentPosition() < (ArmHeight.getTargetPosition() - 5)) {
                ArmHeight.setPower(1);
            } else if (ArmHeight.getCurrentPosition() > (ArmHeight.getTargetPosition() + 5)) {
                ArmHeight.setPower(-1);
            } else {
                ArmHeight.setPower(0);
            }
        }

        //Works
        if (gamepad1.atRest()) {
            runtime.reset();
        }

        //Works
        if (gamepad1.right_trigger > 0.5) {
            VClamp.setPosition(1);
            if (runtime.milliseconds() > 400) {
                VClamp.setPosition(1);
                telemetry.addLine("Runtime has been Reset");
                telemetry.update();
                ClawWristV.setPosition(0.85);
                ArmHeight.setTargetPosition(0);
                ArmHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ArmHeight.getCurrentPosition() > (ArmHeight.getTargetPosition() + 5)) {
                    VClamp.setPosition(1);
                    ArmHeight.setPower(-0.8);
                }
            }
        }

        if (gamepad1.dpad_down && gamepad1.left_trigger > 0.1) {
            ArmHeight.setPower(-0.5);
        }






        //Works
        if (Horizontal1.getCurrentPosition() < 20 && !gamepad2.dpad_down) {
            Horizontal1.setPower(Math.max(HorizontalPowerMultiplier * gamepad2.left_stick_y, 0));
        } else if (gamepad2.dpad_down && gamepad2.left_trigger > 0.1) {
            Horizontal1.setPower(HorizontalPowerMultiplier * gamepad2.left_stick_y);
        } else {
            Horizontal1.setPower(HorizontalPowerMultiplier * gamepad2.left_stick_y);
        }

        //Works
        if (gamepad2.right_bumper) {
            HorizontalPowerMultiplier = 1;
        } else {
            HorizontalPowerMultiplier = 0.6;
        }

        //Works
        if (!gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right && gamepad2.atRest()) {
            horizontal_time.reset();
        }

        //Works
        if (gamepad2.b) {
            ServoLeftClawH.setPosition(0.65);
            if (horizontal_time.milliseconds() > 400) {
                ClawWristH.setPosition(1);
                HElbow.setPosition(0.9);
            }
        }
        //Works
        if (gamepad2.a) {
            if (horizontal_time.milliseconds() < 400) {
                ServoLeftClawH.setPosition(0);
            }
            if (horizontal_time.milliseconds() > 400) {
                ClawWristH.setPosition(1);
                HElbow.setPosition(0.25);
                ServoLeftClawH.setPosition(0.47);
            }
        }

        if (gamepad2.right_trigger > 0.1) {
            HElbow.setPosition(0.4);
            if (horizontal_time.milliseconds() > 600) {
                ServoLeftClawH.setPosition(0);
            }
        }

        Turret.setPower(gamepad2.right_stick_x);

    }

    @Override
    public void stop() {
    }
}