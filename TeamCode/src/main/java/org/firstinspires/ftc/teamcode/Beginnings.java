package org.firstinspires.ftc.teamcode;

// imports

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp
public class Beginnings extends LinearOpMode {
    // Declare vars

    private Servo launcher;
    private Servo rightLift;
    private Servo leftLift;
    private Servo shoulder;
    private Servo wrist;
    private Servo hopper;
    MecanumDrive drive;
    OgDrive ogDrive;
    DcMotor frontIntake;
    DcMotor rearIntake;


    // Servo prep

    double MinLiftHeight = 0.05;
    double MaxLiftHeight = 0.65;
    double LiftLeftOffset = -0.08;
    double LiftHeight;
    double LiftOffset = 0;
    private AndroidTextToSpeech androidTextToSpeech;



    // Functions \/


    private void servo_shenanigans() {
       setLiftHeight(0.4);
       sleep(10000);
       setLiftHeight(0.05);
    }

    private void setLiftHeight(double inputLiftHeight) {
        if (inputLiftHeight < 0.05){
            inputLiftHeight = 0.05;
        }
        if (inputLiftHeight > 0.65){
            inputLiftHeight = 0.65;
        }
        LiftHeight = inputLiftHeight;
        leftLift.setPosition(LiftLeftOffset + LiftHeight);
        rightLift.setPosition(LiftHeight);

    }

    private void worm() {
        if(gamepad2.left_bumper) {
            shoulder.setPosition(0.445);
            wrist.setPosition(0.26);
        }
        if(gamepad2.right_bumper) {
            shoulder.setPosition(0.92);
        }
    }

    private void airplane() {
        if (gamepad2.dpad_down) {
            launcher.setPosition(0.1);
        }
        else {
            launcher.setPosition(0.8);
        }
    }
    private void tuneshoulder() {
        if (gamepad2.right_bumper) {
            shoulder.setPosition(shoulder.getPosition() + .01);
        } else if (gamepad2.right_trigger > .5){
            shoulder.setPosition(shoulder.getPosition() - .01);
        } else {
            //shoulder.setPosition(0.1);
        }
        telemetry.addData("shoulder", shoulder.getPosition());
    }
    /*
    private void tuneWrist() {
        if (gamepad2.left_bumper) {
            wrist.setPosition(wrist.getPosition() + .01);
        } else if (gamepad2.left_trigger > .5){
            wrist.setPosition(wrist.getPosition() - .01);
        } else {
            //shoulder.setPosition(0.1);
        }
        telemetry.addData("wrist", wrist.getPosition());
    }

    private void jukeBeta() {
        if (gamepad2.dpad_up) {
            shoulder.setPosition(.48);
            wrist.setPosition(.31);
        } else if (gamepad2.dpad_down) {
            shoulder.setPosition(.445);
            wrist.setPosition(.26);
        }
    }
     */
    private void liftFunction() {
        if (gamepad2.y) {
            leftLift.setPosition(0.9 + LiftOffset);
            rightLift.setPosition(0.9);
        }
        else if (gamepad2.x) {
            leftLift.setPosition(0.8 + LiftOffset);
            rightLift.setPosition(0.8);
        }
        else if (gamepad2.b) {
            leftLift.setPosition(0.7 + LiftOffset);
            rightLift.setPosition(0.7);
        }
        else if (gamepad2.a) {
            leftLift.setPosition(0.5 + LiftOffset);
            rightLift.setPosition(0.5);
        }
    }
    private void intakeFunction() {
        if (gamepad1.right_trigger > 0.5) {
            frontIntake.setPower(1);
            rearIntake.setPower(1);
            telemetry.addData("Intake", "in");
        }
        else if (gamepad1.right_bumper) {
            frontIntake.setPower(-1);
            rearIntake.setPower(-1);
            telemetry.addData("Intake", "out");
        }
        else {
            telemetry.addData("Intake", "stopped");
            frontIntake.setPower(0);
            rearIntake.setPower(0);
        }

    }


    private void driveCode() {

        double SLOW_DOWN_FACTOR;
        SLOW_DOWN_FACTOR = 0.5;
        telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","TEAM NUMBER");
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                        -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                ),
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
        ));

        drive.updatePoseEstimate();

        //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
        //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
        //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
        //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

        telemetry.addLine("Current Pose");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
       // telemetry.update();

    }



    @Override
    // NOT loop \/ - Or int of vars
    public void runOpMode() throws InterruptedException {

        androidTextToSpeech = new AndroidTextToSpeech();
        androidTextToSpeech.initialize();

        // for wires driving
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // for Og driving (the best driving)
        ogDrive = new OgDrive(hardwareMap);

        launcher = hardwareMap.get(Servo.class, "launcher");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");
        shoulder = hardwareMap.get(Servo.class, "shoulder");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);


        launcher.setDirection(Servo.Direction.REVERSE);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // servos
        launcher.setPosition(0.8);
        leftLift.setPosition(0.42);
        rightLift.setPosition(0.42);
        shoulder.setPosition(0.445);
        wrist.setPosition(0.26);
        hopper.setPosition(0.0);
        /*
        sleep(500);
        leftLift.setPosition(0.8);
        rightLift.setPosition(0.8);
        wrist.setPosition(0.7);
        */


        double SLOW_DOWN_FACTOR = 0.5;
        telemetry.addData("Initializing TeleOp","");
        telemetry.update();


        waitForStart();
       // servo_shenanigans();
        // loop real
        while(opModeIsActive()){
            //driveCode();
            airplane();
            ogDrive.og_drive_code(gamepad1, telemetry);
            intakeFunction();
            liftFunction();
            worm();
            telemetry.update();
            sleep(100);
        }
    }
}
