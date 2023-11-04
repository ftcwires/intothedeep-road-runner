package org.firstinspires.ftc.teamcode;

// imports

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor frontIntake;
    DcMotor rearIntake;


    // Servo prep

    double MinLiftHeight = 0.05;
    double MaxLiftHeight = 0.65;
    double LiftLeftOffset = -0.08;
    double LiftHeight;
    private AndroidTextToSpeech androidTextToSpeech;



    // Functions \/

    private void og_drive_code() {

        double Scale_Factor_of_Drive;

        if (gamepad1.right_bumper) {
            Scale_Factor_of_Drive = 1;
        } else {
            Scale_Factor_of_Drive = 0.55;
        }
        // drive with joysticks
        BackLeft.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - 0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x + gamepad1.left_stick_y));
        BackRight.setPower(0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x - gamepad1.left_stick_y));
        FrontLeft.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x - gamepad1.left_stick_y));
        FrontRight.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x + gamepad1.left_stick_y));
        telemetry.addData("FL Motor", FrontLeft.getPower());
        telemetry.addData("FR Motor", FrontRight.getPower());
        telemetry.addData("BL Motor", BackLeft.getPower());
        telemetry.addData("BR Motor", BackRight.getPower());

    }


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

        launcher = hardwareMap.get(Servo.class, "launcher");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* all for OG driving
        BackLeft = hardwareMap.get(DcMotor.class, "leftRear");
        BackRight = hardwareMap.get(DcMotor.class, "rightRear");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        LiftRight = hardwareMap.get(Servo.class, "LiftRight");
        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");


         LiftRight.setDirection(Servo.Direction.REVERSE); only for og drive code


        // init servo hardware
        // init drive hardware and variables
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
         */

        // servos
        launcher.setPosition(0.8);
        leftLift.setPosition(0);
        rightLift.setPosition(0);
        shoulder.setPosition(0.1);
        wrist.setPosition(-0.04);
        hopper.setPosition(0.01);


        double SLOW_DOWN_FACTOR = 0.5;
        telemetry.addData("Initializing TeleOp","");
        telemetry.update();


        waitForStart();
       // servo_shenanigans();
        // loop real
        while(opModeIsActive()){
            driveCode();
            intakeFunction();
            telemetry.update();
            sleep(100);
        }
    }
}
