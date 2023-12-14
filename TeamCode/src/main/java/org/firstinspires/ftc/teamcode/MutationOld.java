package org.firstinspires.ftc.teamcode;

// imports

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

//@TeleOp
public class MutationOld extends LinearOpMode {
    // Declare vars
    private RevTouchSensor rightUpper;
    private RevTouchSensor leftUpper;
    private RevTouchSensor rightLower;
    private RevTouchSensor leftLower;
    private Servo launcher;
    private Servo rightLift;
    private Servo leftLift;
    private Servo shoulder;
    private Servo wrist;
    private Servo elbow;
    private Servo leftFinger;
    private Servo rightFinger;

    //MecanumDrive drive;
    //OgDrive ogDrive;
    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1
    private DcMotor leftBack; //rear left 3
    private DcMotor leftHang;
    private DcMotor rightHang;


    // Servo prep
    private static enum ArmPosition {
        INTAKE,
        DRIVE,
        UPGO1,
        UPGO2,
        UPGO3,
        UPGO4,
        UPGO5;
    }

    private ArmPosition currentArmPos;
    double LiftLeftOffset = .04;
    double LiftHeight;
    private AndroidTextToSpeech androidTextToSpeech;


    // Functions \/


    private void servo_shenanigans() {
        setLiftHeight(0.4);
        sleep(10000);
        setLiftHeight(0.05);
    }

    private void setLiftHeight(double inputLiftHeight) {
        if (inputLiftHeight < 0.42) {
            inputLiftHeight = 0.42;
        }
        if (inputLiftHeight > 1) {
            inputLiftHeight = 1;
        }
        LiftHeight = inputLiftHeight;
        leftLift.setPosition(LiftLeftOffset + LiftHeight);
        rightLift.setPosition(LiftHeight);

    }

    private void worm() {
        if (gamepad2.left_bumper) {
            shoulder.setPosition(0.445);
            wrist.setPosition(0.26);
        }
        if (gamepad2.right_bumper) {
            shoulder.setPosition(0.92);
        }
    }

    private void airplane() {
        if (gamepad2.back) {
            launcher.setPosition(0.1);
        } else {
            launcher.setPosition(0.8);
        }
    }

    private void liftFunction() {
        if (gamepad2.y) {
            shoulder.setPosition(0.79);
            setLiftHeight(0.42);

            wrist.setPosition(0.2);
            sleep(600);
            shoulder.setPosition(0.60);
            sleep(600);
            shoulder.setPosition(0.50);
            sleep(600);
            intakePos();
        } else if (gamepad2.x) {
            setLiftHeight(0.71);
        } else if (gamepad2.b) {
            setLiftHeight(0.56);
        }
    }
    private void armmDown() {
        if (gamepad2.y) {
            switch (currentArmPos) {
                case UPGO1:
                    currentArmPos = ArmPosition.INTAKE;
                    break;
                case UPGO2:
                    currentArmPos = ArmPosition.UPGO1;
                    break;
                case UPGO3:
                    currentArmPos = ArmPosition.UPGO2;
                    break;
                case UPGO4:
                    currentArmPos = ArmPosition.UPGO3;
                    break;
                case UPGO5:
                    currentArmPos = ArmPosition.UPGO4;
                    break;
            }
            doArmm();
            telemetry.addLine();
            telemetry.addData("armpostion", currentArmPos.toString());
            sleep(100);
        }
    }
    private void doArmm() {
        switch (currentArmPos) {
            case INTAKE:
                intakePos();
                break;
            case UPGO1:
                shoulder.setPosition(0.55);

                wrist.setPosition(0.28);
                //setLiftHeight(0.42);
                break;
            case UPGO2:
                wrist.setPosition(0.22);
                shoulder.setPosition(0.79);

                //setLiftHeight(0.42);
                break;
            case UPGO3:

                wrist.setPosition(0.6);
                shoulder.setPosition(0.79);
                //setLiftHeight(0.42);
                break;
            case UPGO4:
                shoulder.setPosition(0.91);

                wrist.setPosition(0.6);
                //setLiftHeight(0.42);
                break;
            case UPGO5:
                wrist.setPosition(0.6);
                shoulder.setPosition(1);

                //setLiftHeight(0.42);
                break;
        }
    }
    private void armmUp(){
        if (gamepad2.a) {
            switch (currentArmPos) {
                case INTAKE:
                    currentArmPos = ArmPosition.UPGO1;
                    break;
                case UPGO1:
                    currentArmPos = ArmPosition.UPGO2;
                    break;
                case UPGO2:
                    currentArmPos = ArmPosition.UPGO3;
                    break;
                case UPGO3:
                    currentArmPos = ArmPosition.UPGO4;
                    break;
                case UPGO4:
                    currentArmPos = ArmPosition.UPGO5;
                    break;
            }
            doArmm();
            telemetry.addLine();
            telemetry.addData("armpostion", currentArmPos.toString());
            sleep(100);
        }
    }
    private void aroundthetop() {
        if (gamepad2.start) {
        shoulder.setPosition(0.56);
        wrist.setPosition(0.5);

        }
    }
    private void dumpPrep() {
        if (gamepad2.back) {
            shoulder.setPosition(0.75);
            sleep(400);

            wrist.setPosition(0.93);
            shoulder.setPosition(0.91);
        }
    }
    private void dumpPrepTwo() {
        if (gamepad2.back) {

        }
    }
    private void SlideFunction() {

    }

    private void dumpLeft() {
        if (gamepad2.left_bumper) {

        }
        if (gamepad2.right_bumper) {

        }
    }
    private void driveAroundPos() {
        if (gamepad2.dpad_down) {
            intakePos();
        }
    }
    private void drivePos() {
        if ((gamepad1.a) && (currentArmPos == ArmPosition.INTAKE)) {
            shoulder.setPosition(0.49);
            wrist.setPosition(0.55);

            //setLiftHeight(0.42);
            currentArmPos = ArmPosition.DRIVE;
        }
    }
    private void intakePos() {
        //hopper.setPosition(0.02);
        //shoulder.setPosition(0.44);
        //wrist.setPosition(0.26);
        wrist.setPosition(0.265);
        shoulder.setPosition(0.455);
        leftFinger.setPosition(0.5);
        rightFinger.setPosition(0.5);
        elbow.setPosition(0.5);

        //setLiftHeight(0.42);
        currentArmPos = ArmPosition.INTAKE;
    }
    private void theJuke() {
        if ((gamepad2.dpad_up || gamepad1.x) && (currentArmPos == ArmPosition.INTAKE)) {
            shoulder.setPosition(0.48);
            sleep(200);
            intakePos();
        }
    }
    private void incrementalIntake() {
        shoulder.setPosition(0.455);
        sleep(653);
        shoulder.setPosition(0.47);
        wrist.setPosition(0.55);
        sleep(531);
        shoulder.setPosition(0.49);
        wrist.setPosition(0.55);
    }
    private void launcherstartPos() {
        launcher.setPosition(0.8);
    }



/*
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

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addLine("Current Pose");
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
       // telemetry.update();

    }
*/


    @Override
    // NOT loop \/ - Or int of vars
    public void runOpMode() throws InterruptedException {

        androidTextToSpeech = new AndroidTextToSpeech();
        androidTextToSpeech.initialize();

        // for wires driving
        //drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // for Og driving (the best driving)
        //ogDrive = new OgDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        rightHang = hardwareMap.get(DcMotor.class, "rightHang");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "OdoMec2 is ready to run!");
        telemetry.update();



        launcher = hardwareMap.get(Servo.class, "launcher");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");

        shoulder = hardwareMap.get(Servo.class, "shoulder");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        launcher.setDirection(Servo.Direction.REVERSE);



        // sensors
        leftUpper = hardwareMap.get(RevTouchSensor.class, "leftUpper");
        rightUpper = hardwareMap.get(RevTouchSensor.class, "rightUpper");
        leftLower = hardwareMap.get(RevTouchSensor.class, "leftLower");
        rightLower = hardwareMap.get(RevTouchSensor.class, "rightLower");

        // servos
        launcherstartPos();
        intakePos();

        //this is a coment to mAKE git update
        double SLOW_DOWN_FACTOR = 0.5;
        telemetry.addData("Initializing TeleOp","");
        telemetry.update();


        waitForStart();
        runtime.reset();
       // servo_shenanigans();
        // loop real
        while(opModeIsActive()){
            //Drivetrain
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

            double rightFrontPower = (forward - strafe - turn) / denominator;
            double leftFrontPower = (forward + strafe + turn) / denominator;
            double rightBackPower = (forward + strafe - turn) / denominator;
            double leftBackPower = (forward - strafe + turn) / denominator;

            if (gamepad1.left_bumper) {
                rightFrontPower = Range.clip(rightFrontPower, -0.4, 0.4);
                leftFrontPower = Range.clip(leftFrontPower, -0.4, 0.4);
                rightBackPower = Range.clip(rightBackPower, -0.4, 0.4);
                leftBackPower = Range.clip(leftBackPower, -0.4, 0.4);
            } else {
                rightFrontPower = Range.clip(rightFrontPower, -0.8, 0.8);
                leftFrontPower = Range.clip(leftFrontPower, -0.8, 0.8);
                rightBackPower = Range.clip(rightBackPower, -0.8, 0.8);
                leftBackPower = Range.clip(leftBackPower, -0.8, 0.8);
            }


            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            rightBack.setPower(rightBackPower);
            leftBack.setPower(leftBackPower);

            if (gamepad1.right_bumper){
                leftHang.setDirection(DcMotor.Direction.FORWARD);
                rightHang.setDirection(DcMotor.Direction.FORWARD);

                leftHang.setPower(.9);
                rightHang.setPower(.9);
            } else if (leftUpper.isPressed() || rightUpper.isPressed() || gamepad1.y) {
                leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftHang.setPower(0);
                rightHang.setPower(0);
            }
            if (gamepad1.left_bumper){
                leftHang.setDirection(DcMotor.Direction.REVERSE);
                rightHang.setDirection(DcMotor.Direction.REVERSE);

                leftHang.setPower(.9);
                rightHang.setPower(.9);
            } else if (leftLower.isPressed() || rightLower.isPressed() || gamepad1.y) {
                leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftHang.setPower(0);
                rightHang.setPower(0);
            }




            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)", forward, strafe, turn);

            //Code();
            airplane();
            //ogDrive.og_drive_code(gamepad1, telemetry);
            //IsDrive.is_drive_code(gamepad1, telemetry);
            driveAroundPos();
            //dumpPrepTwo();
            //homePrep();
            drivePos();
            dumpLeft();
            theJuke();
            //liftFunction();
            //worm();
            //stopMotion();
            //aroundthetop();
            armmUp();
            armmDown();
            SlideFunction();
            telemetry.update();
            sleep(100);
        }
    }
}
