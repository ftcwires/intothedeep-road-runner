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

@TeleOp
public class DuckOps extends LinearOpMode {
    class ScorePosition {
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;

        public ScorePosition(double shoulderPos, double wristPos, double elbowPos) {
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
        }
    }
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

    //Servo init values
    private static final double SHOULDER_INT = 0.425;
    private static final double ELBOW_INT = 0.5;
    private static final double ELBOW_INTAKE = 0.8;
    private static final double WRIST_INT = 0.55;
    private static final double WRIST_INTAKE = 0.575;
    private static final double WRIST_DRIVE = 0.265;
    private static final double LEFT_FINGER_GRIP = 0.74;
    private static final double LEFT_FINGER_DROP = 0.5;
    private static final double LEFT_FINGER_INTAKE = 0.34;
    private static final double RIGHT_FINGER_GRIP = .27;
    private static final double RIGHT_FINGER_DROP = .5;
    private static final double RIGHT_FINGER_INTAKE = 0.64;
    // Define a threshold for trigger activation
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double LAUNCHER_START_POS = 0.8;

    private static final double SERVO_TOLERANCE = 0.01;

    private enum intakeState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_ELBOW,
        MOVING_WRIST,
        MOVING_CLAWS,
        COMPLETED
    }

    private intakeState currentintakeState = intakeState.IDLE;

    // for picking up the top two off the stack of 5
    private enum intakeStackTopTwoState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_ELBOW,
        MOVING_WRIST,
        MOVING_CLAWS,
        COMPLETED
    }
    // for picking up the second two off the stack of 3
    private intakeStackTopTwoState currentintakeStackTopTwoState = intakeStackTopTwoState.IDLE;

    private enum intakeStackSecondTwoState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_ELBOW,
        MOVING_WRIST,
        MOVING_CLAWS,
        COMPLETED
    }

    private intakeStackSecondTwoState currentintakeStackSecondTwoState = intakeStackSecondTwoState.IDLE;

    private enum driveState {
        IDLE,
        MOVING_ELBOW,
        MOVING_WRIST,
        MOVING_SHOULDER,
        COMPLETED
    }

    private driveState currentdriveState = driveState.IDLE;

    private enum scoreState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private scoreState currentscoreState = scoreState.IDLE;

    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1
    private DcMotor leftBack; //rear left 3
    private DcMotor leftHang;
    private DcMotor rightHang;
    double LiftLeftOffset = .04;
    double LiftHeight;
    private AndroidTextToSpeech androidTextToSpeech;


    // Functions \/

    private void handleIntakeSequence() {
        switch (currentintakeState) {
            case IDLE:
                // Start the sequence
                currentintakeState = intakeState.MOVING_SHOULDER;
                break;

            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                shoulder.setPosition(SHOULDER_INT);
                if (isServoAtPosition(shoulder, SHOULDER_INT)) {
                    currentintakeState = intakeState.MOVING_ELBOW;
                }
                break;

            case MOVING_ELBOW:
                // Move the elbow to intake position
                elbow.setPosition(ELBOW_INTAKE);
                if (isServoAtPosition(elbow, ELBOW_INTAKE)) {
                    currentintakeState = intakeState.MOVING_WRIST;
                }
                break;

            case MOVING_WRIST:
                // Move the wrist to intake position
                wrist.setPosition(WRIST_INTAKE);
                if (isServoAtPosition(wrist, WRIST_INTAKE)) {
                    currentintakeState = intakeState.MOVING_CLAWS;
                }
                break;

            case MOVING_CLAWS:
                // Move the claws to intake position
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                if (isServoAtPosition(leftFinger, LEFT_FINGER_INTAKE) && isServoAtPosition(rightFinger, RIGHT_FINGER_INTAKE)) {
                    currentintakeState = intakeState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        if (currentintakeState == intakeState.COMPLETED) {
            currentintakeState = intakeState.IDLE;
        }
    }
// TODO Assign Buttons to this
// intaking the top two off a stack of 5
    private void handleIntakeStackTopTwoSequence() {
        switch (currentintakeStackTopTwoState) {
            case IDLE:
                // Start the sequence
                currentintakeStackTopTwoState = intakeStackTopTwoState.MOVING_SHOULDER;
                break;

            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                shoulder.setPosition(0); // TODO set position with McTuner
                if (isServoAtPosition(shoulder, 0)) { // TODO same position as above
                    currentintakeStackTopTwoState = intakeStackTopTwoState.MOVING_ELBOW;
                }
                break;

            case MOVING_ELBOW:
                // Move the elbow to intake position
                elbow.setPosition(ELBOW_INTAKE); // TODO set position with McTuner
                if (isServoAtPosition(elbow, ELBOW_INTAKE)) { // TODO same position as above
                    currentintakeStackTopTwoState = intakeStackTopTwoState.MOVING_WRIST;
                }
                break;

            case MOVING_WRIST:
                // Move the wrist to intake position
                wrist.setPosition(WRIST_INTAKE); // TODO set position with McTuner
                if (isServoAtPosition(wrist, WRIST_INTAKE)) { // TODO same position as above
                    currentintakeStackTopTwoState = intakeStackTopTwoState.MOVING_CLAWS;
                }
                break;

            case MOVING_CLAWS:
                // Move the claws to intake position
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                if (isServoAtPosition(leftFinger, LEFT_FINGER_INTAKE) && isServoAtPosition(rightFinger, RIGHT_FINGER_INTAKE)) {
                    currentintakeStackTopTwoState = intakeStackTopTwoState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        if (currentintakeStackTopTwoState == intakeStackTopTwoState.COMPLETED) {
            currentintakeStackTopTwoState = intakeStackTopTwoState.IDLE;
        }
    }
// TODO Assign Buttons to this
// intaking the second two off a stack of 3
    private void handleIntakeStackSecondTwoSequence() {
        switch (currentintakeStackSecondTwoState) {
            case IDLE:
                // Start the sequence
                currentintakeStackSecondTwoState = intakeStackSecondTwoState.MOVING_SHOULDER;
                break;

            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                shoulder.setPosition(0); // TODO set position with McTuner
                if (isServoAtPosition(shoulder, 0)) { // TODO same position as above
                    currentintakeStackSecondTwoState = intakeStackSecondTwoState.MOVING_ELBOW;
                }
                break;

            case MOVING_ELBOW:
                // Move the elbow to intake position
                elbow.setPosition(ELBOW_INTAKE); // TODO set position with McTuner
                if (isServoAtPosition(elbow, ELBOW_INTAKE)) { // TODO same position as above
                    currentintakeStackSecondTwoState = intakeStackSecondTwoState.MOVING_WRIST;
                }
                break;

            case MOVING_WRIST:
                // Move the wrist to intake position
                wrist.setPosition(WRIST_INTAKE); // TODO set position with McTuner
                if (isServoAtPosition(wrist, WRIST_INTAKE)) { // TODO same position as above
                    currentintakeStackSecondTwoState = intakeStackSecondTwoState.MOVING_CLAWS;
                }
                break;

            case MOVING_CLAWS:
                // Move the claws to intake position
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                if (isServoAtPosition(leftFinger, LEFT_FINGER_INTAKE) && isServoAtPosition(rightFinger, RIGHT_FINGER_INTAKE)) {
                    currentintakeStackSecondTwoState = intakeStackSecondTwoState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        if (currentintakeStackSecondTwoState == intakeStackSecondTwoState.COMPLETED) {
            currentintakeStackSecondTwoState = intakeStackSecondTwoState.IDLE;
        }
    }
    private void handleDriveSequence() {
        switch (currentdriveState) {
            case MOVING_ELBOW:
                // Move the elbow to drive position
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                elbow.setPosition(ELBOW_INT);
                if (isServoAtPosition(elbow, ELBOW_INT)) {
                    currentdriveState = driveState.MOVING_WRIST;
                }
                break;

            case MOVING_WRIST:
                // Move the wrist to drive position
                wrist.setPosition(WRIST_DRIVE);
                if (isServoAtPosition(wrist, WRIST_DRIVE)) {
                    currentdriveState = driveState.MOVING_SHOULDER;
                }
                break;

            case MOVING_SHOULDER:
                // Move the shoulder to drive position
                shoulder.setPosition(SHOULDER_INT);
                if (isServoAtPosition(shoulder, SHOULDER_INT)) {
                    currentdriveState = driveState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentdriveState == driveState.COMPLETED) {
            currentdriveState = driveState.IDLE;
        }
    }

    private void handleScorePosSequence(ScorePosition scorePos) {
        switch (currentscoreState) {
            case IDLE:
                // Start the sequence
                currentscoreState = scoreState.MOVING_SHOULDER;
                break;

            case MOVING_SHOULDER:
                // Move the shoulder to the specified position
                shoulder.setPosition(scorePos.shoulderPosition);
                if (isServoAtPosition(shoulder, scorePos.shoulderPosition)) {
                    currentscoreState = scoreState.MOVING_WRIST;
                }
                break;

            case MOVING_WRIST:
                // Move the wrist to the specified position
                wrist.setPosition(scorePos.wristPosition);
                if (isServoAtPosition(wrist, scorePos.wristPosition)) {
                    currentscoreState = scoreState.MOVING_ELBOW;
                }
                break;

            case MOVING_ELBOW:
                // Move the elbow to the specified position
                elbow.setPosition(scorePos.elbowPosition);
                if (isServoAtPosition(elbow, scorePos.elbowPosition)) {
                    currentscoreState = scoreState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }

        if (currentscoreState == scoreState.COMPLETED) {
            currentscoreState = scoreState.IDLE;
        }
    }


    private boolean isServoAtPosition(Servo servo, double position) {
        return Math.abs(servo.getPosition() - position) < SERVO_TOLERANCE;
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

    private void airplane() {
        if (gamepad2.back) {
            launcher.setPosition(0.1);
        } else {
            launcher.setPosition(0.8);
        }
    }

    private void launcherstartPos() {
        launcher.setPosition(LAUNCHER_START_POS);
    }


    @Override
    // NOT loop \/ - Or int of vars
    public void runOpMode() throws InterruptedException {

        androidTextToSpeech = new AndroidTextToSpeech();
        androidTextToSpeech.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Intake State", currentintakeState);
// ... other telemetry data ...
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

        // servos init
        wrist.setPosition(WRIST_INT);
        elbow.setPosition(ELBOW_INT);
        shoulder.setPosition(SHOULDER_INT);
        leftFinger.setPosition(LEFT_FINGER_GRIP);
        rightFinger.setPosition(RIGHT_FINGER_GRIP);

        telemetry.addData("Status", "DuckOps is ready to run!");
        telemetry.update();

        telemetry.addData("Initializing TeleOp","");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // loop real
        while(opModeIsActive()){

// Mecanum Drive
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

// Hanging
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD){
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

            if (gamepad1.left_trigger > TRIGGER_THRESHOLD){
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
// Intaking from the floor

            if (gamepad1.left_bumper && currentintakeState == intakeState.IDLE) {
                currentintakeState = intakeState.MOVING_SHOULDER;
            }

            handleIntakeSequence();

// Go to drive around position
            // Check if the right bumper is pressed to start the drive sequence
            if (gamepad1.right_bumper && (currentdriveState == driveState.IDLE || currentscoreState == scoreState.IDLE)) {
                currentdriveState = driveState.MOVING_ELBOW;
            }

            handleDriveSequence();

// Go to score position one
            ScorePosition positionOne = null;

            if (gamepad2.left_bumper && gamepad2.y && currentdriveState == driveState.IDLE) {
                // Assign a new ScorePosition to positionOne inside the if block
                positionOne = new ScorePosition(SHOULDER_INT, WRIST_INT, ELBOW_INTAKE);
            }

            if (positionOne != null) {
                handleScorePosSequence(positionOne);
            }


// Dropping on the backboard for scoring
            // operate that claw dropping style
            if (gamepad2.dpad_up  && currentintakeState == intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_DROP);
                rightFinger.setPosition(RIGHT_FINGER_DROP);
            } else if (gamepad2.dpad_left  && currentintakeState == intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_DROP);
            } else if (gamepad2.dpad_right  && currentintakeState == intakeState.IDLE) {
                rightFinger.setPosition(RIGHT_FINGER_DROP);
            } else if (gamepad2.dpad_down  && currentintakeState == intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
            }

// Grabbing off the floor or stack
            // operate that claw gripping style
            if (gamepad2.right_bumper && gamepad2.dpad_up && (currentscoreState == scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_left && (currentscoreState == scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_right && (currentscoreState == scoreState.IDLE)) {
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_down && (currentscoreState == scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
            }

// Telemetry
            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)", forward, strafe, turn);


            airplane();
            telemetry.update();
        }
    }
}
