package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp
public class OdoMec extends LinearOpMode {
// adding for push test
    // Declare vars
    // timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

    // motors
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1

    private DcMotor leftBack; //rear left 3
    private DcMotor leftHang;
    private DcMotor rightHang;

    // servos
    private Servo launcher;
    private Servo shoulder;
    private Servo wrist;
    private Servo elbow;
    private Servo leftFinger;
    private Servo rightFinger;
    private Servo rightLift;
    private Servo leftLift;
    private double servoposition = 0.0;
    private double servodelta = 0.02;
    private double servodelaytime = 0.03;


    // servo values
    private static final double SHOULDER_DRIVE = 0.425; // 0.425
    private static final double SHOULDER_INT = 0.43; // 0.455
    private static final double ELBOW_DRIVE= 0.5;
    private static final double ELBOW_INTAKE = 0.8;
    private static final double WRIST_INT = 0.55; // int for auto & testing 0.55; // int for DRIVE 0.265
    private static final double WRIST_DRIVE = 0.265; // int for auto & testing 0.55; // int for DRIVE 0.265
    private static final double WRIST_INTAKE = 0.575;
    private static final double LEFT_FINGER_GRIP = 0.74;
    private static final double LEFT_FINGER_DROP = 0.5;
    private static final double LEFT_FINGER_INTAKE = 0.34;
    private static final double RIGHT_FINGER_GRIP = .27;
    private static final double RIGHT_FINGER_DROP = .5;
    private static final double RIGHT_FINGER_INTAKE = 0.64;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double LAUNCHER_START_POS = 0.8;
    private static final double SERVO_TOLERANCE = 0.01;
    private static final double LIFT_DRIVE = 0.06;
    private double liftTargetPosition = LIFT_DRIVE;

    // stack positions (top 2 o 5 and next 2 of 3 )
    // TODO find positions with McMuffin (currently all set to drive) change to the actual double values like above from McMuffin
    // intake two off a stack of five
    private static final double SHOULDER_TOP_TWO = SHOULDER_DRIVE;
    private static final double WRIST_TOP_TWO = 0.55;
    private static final double ELBOW_TOP_TWO = 0.6;

    // intake two off a stack of three
    private static final double SHOULDER_NEXT_TWO = SHOULDER_DRIVE;
    private static final double WRIST_NEXT_TWO = 0.44;
    private static final double ELBOW_NEXT_TWO = 0.7;

    //=^-^=
    // score positions (11 rows on the board)
    // TODO find positions with McMuffin (currently all set to drive) change to the actual double values like above from McMuffin
    // score position one button map (gamepad2.y)
    private static final double SCORE_ONE_SHOULDER = 0.936;
    private static final double SCORE_ONE_WRIST = 0.685;
    private static final double SCORE_ONE_ELBOW = 0.52;
    private static final double SCORE_ONE_LIFT = LIFT_DRIVE;
    // score position two button map (gamepad2.b)
    private static final double SCORE_TWO_SHOULDER = 0.9;
    private static final double SCORE_TWO_WRIST = 0.745;
    private static final double SCORE_TWO_ELBOW = 0.55;
    private static final double SCORE_TWO_LIFT = LIFT_DRIVE;
    // score position three button map (gamepad2.a)
    private static final double SCORE_THREE_SHOULDER = 0.92;
    private static final double SCORE_THREE_WRIST = 0.91;
    private static final double SCORE_THREE_ELBOW = 0.67;
    private static final double SCORE_THREE_LIFT = LIFT_DRIVE;
    // score position four button map (gamepad2.x)
    private static final double SCORE_FOUR_SHOULDER = 0.91;
    private static final double SCORE_FOUR_WRIST =0.985;
    private static final double SCORE_FOUR_ELBOW = 0.72;
    private static final double SCORE_FOUR_LIFT = LIFT_DRIVE;
    // score position five button map (gamepad2.left_bumper && gamepad2.y)
    private static final double SCORE_FIVE_SHOULDER = 0.93;
    private static final double SCORE_FIVE_WRIST = 1;
    private static final double SCORE_FIVE_ELBOW = 0.82;
    private static final double SCORE_FIVE_LIFT = LIFT_DRIVE;
    // score position six button map (gamepad2.left_bumper && gamepad2.b)
    private static final double SCORE_SIX_SHOULDER = 0.92;
    private static final double SCORE_SIX_WRIST = 1;
    private static final double SCORE_SIX_ELBOW = 0.85;
    private static final double SCORE_SIX_LIFT = LIFT_DRIVE;
    // score position seven button map (gamepad2.left_bumper && gamepad2.a)
    private static final double SCORE_SEVEN_SHOULDER = 0.9;
    private static final double SCORE_SEVEN_WRIST = 1;
    private static final double SCORE_SEVEN_ELBOW = 0.81;
    private static final double SCORE_SEVEN_LIFT = 0.51;
    // score position eight button map (gamepad2.left_bumper && gamepad2.x)
    private static final double SCORE_EIGHT_SHOULDER = 0.94;
    private static final double SCORE_EIGHT_WRIST = 0.93;
    private static final double SCORE_EIGHT_ELBOW = 0.8;
    private static final double SCORE_EIGHT_LIFT = 0.8;
    // score position nine button map (gamepad2.left_trigger && gamepad2.y)
    private static final double SCORE_NINE_SHOULDER = 0.93;
    private static final double SCORE_NINE_WRIST = 0.93;
    private static final double SCORE_NINE_ELBOW = 0.79;
    private static final double SCORE_NINE_LIFT = 0.86;
    // score position ten button map (gamepad2.left_trigger && gamepad2.b)
    private static final double SCORE_TEN_SHOULDER = 0.9;
    private static final double SCORE_TEN_WRIST = 0.93;
    private static final double SCORE_TEN_ELBOW = 0.76;
    private static final double SCORE_TEN_LIFT = 0.98;
    // score position eleven button map (gamepad2.left_trigger && gamepad2.a)
    private static final double SCORE_ELEVEN_SHOULDER = 0.9;
    private static final double SCORE_ELEVEN_WRIST = 0.93;
    private static final double SCORE_ELEVEN_ELBOW = 0.76;
    private static final double SCORE_ELEVEN_LIFT = 0.98;

    // sensors
    private RevTouchSensor rightUpper;
    private RevTouchSensor leftUpper;
    private RevTouchSensor rightLower;
    private RevTouchSensor leftLower;


    // state machines enumeration
    // intake
    private enum intakeState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }
    private OdoMec.intakeState currentIntakeState = OdoMec.intakeState.IDLE;
    private OdoMec.IntakePosition activeIntakePosition = null;
    // drive
    private enum driveState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_ELBOW,
        MOVING_WRIST,
        COMPLETED
    }

    private OdoMec.driveState currentDriveState = OdoMec.driveState.IDLE;
    // score
    private enum scoreState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        MOVING_LIFT,
        COMPLETED
    }

    private OdoMec.scoreState currentScoreState = OdoMec.scoreState.IDLE;
    private OdoMec.ScorePosition activeScorePosition = null;

    // state machine sequences to provide cascading actions with a single input (button press)
    // intake
    private void handleIntakeSequence(OdoMec.IntakePosition intakePos) {
        switch (currentIntakeState) {
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoGradually(shoulder, intakePos.shoulderPosition + .05);
                shoulder.setPosition(intakePos.shoulderPosition);
                if (isServoAtPosition(shoulder, intakePos.shoulderPosition)) {
                    currentIntakeState = OdoMec.intakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                wrist.setPosition(intakePos.wristPosition);
                if (isServoAtPosition(wrist, intakePos.wristPosition)) {
                    currentIntakeState = intakeState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                moveServoGradually(elbow, intakePos.elbowPosition);
                if (isServoAtPosition(elbow, intakePos.elbowPosition)) {
                    currentIntakeState = intakeState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentIntakeState == OdoMec.intakeState.COMPLETED) {
            currentIntakeState = OdoMec.intakeState.IDLE;
        }
    }

    // drive
    private void handleDriveSequence() {
        switch (currentDriveState) {
            case MOVING_SHOULDER:
                // Move the shoulder to drive position
                moveServoGradually(shoulder, SHOULDER_DRIVE + .05);
                shoulder.setPosition(SHOULDER_DRIVE);
                if (isServoAtPosition(shoulder, SHOULDER_DRIVE)) {
                    currentDriveState = OdoMec.driveState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to drive position
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                elbow.setPosition(ELBOW_DRIVE);
                if (isServoAtPosition(elbow, ELBOW_DRIVE)) {
                    currentDriveState = OdoMec.driveState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to drive position
                wrist.setPosition(WRIST_DRIVE);
                if (isServoAtPosition(wrist, WRIST_DRIVE)) {
                    currentDriveState = OdoMec.driveState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentDriveState == OdoMec.driveState.COMPLETED) {
            currentDriveState = OdoMec.driveState.IDLE;
        }
    }

    // score
    private void handleScorePosSequence(OdoMec.ScorePosition scorePos) {
        switch (currentScoreState) {
            case MOVING_SHOULDER:
                // Move the shoulder to the specified position
                moveServoGradually(shoulder, scorePos.shoulderPosition - .05);
                shoulder.setPosition(scorePos.shoulderPosition);
                if (isServoAtPosition(shoulder, scorePos.shoulderPosition)) {
                    currentScoreState = OdoMec.scoreState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to the specified position
                wrist.setPosition(scorePos.wristPosition);
                if (isServoAtPosition(wrist, scorePos.wristPosition)) {
                    currentScoreState = OdoMec.scoreState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to the specified position
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                elbow.setPosition(scorePos.elbowPosition);
                if (isServoAtPosition(elbow, scorePos.elbowPosition)) {
                    currentScoreState = OdoMec.scoreState.MOVING_LIFT;
                }
                break;
            case MOVING_LIFT:
                setLiftPosition(scorePos.liftPosition);
                if (isServoAtPosition(leftLift, scorePos.liftPosition) || isServoAtPosition(rightLift, scorePos.liftPosition)) {
                    currentScoreState = OdoMec.scoreState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                currentScoreState = OdoMec.scoreState.IDLE;
                activeScorePosition = null;
                break;
        }

        if (currentScoreState == OdoMec.scoreState.COMPLETED) {
            currentScoreState = OdoMec.scoreState.IDLE;
        }
    }

    // classes to support state machine actions
    static class IntakePosition {
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;

        public IntakePosition(double shoulderPos, double wristPos, double elbowPos) {
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
        }
    }

    static class ScorePosition {
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;
        double liftPosition;

        public ScorePosition(double shoulderPos, double wristPos, double elbowPos, double liftPos) {
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
            this.liftPosition = liftPos;
        }
    }

    // functions to support state machine actions
    private boolean isServoAtPosition(Servo servo, double position) {
        return Math.abs(servo.getPosition() - position) < SERVO_TOLERANCE;
    }

    private void moveServoGradually(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();

        // Check if enough time has passed since the last update
        if (servoTimer.seconds() > servodelaytime) {
            // Determine the direction of movement
            double direction = targetPosition > currentPosition ? servodelta : -servodelta;

            // Calculate the new position
            servoposition = currentPosition + direction;
            servoposition = Range.clip(servoposition, 0, 1); // Ensure the position is within valid range

            // Update the servo position
            servo.setPosition(servoposition);

            // Reset the timer
            servoTimer.reset();
        }
    }

    // lift (has been adjusted mechanically to use the same height)
    public void setLiftPosition(double targetPosition) {
        // Ensure the target position is within the valid range
        targetPosition = Math.max(0.0, Math.min(targetPosition, 1.0));

        // Set the servo positions
        leftLift.setPosition(targetPosition);
        rightLift.setPosition(targetPosition);
    }

    // launcher
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // hardware mappings and parameters

        // motors
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

        // motor modes
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servos
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");
        launcher = hardwareMap.get(Servo.class, "launcher");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");

        // servo modes
        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        launcher.setDirection(Servo.Direction.REVERSE);

        // sensors
        leftUpper = hardwareMap.get(RevTouchSensor.class, "leftUpper");
        rightUpper = hardwareMap.get(RevTouchSensor.class, "rightUpper");
        leftLower = hardwareMap.get(RevTouchSensor.class, "leftLower");
        rightLower = hardwareMap.get(RevTouchSensor.class, "rightLower");

        // init positions
        leftFinger.setPosition(LEFT_FINGER_GRIP);
        rightFinger.setPosition(RIGHT_FINGER_GRIP);
        setLiftPosition(LIFT_DRIVE);
        shoulder.setPosition(SHOULDER_INT);
        elbow.setPosition(ELBOW_DRIVE);
        wrist.setPosition(WRIST_INTAKE);
        launcherstartPos();

        telemetry.addData("Status", "OdoMec is ready to run!");
        telemetry.addData("Initializing TeleOp","");

        waitForStart();
        runtime.reset();


        while(opModeIsActive()){

            // mecanum drive
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
                rightFrontPower = Range.clip(rightFrontPower, -1, 1);
                leftFrontPower = Range.clip(leftFrontPower, -1, 1);
                rightBackPower = Range.clip(rightBackPower, -1, 1);
                leftBackPower = Range.clip(leftBackPower, -1, 1);
            }

            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            rightBack.setPower(rightBackPower);
            leftBack.setPower(leftBackPower);

            // hanging
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD){
                if (!leftUpper.isPressed()) {
                    leftHang.setDirection(DcMotor.Direction.FORWARD);
                    leftHang.setPower(.9);
                }
                if (!rightUpper.isPressed()) {
                    rightHang.setDirection(DcMotor.Direction.FORWARD);
                    rightHang.setPower(.9);
                }
            } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD){
                if (!leftLower.isPressed()) {
                    leftHang.setDirection(DcMotor.Direction.REVERSE);
                    leftHang.setPower(.9);
                }
                if (!rightLower.isPressed()) {
                    rightHang.setDirection(DcMotor.Direction.REVERSE);
                    rightHang.setPower(.9);
                }
            } else {
                leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftHang.setPower(0);
                rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightHang.setPower(0);
            }


            // emergency stop
            if (gamepad1.y) {
                leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftHang.setPower(0);
                rightHang.setPower(0);
            }

            // claw drive position
            //TODO: add safety, this is drive around pos
            if (gamepad1.right_bumper && currentDriveState == OdoMec.driveState.IDLE) {
                currentDriveState = OdoMec.driveState.MOVING_SHOULDER;
            }
            handleDriveSequence();

            // intake
            // claw intake from floor
            //TODO: add saftey, this is intake from stuff
            if (gamepad1.left_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                activeIntakePosition = new OdoMec.IntakePosition(SHOULDER_DRIVE, WRIST_INTAKE, ELBOW_INTAKE);
                currentIntakeState = OdoMec.intakeState.MOVING_SHOULDER;
            }
            // claw intake the top 2 from a stack of 5
            if (gamepad1.b && currentIntakeState == OdoMec.intakeState.IDLE) {
                activeIntakePosition = new OdoMec.IntakePosition(SHOULDER_TOP_TWO, WRIST_TOP_TWO, ELBOW_TOP_TWO);
                currentIntakeState = OdoMec.intakeState.MOVING_SHOULDER;
            }
            // claw intake the next 2 from a stack of 3
            if (gamepad1.a && currentIntakeState == OdoMec.intakeState.IDLE) {
                activeIntakePosition = new OdoMec.IntakePosition(SHOULDER_NEXT_TWO, WRIST_NEXT_TWO, ELBOW_NEXT_TWO);
                currentIntakeState = OdoMec.intakeState.MOVING_SHOULDER;
            }
            if (activeIntakePosition != null) {
                handleIntakeSequence(activeIntakePosition);
            }
            if (currentIntakeState == OdoMec.intakeState.COMPLETED) {
                currentIntakeState = OdoMec.intakeState.IDLE;
                activeIntakePosition = null; // Reset the active position
            }

            // scoring
            // score position one
            if (gamepad2.y && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_ONE_SHOULDER, SCORE_ONE_WRIST, SCORE_ONE_ELBOW, SCORE_ONE_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position two
            if (gamepad2.b && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_TWO_SHOULDER, SCORE_TWO_WRIST, SCORE_TWO_ELBOW, SCORE_TWO_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position three
            if (gamepad2.a && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_THREE_SHOULDER, SCORE_THREE_WRIST, SCORE_THREE_ELBOW, SCORE_THREE_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position four
            //TODO: gotta put !gamepad2.left_bumper above
            if (gamepad2.x && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_FOUR_SHOULDER, SCORE_FOUR_WRIST, SCORE_FOUR_ELBOW, SCORE_FOUR_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position five
            if ((gamepad2.left_bumper && gamepad2.y) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_FIVE_SHOULDER, SCORE_FIVE_WRIST, SCORE_FIVE_ELBOW, SCORE_FIVE_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position six
            if ((gamepad2.left_bumper && gamepad2.b) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_SIX_SHOULDER, SCORE_SIX_WRIST, SCORE_SIX_ELBOW, SCORE_SIX_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position seven
            if ((gamepad2.left_bumper && gamepad2.a) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_SEVEN_SHOULDER, SCORE_SEVEN_WRIST, SCORE_SEVEN_ELBOW, SCORE_SEVEN_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position eight
            if ((gamepad2.left_bumper && gamepad2.x) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_EIGHT_SHOULDER, SCORE_EIGHT_WRIST, SCORE_EIGHT_ELBOW, SCORE_EIGHT_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position nine
            if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.y) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_NINE_SHOULDER, SCORE_NINE_WRIST, SCORE_NINE_ELBOW, SCORE_NINE_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position ten
            if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.b) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_TEN_SHOULDER, SCORE_TEN_WRIST, SCORE_TEN_ELBOW, SCORE_TEN_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            // score position eleven
            if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.a) && (currentScoreState == OdoMec.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new OdoMec.ScorePosition(SCORE_ELEVEN_SHOULDER, SCORE_ELEVEN_WRIST, SCORE_ELEVEN_ELBOW, SCORE_ELEVEN_LIFT);
                currentScoreState = OdoMec.scoreState.MOVING_SHOULDER;
            }
            if (activeScorePosition != null) {
                handleScorePosSequence(activeScorePosition);
            }
            if (currentScoreState == OdoMec.scoreState.COMPLETED) {
                currentScoreState = OdoMec.scoreState.IDLE;
                activeScorePosition = null; // Reset the active position
            }

            // dropping on the backboard for scoring
            if (gamepad2.dpad_up  && !gamepad2.right_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_DROP);
                rightFinger.setPosition(RIGHT_FINGER_DROP);
            } else if (gamepad2.dpad_left && !gamepad2.right_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_DROP);
            } else if (gamepad2.dpad_right  && !gamepad2.right_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                rightFinger.setPosition(RIGHT_FINGER_DROP);
            } else if (gamepad2.dpad_down  && !gamepad2.right_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
            }

            // grabbing off the floor or stack
            if (gamepad2.right_bumper && gamepad2.dpad_up && (currentScoreState == OdoMec.scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_left && (currentScoreState == OdoMec.scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_right && (currentScoreState == OdoMec.scoreState.IDLE)) {
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
            } else if (gamepad2.right_bumper && gamepad2.dpad_down && (currentScoreState == OdoMec.scoreState.IDLE)) {
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
            }

            // telemetry
            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)", forward, strafe, turn);
            telemetry.addData("Intake", currentIntakeState);
            telemetry.addData("Drive", currentDriveState);
            telemetry.addData("Score", currentScoreState);

            //telemetry.addData("Left Lower", leftLower.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Left Upper", leftUpper.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Right Lower", rightLower.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Right Upper", rightUpper.isPressed() ? "Pressed" : "Not Pressed");

            // launcher
            airplane();

            telemetry.update();
        }

    }
}