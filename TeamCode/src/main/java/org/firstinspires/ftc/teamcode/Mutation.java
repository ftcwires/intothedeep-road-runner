package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Mutation extends LinearOpMode {

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

    private static final double LOW_ACC = 7.25;
    private static final double LOW_VEL = 8.5;
    private static final double MED_ACC = 10.25;
    private static final double MED_VEL = 11.5;
    private static final double HIGH_ACC = 11.25;
    private static final double HIGH_VEL = 13.5;

    private static final double SUPER_ACC = 20.0;
    private static final double SUPER_VEL = 22.0;


    // servo values
    public static final double SHOULDER_DRIVE = 0.425; // 0.425
    public static final double SHOULDER_INT = 0.43;
    public static final double ELBOW_DRIVE= 0.47;
    public static final double ELBOW_INTAKE = 0.78;
    public static final double WRIST_TUCK = 0.29;
    public static final double WRIST_DRIVE = 0.82;
    public static final double WRIST_INTAKE = 0.555;
    public static final double LEFT_FINGER_GRIP = 0.72;
    public static final double LEFT_FINGER_DROP = 0.5;
    public static final double LEFT_FINGER_INTAKE = 0.3;
    public static final double RIGHT_FINGER_GRIP = .27;
    public static final double RIGHT_FINGER_DROP = .5;
    public static final double RIGHT_FINGER_INTAKE = 0.64;
    public static final double TRIGGER_THRESHOLD = 0.5;
    public static final double LAUNCHER_START_POS = 0.8;
    public static final double SERVO_TOLERANCE = 0.01;
    public static final double LIFT_DRIVE = 0.10; // 0.78 is the highest it can mechanically go right now
    private double liftTargetPosition = LIFT_DRIVE; // was 0.37 before moving servos for larger range

    // stack positions (top 2 o 5 and next 2 of 3 )
    // TODO find positions with McMuffin (currently all set to drive) change to the actual double values like above from McMuffin
    // intake two off a stack of five
    public static final double SHOULDER_TOP_TWO = 0.425;
    public static final double WRIST_TOP_TWO = 0.59;
    public static final double ELBOW_TOP_TWO = 0.74;

    // intake two off a stack of three
    public static final double SHOULDER_NEXT_TWO = 0.425;
    public static final double WRIST_NEXT_TWO = 0.565;
    public static final double ELBOW_NEXT_TWO = 0.76;


    //=^-^=
    // score positions (11 rows on the board)
    // TODO find positions with McMuffin (currently all set to drive) change to the actual double values like above from McMuffin
    // score position one button map (gamepad2.y)
    public static final double SCORE_ONE_SHOULDER = 0.915;
    public static final double SCORE_ONE_WRIST = 0.365;
    public static final double SCORE_ONE_ELBOW = 0.57;
    public static final double SCORE_ONE_LIFT = LIFT_DRIVE;
    // score position two button map (gamepad2.b)
    public static final double SCORE_TWO_SHOULDER = SCORE_ONE_SHOULDER;
    public static final double SCORE_TWO_WRIST = SCORE_ONE_WRIST;
    public static final double SCORE_TWO_ELBOW = SCORE_ONE_ELBOW;
    public static final double SCORE_TWO_LIFT = 0.26;
    // score position three button map (gamepad2.a)
    public static final double SCORE_THREE_SHOULDER = SCORE_ONE_SHOULDER;
    public static final double SCORE_THREE_WRIST = SCORE_ONE_WRIST;
    public static final double SCORE_THREE_ELBOW = SCORE_ONE_ELBOW;
    public static final double SCORE_THREE_LIFT = 0.37;
    // score position four button map (gamepad2.x)
    public static final double SCORE_FOUR_SHOULDER = SCORE_ONE_SHOULDER;
    public static final double SCORE_FOUR_WRIST = SCORE_ONE_WRIST;
    public static final double SCORE_FOUR_ELBOW = SCORE_ONE_ELBOW;
    public static final double SCORE_FOUR_LIFT = .44;
    // score position five button map (gamepad2.left_bumper && gamepad2.y)
    public static final double SCORE_FIVE_SHOULDER = SCORE_ONE_SHOULDER;
    public static final double SCORE_FIVE_WRIST = SCORE_ONE_WRIST;
    public static final double SCORE_FIVE_ELBOW = SCORE_ONE_ELBOW;
    public static final double SCORE_FIVE_LIFT = 0.55;
    // score position six button map (gamepad2.left_bumper && gamepad2.b)
    private static final double SCORE_SIX_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_SIX_WRIST = SCORE_ONE_WRIST;
    private static final double SCORE_SIX_ELBOW = SCORE_ONE_ELBOW;
    private static final double SCORE_SIX_LIFT = .66;
    // score position seven button map (gamepad2.left_bumper && gamepad2.a)
    private static final double SCORE_SEVEN_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_SEVEN_WRIST = 0.305;
    private static final double SCORE_SEVEN_ELBOW = 0.6;
    private static final double SCORE_SEVEN_LIFT = .66;
    // score position eight button map (gamepad2.left_bumper && gamepad2.x)
    private static final double SCORE_EIGHT_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_EIGHT_WRIST = 0.235;
    private static final double SCORE_EIGHT_ELBOW = 0.67;
    private static final double SCORE_EIGHT_LIFT = .66;
    // score position nine button map (gamepad2.left_trigger && gamepad2.y)
    private static final double SCORE_NINE_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_NINE_WRIST = 0.174;
    private static final double SCORE_NINE_ELBOW = 0.72;
    private static final double SCORE_NINE_LIFT = .66;
    // score position ten button map (gamepad2.left_trigger && gamepad2.b)
    private static final double SCORE_TEN_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_TEN_WRIST = SCORE_ONE_WRIST;
    private static final double SCORE_TEN_ELBOW = SCORE_ONE_ELBOW;
    private static final double SCORE_TEN_LIFT = .66;
    // score position eleven button map (gamepad2.left_trigger && gamepad2.a)
    private static final double SCORE_ELEVEN_SHOULDER = SCORE_ONE_SHOULDER;
    private static final double SCORE_ELEVEN_WRIST = SCORE_ONE_WRIST;
    private static final double SCORE_ELEVEN_ELBOW = SCORE_ONE_ELBOW;
    private static final double SCORE_ELEVEN_LIFT = .66;
    
    // sensors
    private RevTouchSensor rightUpper;
    private RevTouchSensor leftUpper;
    private RevTouchSensor rightLower;
    private RevTouchSensor leftLower;


    // state machines - states enumeration, positions, handle sequences, functions
    // INTAKE
    private enum intakeState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        MOVING_CLAWS,
        COMPLETED
    }

    private Mutation.intakeState currentIntakeState = Mutation.intakeState.IDLE;
    private Mutation.IntakePosition activeIntakePosition = null;


    static class IntakePosition {
        double liftPosition;
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;
        double accelerationMax;
        double velocityMax;

        public IntakePosition(double liftPos, double shoulderPos, double wristPos, double elbowPos, double accMax, double velMax) {
            this.liftPosition = liftPos;
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }

    private void handleIntakeSequence(Mutation.IntakePosition intakePos) {
        switch (currentIntakeState) {
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, intakePos.shoulderPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(shoulder, intakePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentIntakeState = Mutation.intakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                //moveServoGradually(wrist, intakePos.wristPosition);
                wrist.setPosition(intakePos.wristPosition);
                //moveServoWithTrapezoidalVelocity(wrist, intakePos.wristPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(wrist, intakePos.wristPosition, SERVO_TOLERANCE)) {
                    currentIntakeState = Mutation.intakeState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                moveServoGradually(elbow, intakePos.elbowPosition);
               // moveServoWithTrapezoidalVelocity(elbow, intakePos.elbowPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(elbow, intakePos.elbowPosition, SERVO_TOLERANCE)) {
                    // Check if the elbow is 70% down and open the claws if it is
                    currentIntakeState = Mutation.intakeState.MOVING_CLAWS;
                }
                break;
            case MOVING_CLAWS:
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                if (isServoAtPosition(leftFinger, LEFT_FINGER_INTAKE, SERVO_TOLERANCE) || isServoAtPosition(rightFinger, RIGHT_FINGER_INTAKE, SERVO_TOLERANCE)) {
                    // Check if the elbow is 70% down and open the claws if it is
                    currentIntakeState = Mutation.intakeState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentIntakeState == Mutation.intakeState.COMPLETED) {
            currentIntakeState = Mutation.intakeState.IDLE;
        }
    }

    private void intakeFunction() {
        // intake
        // claw intake from floor
        //TODO: add saftey
        if (gamepad1.left_bumper && currentIntakeState == Mutation.intakeState.IDLE) {
            activeIntakePosition = new Mutation.IntakePosition(LIFT_DRIVE, SHOULDER_DRIVE, WRIST_INTAKE, ELBOW_INTAKE, SUPER_ACC, SUPER_VEL);
            currentIntakeState = Mutation.intakeState.MOVING_SHOULDER;
        }
        // claw intake the top 2 from a stack of 5
        if (gamepad1.b && currentIntakeState == Mutation.intakeState.IDLE) {
            activeIntakePosition = new Mutation.IntakePosition(LIFT_DRIVE, SHOULDER_TOP_TWO, WRIST_TOP_TWO, ELBOW_TOP_TWO, MED_ACC, MED_VEL);
            currentIntakeState = Mutation.intakeState.MOVING_SHOULDER;
        }
        // claw intake the next 2 from a stack of 3
        if (gamepad1.a && currentIntakeState == Mutation.intakeState.IDLE) {
            activeIntakePosition = new Mutation.IntakePosition(LIFT_DRIVE, SHOULDER_NEXT_TWO, WRIST_NEXT_TWO, ELBOW_NEXT_TWO, MED_ACC, MED_VEL);
            currentIntakeState = Mutation.intakeState.MOVING_SHOULDER;
        }
        if (activeIntakePosition != null) {
            handleIntakeSequence(activeIntakePosition);
        }
        if (currentIntakeState == Mutation.intakeState.COMPLETED) {
            currentIntakeState = Mutation.intakeState.IDLE;
            activeIntakePosition = null; // Reset the active position
        }

    }

    // DRIVE
    private enum driveState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private Mutation.driveState currentDriveState = Mutation.driveState.IDLE;
    private Mutation.DrivePosition activeDrivePosition = null;


    static class DrivePosition {
        double liftPosition;
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;
        double accelerationMax;
        double velocityMax;

        public DrivePosition(double liftPos, double shoulderPos, double wristPos, double elbowPos, double accMax, double velMax) {
            this.liftPosition = liftPos;
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }
    private void handleDriveSequence(Mutation.DrivePosition drivePos) {
        switch (currentDriveState) {
            case MOVING_LIFT:
                setLiftPosition(LIFT_DRIVE);
                currentDriveState = Mutation.driveState.MOVING_SHOULDER;
                break;
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, drivePos.shoulderPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(shoulder, drivePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentDriveState = Mutation.driveState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, drivePos.elbowPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(elbow, drivePos.elbowPosition, SERVO_TOLERANCE)) {
                    currentDriveState = Mutation.driveState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, drivePos.wristPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(wrist, drivePos.wristPosition, SERVO_TOLERANCE)) {
                    currentDriveState = Mutation.driveState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentDriveState == Mutation.driveState.COMPLETED) {
            currentDriveState = Mutation.driveState.IDLE;
        }
    }

    private void drivingFunction() {
        // Check if the right bumper is pressed and the drive state is IDLE
        if (gamepad1.right_bumper && currentDriveState == Mutation.driveState.IDLE) {
            activeDrivePosition = new Mutation.DrivePosition(LIFT_DRIVE, SHOULDER_DRIVE, WRIST_TUCK, ELBOW_DRIVE, SUPER_ACC, SUPER_VEL);
            currentDriveState = Mutation.driveState.MOVING_LIFT;
        }

        if (activeDrivePosition != null) {
            handleDriveSequence(activeDrivePosition);
        }

        if (currentDriveState == Mutation.driveState.COMPLETED) {
            currentDriveState = Mutation.driveState.IDLE;
            activeDrivePosition = null; // Reset the active position
        }
    }
    // SCORE
    private enum scoreState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private Mutation.scoreState currentScoreState = Mutation.scoreState.IDLE;
    private Mutation.ScorePosition activeScorePosition = null;


    static class ScorePosition {
        double liftPosition;
        double shoulderPosition;
        double wristPosition;
        double elbowPosition;
        double accelerationMax;
        double velocityMax;

        public ScorePosition(double liftPos, double shoulderPos, double wristPos, double elbowPos, double accMax, double velMax) {
            this.liftPosition = liftPos;
            this.shoulderPosition = shoulderPos;
            this.wristPosition = wristPos;
            this.elbowPosition = elbowPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }
    private void handleScoreSequence(Mutation.ScorePosition scorePos) {
        switch (currentScoreState) {

            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, scorePos.shoulderPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(shoulder, scorePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentScoreState = Mutation.scoreState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, scorePos.elbowPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(elbow, scorePos.elbowPosition, SERVO_TOLERANCE)) {
                    currentScoreState = Mutation.scoreState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, scorePos.wristPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(wrist, scorePos.wristPosition, SERVO_TOLERANCE)) {
                    currentScoreState = Mutation.scoreState.MOVING_LIFT;
                }
                break;
            case MOVING_LIFT:
                setLiftPosition(scorePos.liftPosition);
                if (isServoAtPosition(leftLift, scorePos.liftPosition, SERVO_TOLERANCE) && isServoAtPosition(rightLift, scorePos.liftPosition, SERVO_TOLERANCE)) {
                    currentScoreState = Mutation.scoreState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentScoreState == Mutation.scoreState.COMPLETED) {
            currentScoreState = Mutation.scoreState.IDLE;
        }
    }

    private void scoringFunction() {
        // scoring
        // score position one
        if (gamepad2.y && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_ONE_LIFT, SCORE_ONE_SHOULDER, SCORE_ONE_WRIST, SCORE_ONE_ELBOW, SUPER_ACC, SUPER_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position two
        if (gamepad2.b && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_TWO_LIFT, SCORE_TWO_SHOULDER, SCORE_TWO_WRIST, SCORE_TWO_ELBOW, SUPER_ACC, SUPER_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position three
        if (gamepad2.a && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_THREE_LIFT, SCORE_THREE_SHOULDER, SCORE_THREE_WRIST, SCORE_THREE_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position four
        //TODO: gotta put !gamepad2.left_bumper above
        if (gamepad2.x && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_FOUR_LIFT, SCORE_FOUR_SHOULDER, SCORE_FOUR_WRIST, SCORE_FOUR_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position five
        if ((gamepad2.left_bumper && gamepad2.y) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_FIVE_LIFT, SCORE_FIVE_SHOULDER, SCORE_FIVE_WRIST, SCORE_FIVE_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position six
        if ((gamepad2.left_bumper && gamepad2.b) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_SIX_LIFT, SCORE_SIX_SHOULDER, SCORE_SIX_WRIST, SCORE_SIX_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position seven
        if ((gamepad2.left_bumper && gamepad2.a) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_SEVEN_LIFT, SCORE_SEVEN_SHOULDER, SCORE_SEVEN_WRIST, SCORE_SEVEN_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position eight
        if ((gamepad2.left_bumper && gamepad2.x) && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_EIGHT_LIFT, SCORE_EIGHT_SHOULDER, SCORE_EIGHT_WRIST, SCORE_EIGHT_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position nine
        if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.y) && !gamepad2.left_bumper && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_NINE_LIFT, SCORE_NINE_SHOULDER, SCORE_NINE_WRIST, SCORE_NINE_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position ten
        if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.b) && !gamepad2.left_bumper && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_TEN_LIFT, SCORE_TEN_SHOULDER, SCORE_TEN_WRIST, SCORE_TEN_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        // score position eleven
        if (((gamepad2.left_trigger > TRIGGER_THRESHOLD) && gamepad2.a) && !gamepad2.left_bumper && (currentScoreState == Mutation.scoreState.IDLE)) {
            // Assign a new ScorePosition inside the if block
            activeScorePosition = new Mutation.ScorePosition(SCORE_ELEVEN_LIFT, SCORE_ELEVEN_SHOULDER, SCORE_ELEVEN_WRIST, SCORE_ELEVEN_ELBOW, MED_ACC, MED_VEL);
            currentScoreState = Mutation.scoreState.MOVING_SHOULDER;
        }
        if (activeScorePosition != null) {
            handleScoreSequence(activeScorePosition);
        }
        if (currentScoreState == Mutation.scoreState.COMPLETED) {
            currentScoreState = Mutation.scoreState.IDLE;
            activeScorePosition = null; // Reset the active position
        }
    }
    
    // functions to support state machine actions using servos

    // check servo positions with some tolerance
    private boolean isServoAtPosition(Servo servo, double targetPosition, double tolerance) {
        double currentPosition = servo.getPosition();
        double normalizedTarget = Range.clip(targetPosition, 0.0, 1.0); // Ensure target is within valid range
        double normalizedCurrent = Range.clip(currentPosition, 0.0, 1.0); // Ensure current position is within valid range

        return Math.abs(normalizedCurrent - normalizedTarget) < tolerance;
    }

    // move servo with ramping and soft-start created from the math and methods explained here: https://www.instructables.com/Servo-Ramping-and-Soft-Start/
    private void moveServoWithTrapezoidalVelocity(Servo servo, double targetPosition, double amax, double vmax) {
        double currentPosition = servo.getPosition();
        double currentVelocity = 0.0; // Initial velocity is zero
        double p0 = 0.0; // Distance needed to come to rest

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (Math.abs(currentPosition - targetPosition) > SERVO_TOLERANCE && opModeIsActive()) {
            // Calculate the elapsed time since the last update
            double elapsedTime = timer.seconds();
            timer.reset();

            // Calculate the distance needed to come to rest given the current velocity
            p0 = 2 * currentVelocity + Math.abs(currentVelocity) * currentVelocity / (2 * amax);

            // Calculate the desired velocity based on the position error
            double velocityError = targetPosition - currentPosition - p0;
            double sign = Math.signum(velocityError);
            double desiredVelocity = currentVelocity + sign * amax;
            desiredVelocity = Range.clip(desiredVelocity, -vmax, vmax); // Constrain to vmax

            // Update the servo position based on the desired velocity and elapsed time
            currentPosition += desiredVelocity * elapsedTime;
            currentPosition = Range.clip(currentPosition, 0, 1); // Ensure the position is within valid range
            servo.setPosition(currentPosition);

            // Update the current velocity for the next iteration
            currentVelocity = desiredVelocity;

           // telemetry.addData("Servo Position", currentPosition);
           // telemetry.update();
        }
    }

    // moves servo gradually
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
    public Servo setLiftPosition(double targetPosition) {
        // Ensure the target position is within the valid range
        targetPosition = Math.max(0.0, Math.min(targetPosition, 1.0));

        // Set the servo positions
        leftLift.setPosition(targetPosition);
        rightLift.setPosition(targetPosition);
        return null;
    }

    // Mecanum thread creation
    private class MecanumDriveRunnable implements Runnable {
        public volatile boolean running = true;

        @Override
        public void run() {
            while (running && opModeIsActive()) {
                try {
                    driveCode();
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    running = false; // Set running to false to stop the loop
                    Thread.currentThread().interrupt(); // Preserve interrupt status
                    break; // Break the loop to stop the thread
                }
            }
        }
    }
    private void driveCode() {
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
    }

     private void hangCode() {
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
    }


    private void emergencyStop() {
        if (gamepad1.y) {
            leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftHang.setPower(0);
            rightHang.setPower(0);
        }
    }

    // operating the claws to grab and drop
    private void grapdropFunction() {
        // dropping on the backboard for scoring
        if (gamepad2.dpad_up  && !gamepad2.right_bumper && currentIntakeState == Mutation.intakeState.IDLE) {
            leftFinger.setPosition(LEFT_FINGER_DROP);
            rightFinger.setPosition(RIGHT_FINGER_DROP);
        } else if (gamepad2.dpad_left && !gamepad2.right_bumper && currentIntakeState == Mutation.intakeState.IDLE) {
            leftFinger.setPosition(LEFT_FINGER_DROP);
        } else if (gamepad2.dpad_right  && !gamepad2.right_bumper && currentIntakeState == Mutation.intakeState.IDLE) {
            rightFinger.setPosition(RIGHT_FINGER_DROP);
        } else if (gamepad2.dpad_down  && !gamepad2.right_bumper && currentIntakeState == Mutation.intakeState.IDLE) {
            leftFinger.setPosition(LEFT_FINGER_INTAKE);
            rightFinger.setPosition(RIGHT_FINGER_INTAKE);
        }

        // grabbing off the floor or stack
        if (gamepad2.right_bumper && gamepad2.dpad_up && (currentScoreState == Mutation.scoreState.IDLE)) {
            leftFinger.setPosition(LEFT_FINGER_GRIP);
            rightFinger.setPosition(RIGHT_FINGER_GRIP);
        } else if (gamepad2.right_bumper && gamepad2.dpad_left && (currentScoreState == Mutation.scoreState.IDLE)) {
            leftFinger.setPosition(LEFT_FINGER_GRIP);
        } else if (gamepad2.right_bumper && gamepad2.dpad_right && (currentScoreState == Mutation.scoreState.IDLE)) {
            rightFinger.setPosition(RIGHT_FINGER_GRIP);
        } else if (gamepad2.right_bumper && gamepad2.dpad_down && (currentScoreState == Mutation.scoreState.IDLE)) {
            leftFinger.setPosition(LEFT_FINGER_INTAKE);
            rightFinger.setPosition(RIGHT_FINGER_INTAKE);
        }
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
        wrist.setPosition(WRIST_TUCK);
        launcherstartPos();

        telemetry.addData("Status", "OdoMec is ready to run!");
        telemetry.addData("Initializing TeleOp","");

        // Mecanum drive in a separate thread to avoid blocks in state machines like any loops (while/for) or sleep()
        MecanumDriveRunnable mecanumDriveRunnable = new MecanumDriveRunnable();
        Thread mecanumDriveThread = new Thread(mecanumDriveRunnable);


        waitForStart();
        runtime.reset();

        mecanumDriveThread.start();

        while(opModeIsActive()){

            // telemetry
            /*
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Forward", gamepad1.left_stick_y);
            telemetry.addData("Strafe", -gamepad1.left_stick_x);
            telemetry.addData("Turn", -gamepad1.right_stick_x);
            telemetry.addData("Mecanum Thread Running", mecanumDriveRunnable.running);
            telemetry.addData("Loop Time", "Duration: " + runtime.milliseconds() + " ms");


            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Intake", currentIntakeState);
            telemetry.addData("Drive", currentDriveState);
            telemetry.addData("Score", currentScoreState);
            telemetry.addData("Shoulder Position", shoulder.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Elbow Position", elbow.getPosition());
            telemetry.addData("Lift Left", leftLift.getPosition());
            telemetry.addData("Lift Right", rightLift.getPosition());
            */
            //telemetry.addData("Left Lower", leftLower.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Left Upper", leftUpper.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Right Lower", rightLower.isPressed() ? "Pressed" : "Not Pressed");
            //telemetry.addData("Right Upper", rightUpper.isPressed() ? "Pressed" : "Not Pressed");

            // launcher
            airplane();
            // hang
            hangCode();
            // claws
            grapdropFunction();
            // intake positions
            intakeFunction();
            // driving pos
            drivingFunction();
            // scoring positions
            scoringFunction();
            // emergency stop slides
            emergencyStop();
            // mecanum drive
            //driveCode(); say what
            telemetry.update();
        }

        // Stop the mecanum drive thread after the op mode is over
        mecanumDriveRunnable.running = false;
        mecanumDriveThread.interrupt();
        try {
            mecanumDriveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }
}