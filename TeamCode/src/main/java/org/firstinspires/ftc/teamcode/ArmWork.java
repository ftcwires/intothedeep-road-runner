package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//@TeleOp
public class ArmWork extends LinearOpMode{
    // Declare vars
    // timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

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
    private static final double ELBOW_DRIVE= 0.47;
    private static final double ELBOW_INTAKE = 0.82;
    private static final double WRIST_INT = 0.55; // int for auto & testing 0.55; // int for DRIVE 0.265
    private static final double WRIST_DRIVE = 0.825; // int for auto & testing 0.55; // int for DRIVE 0.265
    private static final double WRIST_INTAKE = 0.525;
    private static final double LEFT_FINGER_GRIP = 0.74;
    private static final double LEFT_FINGER_DROP = 0.5;
    private static final double LEFT_FINGER_INTAKE = 0.34;
    private static final double RIGHT_FINGER_GRIP = .27;
    private static final double RIGHT_FINGER_DROP = .5;
    private static final double RIGHT_FINGER_INTAKE = 0.64;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double LAUNCHER_START_POS = 0.8;
    private static final double SERVO_TOLERANCE = 0.01;
    private static final double LIFT_DRIVE = 0.10;
    private double liftTargetPosition = LIFT_DRIVE;

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
    private static final double LOW_ACC = 7.25;
    private static final double LOW_VEL = 8.5;
    private static final double MED_ACC = 10.25;
    private static final double MED_VEL = 11.5;
    private static final double HIGH_ACC = 11.25;
    private static final double HIGH_VEL = 13.5;
    
    // state machines enumeration
    // intake

    private enum intakeState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private ArmWork.intakeState currentintakeState = ArmWork.intakeState.IDLE;
    private ArmWork.IntakePosition activeIntakePosition = null;


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

    private void handleIntakeSequence(ArmWork.IntakePosition intakePos) {
        switch (currentintakeState) {
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, intakePos.shoulderPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(shoulder, intakePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentintakeState = intakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, intakePos.wristPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(wrist, intakePos.wristPosition, SERVO_TOLERANCE)) {
                    currentintakeState = intakeState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                moveServoWithTrapezoidalVelocity(elbow, intakePos.elbowPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(elbow, intakePos.elbowPosition, SERVO_TOLERANCE)) {
                    // Check if the elbow is 70% down and open the claws if it is
                    if (elbow.getPosition() > (intakePos.elbowPosition - 0.4)) {
                        leftFinger.setPosition(LEFT_FINGER_INTAKE);
                        rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                    }
                    currentintakeState = ArmWork.intakeState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentintakeState == ArmWork.intakeState.COMPLETED) {
            currentintakeState = ArmWork.intakeState.IDLE;
        }
    }


    private enum driveState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private ArmWork.driveState currentdriveState = ArmWork.driveState.IDLE;
    private ArmWork.DrivePosition activeDrivePosition = null;


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
    private void handleDriveSequence(ArmWork.DrivePosition drivePos) {
        switch (currentdriveState) {
            case MOVING_LIFT:
                setLiftPosition(LIFT_DRIVE);
                break;
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, drivePos.shoulderPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(shoulder, drivePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentdriveState = ArmWork.driveState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, drivePos.elbowPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(elbow, drivePos.elbowPosition, SERVO_TOLERANCE)) {
                    currentdriveState = ArmWork.driveState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, drivePos.wristPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(wrist, drivePos.wristPosition, SERVO_TOLERANCE)) {
                    currentdriveState = driveState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentdriveState == ArmWork.driveState.COMPLETED) {
            currentdriveState = ArmWork.driveState.IDLE;
        }
    }

    private enum scoreState {
        IDLE,
        MOVING_LIFT,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }

    private ArmWork.scoreState currentscoreState = ArmWork.scoreState.IDLE;
    private ArmWork.ScorePosition activeScorePosition = null;


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
    private void handleScoreSequence(ArmWork.ScorePosition scorePos) {
        switch (currentscoreState) {

            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, scorePos.shoulderPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(shoulder, scorePos.shoulderPosition, SERVO_TOLERANCE)) {
                    currentscoreState = ArmWork.scoreState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_GRIP);
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, scorePos.elbowPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(elbow, scorePos.elbowPosition, SERVO_TOLERANCE)) {
                    currentscoreState = ArmWork.scoreState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, scorePos.wristPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(wrist, scorePos.wristPosition, SERVO_TOLERANCE)) {
                    currentscoreState = scoreState.MOVING_LIFT;
                }
                break;
            case MOVING_LIFT:
                setLiftPosition(scorePos.liftPosition);
                if (isServoAtPosition(leftLift, scorePos.liftPosition, SERVO_TOLERANCE) && isServoAtPosition(rightLift, scorePos.liftPosition, SERVO_TOLERANCE)) {
                    currentscoreState = scoreState.COMPLETED;
                }
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentscoreState == ArmWork.scoreState.COMPLETED) {
            currentscoreState = ArmWork.scoreState.IDLE;
        }
    }
    // functions to support state machine actions
    private boolean isServoAtPosition(Servo servo, double targetPosition, double tolerance) {
        double currentPosition = servo.getPosition();
        double normalizedTarget = Range.clip(targetPosition, 0.0, 1.0); // Ensure target is within valid range
        double normalizedCurrent = Range.clip(currentPosition, 0.0, 1.0); // Ensure current position is within valid range

        return Math.abs(normalizedCurrent - normalizedTarget) < tolerance;
    }


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

            telemetry.addData("Servo Position", currentPosition);
            telemetry.update();
        }
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
    public Servo setLiftPosition(double targetPosition) {
        // Ensure the target position is within the valid range
        targetPosition = Math.max(0.0, Math.min(targetPosition, 1.0));

        // Set the servo positions
        leftLift.setPosition(targetPosition);
        rightLift.setPosition(targetPosition);
        return null;
    }

    @Override

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // servos
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");

        // servo modes
        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        // init positions
        leftFinger.setPosition(LEFT_FINGER_GRIP);
        rightFinger.setPosition(RIGHT_FINGER_GRIP);
        setLiftPosition(LIFT_DRIVE);
        shoulder.setPosition(SHOULDER_INT);
        elbow.setPosition(ELBOW_DRIVE);
        wrist.setPosition(WRIST_DRIVE);

        telemetry.addData("Status", "Armwork is ready to run!");
        telemetry.addData("Initializing TeleOp","");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Check if the left bumper is pressed and the intake state is IDLE
            if (gamepad1.left_bumper && currentintakeState == ArmWork.intakeState.IDLE) {
                activeIntakePosition = new ArmWork.IntakePosition(LIFT_DRIVE, SHOULDER_DRIVE, WRIST_INTAKE, ELBOW_INTAKE, MED_ACC, MED_VEL);
                currentintakeState = ArmWork.intakeState.MOVING_SHOULDER;
            }

            // Check if the right bumper is pressed and the drive state is IDLE
            if (gamepad1.right_bumper && currentdriveState == driveState.IDLE) {
                activeDrivePosition = new ArmWork.DrivePosition(LIFT_DRIVE, SHOULDER_DRIVE, WRIST_DRIVE, ELBOW_DRIVE, MED_ACC, MED_VEL);
                currentdriveState = ArmWork.driveState.MOVING_SHOULDER;
            }

            if (activeIntakePosition != null) {
                handleIntakeSequence(activeIntakePosition);
            }

            if (currentintakeState == ArmWork.intakeState.COMPLETED) {
                currentintakeState = ArmWork.intakeState.IDLE;
                activeIntakePosition = null; // Reset the active position
            }

            if (activeDrivePosition != null) {
                handleDriveSequence(activeDrivePosition);
            }

            if (currentdriveState == ArmWork.driveState.COMPLETED) {
                currentdriveState = ArmWork.driveState.IDLE;
                activeDrivePosition = null; // Reset the active position
            }

            // score position one
            if (gamepad2.y && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentscoreState == ArmWork.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new ArmWork.ScorePosition(SCORE_ONE_LIFT, SCORE_ONE_SHOULDER, SCORE_ONE_WRIST, SCORE_ONE_ELBOW, MED_ACC, MED_VEL);
                currentscoreState = ArmWork.scoreState.MOVING_SHOULDER;
            }
            // score position two
            if (gamepad2.b && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentscoreState == ArmWork.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new ArmWork.ScorePosition(SCORE_TWO_LIFT, SCORE_TWO_SHOULDER, SCORE_TWO_WRIST, SCORE_TWO_ELBOW, MED_ACC, MED_VEL);
                currentscoreState = ArmWork.scoreState.MOVING_SHOULDER;
            }
            // score position three
            if (gamepad2.a && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentscoreState == ArmWork.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new ArmWork.ScorePosition(SCORE_THREE_LIFT, SCORE_THREE_SHOULDER, SCORE_THREE_WRIST, SCORE_THREE_ELBOW, MED_ACC, MED_VEL);
                currentscoreState = ArmWork.scoreState.MOVING_SHOULDER;
            }
            // score position four
            //TODO: gotta put !gamepad2.left_bumper above
            if (gamepad2.x && !gamepad2.left_bumper && (gamepad2.left_trigger < TRIGGER_THRESHOLD) && (currentscoreState == ArmWork.scoreState.IDLE)) {
                // Assign a new ScorePosition inside the if block
                activeScorePosition = new ArmWork.ScorePosition(SCORE_FOUR_LIFT, SCORE_FOUR_SHOULDER, SCORE_FOUR_WRIST, SCORE_FOUR_ELBOW, MED_ACC, MED_VEL);
                currentscoreState = ArmWork.scoreState.MOVING_SHOULDER;
            }

            if (activeScorePosition != null) {
                handleScoreSequence(activeScorePosition);
            }
            if (currentscoreState == ArmWork.scoreState.COMPLETED) {
                currentscoreState = ArmWork.scoreState.IDLE;
                activeScorePosition = null; // Reset the active position
            }

            telemetry.addData("Intake", currentintakeState);
            telemetry.addData("Drive", currentdriveState);
            telemetry.addData("Shoulder Position", shoulder.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Elbow Position", elbow.getPosition());
            telemetry.addData("Intake State", currentintakeState);
            telemetry.update();
        }
    }
}
