package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class OdoMec extends LinearOpMode {

    // Declare vars
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private Servo shoulder;
    private Servo wrist;
    private Servo elbow;
    private Servo leftFinger;
    private Servo rightFinger;

    private Servo rightLift;
    private Servo leftLift;

    double LiftLeftOffset = -.05;
    double LiftHeight;
    private double servoposition = 0.0;
    private double servodelta = 0.02;
    private double servodelaytime = 0.03;

    //Servo values
    private static final double SHOULDER_DRIVE = 0.425; // 0.425
    private static final double SHOULDER_INTAKE = 0.455; // 0.455
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
    private static final double SERVO_TOLERANCE = 0.01;

    private static final double LIFT_HEIGHT_INT = 0.37;

    private enum intakeState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_WRIST,
        MOVING_ELBOW,
        COMPLETED
    }
    private OdoMec.intakeState currentIntakeState = OdoMec.intakeState.IDLE;

    private enum driveState {
        IDLE,
        MOVING_SHOULDER,
        MOVING_ELBOW,
        MOVING_WRIST,
        COMPLETED
    }
    private OdoMec.driveState currentDriveState = OdoMec.driveState.IDLE;

    private void handleIntakeSequence() {
        switch (currentIntakeState) {
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                shoulder.setPosition(SHOULDER_DRIVE);
                if (isServoAtPosition(shoulder, SHOULDER_DRIVE)) {
                    currentIntakeState = OdoMec.intakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                wrist.setPosition(WRIST_INTAKE);
                if (isServoAtPosition(wrist, WRIST_INTAKE)) {
                    currentIntakeState = intakeState.MOVING_ELBOW;
                }
                break;

            case MOVING_ELBOW:
                // Move the elbow to preintake position so it don't slap the floor
                leftFinger.setPosition(LEFT_FINGER_INTAKE);
                rightFinger.setPosition(RIGHT_FINGER_INTAKE);
                moveServoGradually(elbow, ELBOW_INTAKE);
                if (isServoAtPosition(elbow, ELBOW_INTAKE)) {
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

    private void handleDriveSequence() {
        switch (currentDriveState) {
            case MOVING_SHOULDER:
                // Move the shoulder to drive position
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
                    currentDriveState = driveState.COMPLETED;
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
    private boolean isServoAtPosition(Servo servo, double position) {
        return Math.abs(servo.getPosition() - position) < SERVO_TOLERANCE;
    }

    private void liftDown() {
        leftLift.setPosition(LIFT_HEIGHT_INT);
        rightLift.setPosition(LIFT_HEIGHT_INT);
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
    @Override
    // NOT loop \/ - Or int of vars
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

// hardware mappings and parameters
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");

// init postiions

        leftFinger.setPosition(LEFT_FINGER_GRIP);
        rightFinger.setPosition(RIGHT_FINGER_GRIP);
        liftDown();
        shoulder.setPosition(SHOULDER_DRIVE);
        elbow.setPosition(ELBOW_DRIVE);
        wrist.setPosition(WRIST_DRIVE);




        telemetry.addData("Status", "OdoMec is ready to run!");
        telemetry.addData("Initializing TeleOp","");

        telemetry.update();



        waitForStart();
        runtime.reset();
        while(opModeIsActive()){


            if (gamepad1.left_bumper && currentIntakeState == OdoMec.intakeState.IDLE) {
                currentIntakeState = OdoMec.intakeState.MOVING_SHOULDER;
            }

            handleIntakeSequence();

            if (gamepad1.right_bumper && currentDriveState == OdoMec.driveState.IDLE) {
                currentDriveState = OdoMec.driveState.MOVING_SHOULDER;
            }

            handleDriveSequence();

        }

    }
}