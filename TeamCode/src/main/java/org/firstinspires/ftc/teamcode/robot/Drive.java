package org.firstinspires.ftc.teamcode.robot;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("All")

/**
 * Class responsible with controlling the drive train
 */
public class Drive {
	private Brickbot robot = Brickbot.getInstance();

	/* Constants */
	private static final double ROBOT_WIDTH   = 13.97637795d;
	private static final double ROBOT_LENGTH  = 14.17322835d;

	private static final double COUNTS_PER_MOTOR_REV    = 1120.0d;       // Modify according to the drive motors
	private static final double WHEEL_DIAMETER_INCHES   = 4.0d;
	private static final double DRIVE_GEAR_REDUCTION    = 2.0d;         // Speed reduction
	private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

	private static final double DRIVE_SPEED  = 1.0d;
	private static final double TURN_SPEED   = 1.0d;

	static final double HEADING_THRESHOLD   = 1 ;       // Ignore error if the angle is less than the threshold
	static final double P_TURN_COEFF        = 0.1;
	static final double P_DRIVE_COEFF       = 0.15;

	/* Constructor */
	public Drive() {
		if (DRIVE_SPEED > 1.0d || TURN_SPEED > 1.0d)
			throw new ArithmeticException();
	}

	public enum Direction {
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT
	}

	/**
	 * Sets the power to the drive train motors.
	 * @param args 4 doubles corresponding to the powers of:
	 * 		       <p>1. Front Left Motor</p>
	 * 		       <p>2. Front Right Motor</p>
	 * 		       <p>3. Back Left Motor</p>
	 * 		       <p>4. Back Right Motor</p>
	 * @param args
	 */
	private void setPower(@NonNull double... args) {
		if (args.length == 4) {
			robot.motorFrontLeft.setPower(args[0]);
			robot.motorFrontRight.setPower(args[1]);
			robot.motorBackLeft.setPower(args[2]);
			robot.motorBackRight.setPower(args[3]);
		}
		else {
			throw new ArrayIndexOutOfBoundsException();
		}
	}

	/**
	 * Modifies the target positions to the drive train motors depending on the current position
	 * @param args 4 ints corresponding to the positions of:
	 * 		       <p>1. Front Left Motor</p>
	 * 		       <p>2. Front Right Motor</p>
	 * 		       <p>3. Back Left Motor</p>
	 * 		       <p>4. Back Right Motor</p>
	 */
	private void modifyTargetPosition(@NonNull int... args) {
		if (args.length == 4) {
			robot.motorFrontLeft.setTargetPosition(robot.motorFrontLeft.getCurrentPosition() + args[0]);
			robot.motorFrontRight.setTargetPosition(robot.motorFrontRight.getCurrentPosition() + args[1]);
			robot.motorBackLeft.setTargetPosition(robot.motorBackLeft.getCurrentPosition() + args[2]);
			robot.motorBackRight.setTargetPosition(robot.motorBackRight.getCurrentPosition() + args[3]);
		}
		else {
			throw new ArrayIndexOutOfBoundsException();
		}
	}

	private void resetTargetPosition() {
		robot.motorFrontLeft.setTargetPosition(robot.motorFrontLeft.getCurrentPosition());
		robot.motorFrontRight.setTargetPosition(robot.motorFrontRight.getCurrentPosition());
		robot.motorBackLeft.setTargetPosition(robot.motorBackLeft.getCurrentPosition());
		robot.motorBackRight.setTargetPosition(robot.motorBackRight.getCurrentPosition());
	}

	private boolean isBusy() {
		return robot.motorFrontLeft.isBusy() &&
				robot.motorFrontRight.isBusy() &&
				robot.motorBackLeft.isBusy() &&
				robot.motorBackRight.isBusy();
	}

	/**
	 * Sets the power of the drive train motors to max
	 */
	private void startMotors() {
		robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setPower(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
	}

	/**
	 * Stops the drive train motors
	 */
	private void stopMotors() {
		setPower(0, 0, 0, 0);

		robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	/**
	 * Determines the error between the target angle and the robot's current heading.
	 * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
	 * @return  Degrees in the range +/- 180. Centered on the robot's frame of reference.
	 */
	private double getError(double targetAngle) {
		double error;
		error = targetAngle - robot.gyro.getIntegratedZValue();
		return AngleUnit.normalizeDegrees(error);
	}

	/**
	 * Returns desired steering force.  +/- 1 range.
	 * @param error   Error angle in robot relative degrees
	 * @param PCoeff  Proportional Gain Coefficient
	 */
	private double getSteer(double error, double PCoeff) {
		return Range.clip(error * PCoeff, -1, 1);
	}

	/**
	 * The robot starts moving
	 * @param direction One of 4 cardinal directions in which the robot starts moving
	 * @param unit The unit of measure for the specified distance
	 * @param distance The distance the robot will move
	 */
	public void move (Direction direction, DistanceUnit unit, double distance) {
		double angle = robot.gyro.getIntegratedZValue();
		double error;
		double steer;
		double speed1;
		double speed2;

		distance = Math.abs(distance);
		int modifier = (int)(unit.toInches(distance) * COUNTS_PER_INCH);

		switch (direction) {
			default:
			case FORWARD:
				modifyTargetPosition(modifier, modifier, modifier, modifier);
				break;
			case BACKWARD:
				modifyTargetPosition(-modifier, -modifier, -modifier, -modifier);
				break;
			case LEFT:
				modifyTargetPosition(-modifier, modifier, modifier, -modifier);
				break;
			case RIGHT:
				modifyTargetPosition(modifier, -modifier, -modifier, modifier);
				break;
		}

		startMotors();

		while (isBusy()) {
			error = getError(angle);

			if (error < HEADING_THRESHOLD)
				error = 0;

			steer = getSteer(error, P_DRIVE_COEFF);

			speed1 = DRIVE_SPEED - steer;
			speed2 = DRIVE_SPEED + steer;

			double max = Math.max(Math.abs(speed1), Math.abs(speed2));
			if (max > 1.0)
			{
				speed1 /= max;
				speed2 /= max;
			}

			switch (direction) {
				default:
				case FORWARD:
					setPower(speed1, speed2, speed1, speed2);
					break;
				case BACKWARD:
					setPower(speed2, speed1, speed2, speed1);
					break;
				case LEFT:
					setPower(speed2, speed2, speed1, speed1);
					break;
				case RIGHT:
					setPower(speed1, speed1, speed2, speed2);
					break;
			}
		}

		stopMotors();
	}

	/**
	 * The robot starts rotating
	 * @param rotation The sense of rotation in which the robot starts moving
	 * @param unit The unit of measure for the specified angle
	 * @param angle The amount the robot will rotate
	 */
	public void rotate(AngleUnit unit, double angle) {
		angle = unit.normalize(angle);
		angle = unit.toDegrees(angle);
		angle += robot.gyro.getIntegratedZValue();

		double error = getError(angle);

		while (error < HEADING_THRESHOLD) {
			double power = TURN_SPEED * Math.signum(error);
			setPower(-power, power, -power, power);
		}

		stopMotors();
	}

	/**
	 * Stops the drive train
	 */
	public void stop() {
		stopMotors();
		resetTargetPosition();
	}

	/* TeleOp Functions */
	/**
	 * Normalises and sets the power to the drive train motors.
	 * @param args 4 doubles corresponding to the powers of:
	 * 		       <p>1. Front Left Motor</p>
	 * 		       <p>2. Front Right Motor</p>
	 * 		       <p>3. Back Left Motor</p>
	 * 		       <p>4. Back Right Motor</p>
	 * @param args
	 */
	private void setNormalisedPower(@NonNull double... args) {
		if (args.length == 4) {
			double max = -1;

			for (double arg : args)
				max = Math.max(max, Math.abs(arg));

			if (max > 1)
				for (int i = 0; i < 4; i++)
					args[i] /= max;

			setPower(args);
		}
		else {
			throw new ArrayIndexOutOfBoundsException();
		}
	}

	/**
	 * Commands the drive train
	 * @param gamepad The gamepad responsible for moving the drive train
	 */
	public void teleDrive(Gamepad gamepad) {
		double x = gamepad.right_stick_x;
		double y = -gamepad.right_stick_y;
		double r = gamepad.left_stick_x;

		setNormalisedPower(x + y + r, -x + y - r, -x + y + r, x + y - r);
	}
}
