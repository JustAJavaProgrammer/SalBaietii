package org.firstinspires.ftc.teamcode.robot;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("ALL")

public class Drive {

    /* AUTONOMOUS */
    private static final double COUNTS_PER_MOTOR_REV    = 1120.0d;       // Modify according to the drive motors
    private static final double WHEEL_DIAMETER_INCHES   = 4.0d;
    private static final double DRIVE_GEAR_REDUCTION    = 0.5d;         // Speed reduction
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double DRIVE_SPEED  = 0.8d;
    private static final double TURN_SPEED   = 0.8d;

    static final double HEADING_THRESHOLD   = 1 ;       // Ignore error if the angle is less than the threshold
    static final double P_TURN_COEFF        = 0.1;
    static final double P_DRIVE_COEFF       = 0.15;

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private static void setPower(@NonNull double... args) {
        if(args.length == 4) {
            Robot.motorFrontLeft.setPower(args[0]);
            Robot.motorFrontRight.setPower(args[1]);
            Robot.motorBackLeft.setPower(args[2]);
            Robot.motorBackRight.setPower(args[3]);

            Robot.telemetry.addData("FL", Double.toString(args[0]));
            Robot.telemetry.addData("FR", Double.toString(args[1]));
            Robot.telemetry.addData("BL", Double.toString(args[2]));
            Robot.telemetry.addData("BR", Double.toString(args[3]));
            Robot.telemetry.update();
        } else {
            throw new ArrayIndexOutOfBoundsException();
        }

    }

    private static void modifyTargetPosition(@NonNull int... args) {
        if (args.length == 4) {
            Robot.motorFrontLeft.setTargetPosition(Robot.motorFrontLeft.getCurrentPosition() + args[0]);
            Robot.motorFrontRight.setTargetPosition(Robot.motorFrontRight.getCurrentPosition() + args[1]);
            Robot.motorBackLeft.setTargetPosition(Robot.motorBackLeft.getCurrentPosition() + args[2]);
            Robot.motorBackRight.setTargetPosition(Robot.motorBackRight.getCurrentPosition() + args[3]);
        }
        else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    private static void resetTargetPosition() {
        Robot.motorFrontLeft.setTargetPosition(Robot.motorFrontLeft.getCurrentPosition());
        Robot.motorFrontRight.setTargetPosition(Robot.motorFrontRight.getCurrentPosition());
        Robot.motorBackLeft.setTargetPosition(Robot.motorBackLeft.getCurrentPosition());
        Robot.motorBackRight.setTargetPosition(Robot.motorBackRight.getCurrentPosition());
    }

    private static boolean isBusy() {
        return Robot.motorFrontLeft.isBusy() &&
                Robot.motorFrontRight.isBusy() &&
                Robot.motorBackLeft.isBusy() &&
                Robot.motorBackRight.isBusy();
    }

    private static void startMotors() {
        Robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
    }

    private static void stopMotors() {
        setPower(0, 0, 0, 0);

        Robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static double getError(double targetAngle) {
        double error;
        error = 0;//targetAngle - Robot.gyro.getIntegratedZValue();
        return AngleUnit.normalizeDegrees(error);
    }

    private static double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public static void move(Direction direction, DistanceUnit unit, double distance) {
        double angle = 0;//Robot.gyro.getIntegratedZValue();
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

    public static void rotate(AngleUnit unit, double angle) {
        angle = unit.normalize(angle);
        angle = unit.toDegrees(angle);
        angle += 0;//Robot.gyro.getIntegratedZValue();

        double error = getError(angle);

        while (error < HEADING_THRESHOLD) {
            double power = TURN_SPEED * Math.signum(error);
            setPower(-power, power, -power, power);
        }

        stopMotors();
    }

    /* TELEOP */
    public static void run() {

        double y = Control.robotMove();//-gamepad1.left_stick_y;
        double x = Control.robotSlide();//gamepad1.right_stick_x;
        double r = Control.robotRotate();//gamepad1.left_stick_x;

        Drive.setNormalisedPower(x + y + r, -x + y - r, -x + y + r, x + y - r);
    }


    public static void setNormalisedPower(@NonNull double... args) {
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
}
