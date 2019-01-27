package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

@SuppressWarnings("All")

public class Lift {
    private Brickbot robot = Brickbot.getInstance();

    private static final double COUNTS_PER_MOTOR_REV    = 1120.0d;       // Modify according to the drive motors
    private static final double LIFT_GEAR_REDUCTION    = 3.0d;         // Speed reduction
    private static final double COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION / Math.PI);
    private static final double LIFT_SPEED  = 1.0d;

    public Lift() {

    }

    public enum Direction{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public void teleLift(Gamepad gamepad) {
        boolean y= gamepad.y;
        boolean x= gamepad.x;

        robot.limitSwitchSus.setMode(DigitalChannel.Mode.INPUT);
        robot.limitSwitchJos.setMode(DigitalChannel.Mode.INPUT);

        if (y)
            while (robot.limitSwitchSus.getState())
                robot.motorLift.setPower(LIFT_SPEED);

        if (x)
            while (robot.limitSwitchJos.getState())
               robot.motorLift.setPower(-LIFT_SPEED);
    }
}
