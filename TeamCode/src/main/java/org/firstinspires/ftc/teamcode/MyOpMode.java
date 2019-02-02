package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Control;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;

@SuppressWarnings("ALL")

@TeleOp(name="TeleOp", group="Linear Opmode")

public class MyOpMode extends OpMode {

    @Override
    public void init() {
        telemetry.setAutoClear(true);

        Robot.init(hardwareMap, telemetry);
        Control.init(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        Drive.run();
        Arm.run();
        Lift.run();
    }


}
