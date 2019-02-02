package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Sample;

@SuppressWarnings("all")
@Autonomous(name="AutoOpMode", group="Linear Opmode")
public class AutoOp extends LinearOpMode {

	@Override
	public void runOpMode() {
		Robot.init(hardwareMap, telemetry);
		Sample.init(hardwareMap);

		Sample.identify();

		waitForStart();

		//Lift.raise();

		//Drive.move(Drive.Direction.BACKWARD, DistanceUnit.CM, 4);

	}
}
