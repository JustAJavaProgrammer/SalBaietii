package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Brickbot;

@SuppressWarnings("All")

@Autonomous(name="Brickbot: AutoOpMode (v0.1)", group="Linear Opmode")
@Disabled
public class AutonomousOpMode extends LinearOpMode {

	protected Brickbot              robot           = Brickbot.getInstance();

	@Override
	public void runOpMode() {
		//telemetry.setAutoClear(false);
		robot.init(telemetry, hardwareMap);

		waitForStart();

		//Land the robot

		//robot.test();

		robot.sampleGoldOre();
		sleep(5000);
	}
}
