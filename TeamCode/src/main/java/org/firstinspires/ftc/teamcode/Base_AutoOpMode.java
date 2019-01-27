package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.Brickbot;
import org.firstinspires.ftc.teamcode.robot.Drive;

@SuppressWarnings("all")

@Autonomous(name="Brickbot: BaseAutoOpMode (v0.1)", group="Linear Opmode")
@Disabled
public class Base_AutoOpMode extends AutonomousOpMode {
	private Brickbot robot = Brickbot.getInstance();
	private Drive drive = new Drive();

	@Override
	public void runOpMode() {
		super.runOpMode();

		//Code goes here
	}
}
