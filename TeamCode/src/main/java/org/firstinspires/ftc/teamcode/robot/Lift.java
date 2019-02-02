package org.firstinspires.ftc.teamcode.robot;

@SuppressWarnings("all")
public class Lift {
	public static void run(){
		if(Control.liftUp() && !Robot.liftMax.isPressed())
			Robot.motorLift.setPower(-1);
		else if(Control.liftDown() && !Robot.liftMin.isPressed())
			Robot.motorLift.setPower(1);
		else
			Robot.motorLift.setPower(0);
	}

	public static void raise(){
		Robot.motorLift.setPower(-1);
		while(!Robot.liftMax.isPressed()) { }
		Robot.motorLift.setPower(0);
	}
}
