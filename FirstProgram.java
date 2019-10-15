import lejos.*;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
public class FirstProgram {
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	public static void main(String[] args) {

		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
		
		buttons.waitForAnyPress();
			
		//wheel diameter = 5.5
		//middle to middle = 12
		//max angular speed = 340
		//offset = 5585858
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 5.45).offset(-6.25);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.425).offset(6.25);
		
		Chassis chassis = new WheeledChassis(new Wheel[] {wheel1,wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		
		
		pilot.setAngularSpeed(50);
		
		
		//for(int i = 0; i < 4; i++) {
//			int count1 = 0; 
//			int count2 = 0;
//			while(count1 < 200 || count2 < 200) {
//				count1 = LEFT_MOTOR.getTachoCount();
//				count2 = RIGHT_MOTOR.getTachoCount();
//			}
			//pilot.travel(90);
			//pilot.stop();
			//pilot.rotate(90, false);

		//}
	
		
	}
}
