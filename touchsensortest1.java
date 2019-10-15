import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.port.Port;



public class touchsensortest1 {
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	
	public static void main(String[] args) {

		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
					
		//wheel diameter = 5.5
		//middle to middle = 12
		//max angular speed = 340
		//offset = 5585858
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 5.5).offset(-5.7);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.5).offset(5.7);
		
		Chassis chassis = new WheeledChassis(new Wheel[] {wheel1,wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		pilot.setAngularSpeed(50);
		
		Port s1 = ev3brick.getPort("S1");
		TouchSensor touch = new TouchSensor(s1);
		while(true) {
			Delay.msDelay(2);
			if(touch.isPressed()) {
				pilot.stop();
				pilot.travel(-200);
				pilot.rotate(180);;
				pilot.forward();
			}	
		}
		
		}
		
	}
