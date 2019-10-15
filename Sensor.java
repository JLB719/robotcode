import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.Keys;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
public class Sensor {
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	
	public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
		
		TextLCD lcddisplay = ev3brick.getTextLCD(); 
		
		buttons.waitForAnyPress();
		
		Port portS2 = ev3brick.getPort("S2");
		EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(portS2);
		
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 5.4).offset(-6.25);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.4).offset(6.25);
		
		Chassis chassis = new WheeledChassis(new Wheel[] {wheel1,wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		pilot.setAngularSpeed(50);
		SampleProvider sonicdistance = sonicSensor.getDistanceMode();
		//SampleProvider average = new MeanFilter(sonicdistance, 5);
		
		float[] sample = new float[sonicdistance.sampleSize()];
		pilot.forward();
		while(buttons.getButtons() != Keys.ID_ESCAPE) {
			sonicdistance.fetchSample(sample, 0);
			if(sample[0] <  0.3) {
				pilot.travel(-10);
				pilot.rotate(180);;
				pilot.forward();
			}
			
			try {
				Thread.sleep(5);
				
			} catch (Exception e) {
			}
		}

	
	}
}
