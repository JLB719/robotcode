import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.ColorDetector;
import lejos.robotics.ColorIdentifier;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class ColourSensor {
		public static void main(String[] args) {
			EV3 ev3brick = (EV3) BrickFinder.getLocal();
			
			Keys buttons = ev3brick.getKeys();
			
			TextLCD lcddisplay = ev3brick.getTextLCD(); 
			
			EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S1);
			sensor.setFloodlight(false);
			LCD.drawString("Init",  2,  2);
			LCD.setAutoRefresh(false);
			SampleProvider brightnessSensorMode = sensor.getRGBMode();
			float[] sample = new float[brightnessSensorMode.sampleSize()];
			buttons.waitForAnyPress();
			
			while(buttons.getButtons() != Keys.ID_ESCAPE) {
				brightnessSensorMode.fetchSample(sample, 0);
				LCD.clear();
				System.out.println("R: " + sample[0] +" G: " + sample[1] +" B: " + sample[2] );
				LCD.drawString("R: " + sample[0] , 1, 1);
				LCD.drawString("G: " + sample[1] , 1, 2);
				LCD.drawString("B: " + sample[2] , 1, 3);
				Delay.msDelay(50);
			}
	}
}
