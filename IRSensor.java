/*import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;

public class IRSensor {
	public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3brick.getKeys();
		Port portS4 = ev3brick.getPort("S4");
		EV3IRSensor IRSensor = new EV3IRSensor(portS4);
		TextLCD lcddisplay = ev3brick.getTextLCD();
		
		SampleProvider sonicdistance = IRSensor.getDistanceMode();
		float[] sample = new float[average.sampleSize];
	}
}
*/