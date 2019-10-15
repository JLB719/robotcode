lejos.hardware.port.Port portS4 = ev3brick.getPort("S4");
			EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(portS4);
			SampleProvider ultrasonicdistance = ultrasonicSensor.getDistanceMode();
			SampleProvider averageUltra = new MeanFilter(ultrasonicdistance, 5);
			float[] ultrasample = new float[averageUltra.sampleSize()];
			
			//Distance IR
			lejos.hardware.port.Port portS1 = ev3brick.getPort("S1");
			EV3IRSensor IRSensor = new EV3IRSensor(portS1);
			SampleProvider IRdistance = IRSensor.getDistanceMode();
			SampleProvider averageIR = new MeanFilter(IRdistance, 5);
			float[] IRsample = new float[averageIR.sampleSize()];
			
			IRdistance.fetchSample(IRsample, 0);
				ultrasonicdistance.fetchSample(ultrasample, 0);
				
				if (ultrasample[0] < 10) {
					pilot.rotate(90);
					pilot.travel(10);
					pilot.rotate(-90);
					while(IRsample[0] < 20) {
						pilot.travel(4);
					}
					pilot.rotate(-90);
					pilot.travel(10);
					pilot.rotate(90);
			}