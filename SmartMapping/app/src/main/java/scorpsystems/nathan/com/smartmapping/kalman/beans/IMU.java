package scorpsystems.nathan.com.smartmapping.kalman.beans;

public class IMU {
	private Accelerometer accelerometer;
	private Magnetometer magnetometer;
	private Gravity gravity;
	private Gyroscope gyroscope;
		
	public IMU() {
		super();
	}

	public IMU(Accelerometer accelerometer, Magnetometer magnetometer, Gravity gravity, Gyroscope gyroscope) {
		super();
		this.accelerometer = accelerometer;
		this.magnetometer = magnetometer;
		this.gravity = gravity;
		this.gyroscope = gyroscope;
	}

	public Accelerometer getAccelerometer() {
		return accelerometer;
	}

	public void setAccelerometer(Accelerometer accelerometer) {
		this.accelerometer = accelerometer;
	}

	public Magnetometer getMagnetometer() {
		return magnetometer;
	}

	public void setMagnetometer(Magnetometer magnetometer) {
		this.magnetometer = magnetometer;
	}

	public Gravity getGravity() {
		return gravity;
	}

	public void setGravity(Gravity gravity) {
		this.gravity = gravity;
	}

	public Gyroscope getGyroscope() {
		return gyroscope;
	}

	public void setGyroscope(Gyroscope gyroscope) {
		this.gyroscope = gyroscope;
	}

	@Override
	public String toString() {
		return "IMU [accelerometer=" + accelerometer + ", magnetometer=" + magnetometer + ", gravity=" + gravity
				+ ", gyroscope=" + gyroscope + "]";
	}				
}
