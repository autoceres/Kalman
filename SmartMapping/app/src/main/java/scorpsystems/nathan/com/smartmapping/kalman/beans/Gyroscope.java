package scorpsystems.nathan.com.smartmapping.kalman.beans;

public class Gyroscope {
	private double x;
	private double y;
	private double z;
		
	public Gyroscope() {
		super();
		x = 0;
		y = 0;
		z = 0;
	}

	public Gyroscope(double x, double y, double z) {
		super();
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public double getX() {
		return x;
	}
	public void setX(double x) {
		this.x = x;
	}
	public double getY() {
		return y;
	}
	public void setY(double y) {
		this.y = y;
	}
	public double getZ() {
		return z;
	}
	public void setZ(double z) {
		this.z = z;
	}

	@Override
	public String toString() {
		return "Gyroscope [x=" + x + ", y=" + y + ", z=" + z + "]";
	}		
}
