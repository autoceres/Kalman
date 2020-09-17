package kalman.beans;

public class CoordinateUTM extends Coordinate{	
	private int zone;
	private char letter;
	
	public CoordinateUTM(double x, double y) {
		super(x, y);
		// TODO Auto-generated constructor stub
	}
		
	public CoordinateUTM(double x, double y, int zone, char letter) {
		super(x, y);
		this.zone = zone;
		this.letter = letter;
	}

	public int getZone() {
		return zone;
	}
	public void setZone(int zone) {
		this.zone = zone;
	}
	public char getLetter() {
		return letter;
	}
	public void setLetter(char letter) {
		this.letter = letter;
	}

	@Override
	public String toString() {
		return "CoordinateUTM [zone=" + zone + ", letter=" + letter + ", getX()=" + getX() + ", getY()=" + getY() + "]";
	}
	
	
}
