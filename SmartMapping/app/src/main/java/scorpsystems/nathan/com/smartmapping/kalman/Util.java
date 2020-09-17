package scorpsystems.nathan.com.smartmapping.kalman;

import java.util.ArrayList;

import jeigen.DenseMatrix;
import static jeigen.Shortcuts.*;

import scorpsystems.nathan.com.smartmapping.dcm.UtilMatrix;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Accelerometer;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Coordinate;
import scorpsystems.nathan.com.smartmapping.kalman.beans.CoordinateUTM;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gyroscope;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Magnetometer;

public class Util {
	static final double g = 9.81d; //gravedade
	
	// Calculos
	// Para conversiones de posiciones geogr�ficas
	public static Coordinate convertMeterToGraus(Coordinate coordinate) {
		// Convertir a UTM
		CoordinateUTM utm = Util.convertDegToUTM(coordinate);

		// Calcula el gano en direci�n al eje X
		// Que significa el valor constante
		double new_x = utm.getX() + 5000d;
		Coordinate new_coordinate = Util
				.convertUTMToDeg(new CoordinateUTM(new_x, utm.getY(), utm.getZone(), utm.getLetter()));

		double difLon = coordinate.getY() - new_coordinate.getY();
		double m2gX = difLon / 5000d;
		double new_y = utm.getY() + 5000d;
		new_coordinate = Util.convertUTMToDeg(new CoordinateUTM(utm.getX(), new_y, utm.getZone(), utm.getLetter()));
		double difLat = coordinate.getX() - new_coordinate.getX();
		double m2gY = difLat / 5000d;

		Coordinate deg = new Coordinate(m2gX, m2gY);
		return deg;
	}

	public static Coordinate convertUTMToDeg(CoordinateUTM utm) {
		// De donde vienen los valores constantes
		// Validar Zona
		if (utm.getLetter() > 'X' || utm.getLetter() < 'C') {
			// Lanzar una expeci�n
			throw new IllegalArgumentException("UTM Zone not valid!");
		}

		// Hemisferio
		String hemis = "";
		if (utm.getLetter() > 'M') {
			hemis = "N";
		} else {
			hemis = "S";
		}

		double x = utm.getX();
		double y = utm.getY();

		// Averiguar que significan los valores siguientes:
		double sa = 6378137.000000;
		double sb = 6356752.314245;

		// Estos valores pueden ser constantes
		double e2 = Math.sqrt(Math.pow(sa, 2) - Math.pow(sb, 2)) / sb;
		double e2cuadrada = Math.pow(e2, 2);
		double c = Math.pow(sa, 2) / sb;

		double X = x - 500000d;

		double Y = y;
		if (hemis.equalsIgnoreCase("S")) {
			Y = y - 10000000d;
		}

		double S = ((double) utm.getZone() * 6d) - 183d;
		double lat = Y / (6366197.724d * 0.9996d);
		double v = (c / Math.sqrt((1d + (e2cuadrada * Math.pow(Math.cos(lat), 2))))) * 0.9996d;

		double a = X / v;
		double a1 = Math.sin(2d * lat);
		double a2 = a1 * Math.pow(Math.cos(lat), 2);
		double j2 = lat + (a1 / 2d);
		double j4 = (3 * j2 + a2) / 4;
		double j6 = (5 * j4 + (a2 * Math.pow(Math.cos(lat), 2))) / 3;
		double alfa = (3d / 4d) * e2cuadrada;
		double beta = (5d / 3d) * Math.pow(alfa, 2);
		double gama = (35d / 27d) * Math.pow(alfa, 3);
		double Bm = 0.9996d * c * (lat - alfa * j2 + beta * j4 - gama * j6);
		double b = (Y - Bm) / v;
		double Epsi = (e2cuadrada * Math.pow(a, 2)) / 2d * Math.pow(Math.cos(lat), 2);
		double Eps = a * (1d - Epsi / 3d);
		double nab = (b * (1d - Epsi)) + lat;
		double senoheps = (Math.exp(Eps) - Math.exp(Eps * -1)) / 2d;
		double Delt = Math.atan(senoheps / (Math.cos(nab)));
		double TaO = Math.atan(Math.cos(Delt) * Math.tan(nab));
		double longitude = (Delt * (180d / Math.PI)) + S;
		double latitude = (lat + (1d + e2cuadrada * Math.pow(Math.cos(lat), 2)
				- (3d / 2d) * e2cuadrada * Math.sin(lat) * Math.cos(lat) * (TaO - lat)) * (TaO - lat))
				* (180 / Math.PI);

		Coordinate deg = new Coordinate(latitude, longitude);
		return deg;
	}

	public static CoordinateUTM convertDegToUTM(Coordinate deg) {
		char letter = 'X';

		// Averiguar que significan los valores siguientes:
		double sa = 6378137.000000;
		double sb = 6356752.314245;

		// Estos valores pueden ser constantes
		double e2 = Math.sqrt(Math.pow(sa, 2) - Math.pow(sb, 2)) / sb;
		double e2cuadrada = Math.pow(e2, 2);
		double c = Math.pow(sa, 2) / sb;

		double la = deg.getX() * (Math.PI / 180);
		double lo = deg.getY() * (Math.PI / 180);

		long Huso = (long) ((deg.getY() / 6) + 31);
		long S = Huso * 6 - 183;

		double deltaS = lo - (((double) S) * (Math.PI / 180));

		if (deg.getX() < -72)
			letter = 'C';
		else if (deg.getX() < -64)
			letter = 'D';
		else if (deg.getX() < -56)
			letter = 'E';
		else if (deg.getX() < -48)
			letter = 'F';
		else if (deg.getX() < -40)
			letter = 'G';
		else if (deg.getX() < -32)
			letter = 'H';
		else if (deg.getX() < -24)
			letter = 'J';
		else if (deg.getX() < -16)
			letter = 'K';
		else if (deg.getX() < -8)
			letter = 'L';
		else if (deg.getX() < 0)
			letter = 'M';
		else if (deg.getX() < 8)
			letter = 'N';
		else if (deg.getX() < 16)
			letter = 'P';
		else if (deg.getX() < 24)
			letter = 'Q';
		else if (deg.getX() < 32)
			letter = 'R';
		else if (deg.getX() < 40)
			letter = 'S';
		else if (deg.getX() < 48)
			letter = 'T';
		else if (deg.getX() < 56)
			letter = 'U';
		else if (deg.getX() < 64)
			letter = 'V';
		else if (deg.getX() < 72)
			letter = 'W';
		else
			letter = 'X';

		double a = Math.cos(la) * Math.sin(deltaS);
		double epsilon = 0.5d * Math.log((1d + a) / (1d - a));
		double nu = Math.atan(Math.tan(la) / Math.cos(deltaS)) - la;
		// De donde sale ese valor por defecto
		double v = (c / Math.sqrt(1d + (e2cuadrada * Math.pow(Math.cos(la), 2)))) * 0.9996;
		double ta = (e2cuadrada / 2d) * Math.pow(epsilon, 2) * Math.pow((Math.cos(la)), 2);
		double a1 = Math.sin(2d * la);
		double a2 = a1 * Math.pow(Math.cos(la), 2);
		double j2 = la + (a1 / 2d);
		double j4 = ((3d * j2) + a2) / 4d;
		double j6 = ((5d * j4) + (a2 * Math.pow(Math.cos(la), 2))) / 3d;
		double alfa = (3d / 4d) * e2cuadrada;
		double beta = (5d / 3d) * Math.pow(alfa, 2);
		double gama = (35d / 27d) * Math.pow(alfa, 3);
		// Que significa el valor por defecto
		double Bm = 0.9996 * c * (la - alfa * j2 + beta * j4 - gama * j6);
		// Que significa el valor por defecto
		double xx = epsilon * v * (1d + (ta / 3d)) + 500000d;
		double yy = nu * v * (1d + ta) + Bm;

		// Que significa esta parte
		if (yy < 0)
			yy = 9999999 + yy;

		CoordinateUTM utm = new CoordinateUTM(xx, yy, (int) Huso, letter);

		return utm;
	}

	// Calcular Varianza
	public static double getVariance(ArrayList<Double> values) {
		double variance = 0.0d;

		// Obtener la media
		double mean = Util.getMean(values);

		for (int x = 0; x < values.size(); x++) {
			variance = variance + Math.pow(values.get(x) - mean, 2) / values.size();
		}

		return variance;
	}

	// Calcular Media
	public static double getMean(ArrayList<Double> values) {
		double mean = 0.0d;

		for (int x = 0; x < values.size(); x++) {
			mean = mean + values.get(x);
		}
		mean = mean / values.size();

		return mean;
	}

	// Restar arrays
	public static ArrayList<Double> subtractArraysValues(ArrayList<Double> array1, ArrayList<Double> array2) {
		ArrayList<Double> subtract = new ArrayList<Double>();

		for (int x = 0; x < array1.size(); x++) {
			Double difference = array1.get(x) - array2.get(x);
			subtract.add(difference);
		}

		return subtract;
	}

	// Generar matriz de rotaci�n	
	// Version 1	
	/*
	public static double[][] getRotationMatrix(Accelerometer acc, Magnetometer mag, Gyroscope gyro) {
		double [][] RM = { {0, 0, 0, 0, 0, 0, 0, 0, 0} };

		// Que signfica esto
		double Ax = acc.getX();
		double Ay = acc.getY();
		double Az = acc.getZ();
		
		double Ex = mag.getX();
		double Ey = mag.getY();
		double Ez = mag.getZ();
		
		double Hx = 0.0d;
		double Hy = 0.0d;
		double Hz = 0.0d;
		
		double Mx = 0.0d;
		double My = 0.0d;
		double Mz = 0.0d;
		
		double normsqA = Math.pow(Ax, 2) + Math.pow(Ay, 2) + Math.pow(Az, 2);		
		
		// Que significa la constante
		double freeFallGravitySquared = 0.01d * Math.pow(g, 2);
				
		if (!(normsqA < freeFallGravitySquared)) {
			Hx = Ey * Az - Ez * Ay;
            Hy = Ez * Ax - Ex * Az;
            Hz = Ex * Ay - Ey * Ax;
            
            double normH = Math.sqrt(Math.pow(Hx, 2) + Math.pow(Hy, 2) + Math.pow(Hz, 2));
            
            // Porque es esto
            if (!(normH < 0.1d)) {
            	
            	double invH = 1.0d / normH;
                
            	Hx = Hx * invH;
                Hy = Hy * invH;
                Hz = Hz * invH;
                
                double invA = 1.0d / Math.sqrt(Math.pow(Ax, 2) + Math.pow(Ay, 2) + Math.pow(Az, 2));
                
                Ax = Ax * invA;
                Ay = Ay * invA;
                Az = Az * invA;
                
                Mx = Ay * Hz - Az * Hy;
                My = Az * Hx - Ax * Hz;
                Mz = Ax * Hy - Ay * Hx;
                
                // Asignar valores
                RM[0][0] = Hx;     RM[0][1] = Hy;     RM[0][2] = Hz;
                RM[0][3] = Mx;     RM[0][4] = My;     RM[0][5] = Mz;
                RM[0][6] = Ax;     RM[0][7] = Ay;     RM[0][8] = Az;
            }           
		}
			
		return RM;
	}
	*/
	
	public static DenseMatrix getRotationMatrix(Accelerometer acc, Magnetometer mag, Gyroscope gyro) {
		DenseMatrix RM = zeros(1,9);

		// Que signfica esto
		double Ax = acc.getX();
		double Ay = acc.getY();
		double Az = acc.getZ();
		
		double Ex = mag.getX();
		double Ey = mag.getY();
		double Ez = mag.getZ();
		
		double Hx = 0.0d;
		double Hy = 0.0d;
		double Hz = 0.0d;
		
		double Mx = 0.0d;
		double My = 0.0d;
		double Mz = 0.0d;
		
		double normsqA = Math.pow(Ax, 2) + Math.pow(Ay, 2) + Math.pow(Az, 2);		
		
		// Que significa la constante
		double freeFallGravitySquared = 0.01d * Math.pow(g, 2);
				
		if (!(normsqA < freeFallGravitySquared)) {
			Hx = Ey * Az - Ez * Ay;
            Hy = Ez * Ax - Ex * Az;
            Hz = Ex * Ay - Ey * Ax;
            
            double normH = Math.sqrt(Math.pow(Hx, 2) + Math.pow(Hy, 2) + Math.pow(Hz, 2));
            
            // Porque es esto
            if (!(normH < 0.1d)) {
            	
            	double invH = 1.0d / normH;
                
            	Hx = Hx * invH;
                Hy = Hy * invH;
                Hz = Hz * invH;
                
                double invA = 1.0d / Math.sqrt(Math.pow(Ax, 2) + Math.pow(Ay, 2) + Math.pow(Az, 2));
                
                Ax = Ax * invA;
                Ay = Ay * invA;
                Az = Az * invA;
                
                Mx = Ay * Hz - Az * Hy;
                My = Az * Hx - Ax * Hz;
                Mz = Ax * Hy - Ay * Hx;
                
                // Asignar valores
                RM.set(0,0,Hx);    RM.set(0,1,Hy);    RM.set(0,2,Hz);
                RM.set(0,3,Mx);    RM.set(0,4,My);    RM.set(0,5,Mz);
                RM.set(0,6,Ax);    RM.set(0,7,Ay);    RM.set(0,8,Az);
            }           
		}
			
		return RM;
	}
	
	// Obtener Orientaci�n
	// Versi�n 1
	/*
	public static double[] getOrientation(double [][] RM) {
		double [] vo = {0, 0, 0};
		
		vo[0] = Math.atan2(RM[0][1], RM[0][4]);
		vo[1] = Math.asin(RM[0][7] * -1.0d);
		vo[2] = Math.atan2(RM[0][6] * -1.0d, RM[0][8]);
		
		return vo;
	}
	*/
	
	public static DenseMatrix getOrientation(DenseMatrix RM) {
		DenseMatrix vo = zeros(1,3); 
				
		vo.set(0,0,Math.atan2(RM.get(0,1), RM.get(0,4)));
		vo.set(0,1,Math.asin(RM.get(0,7) * -1.0d));
		vo.set(0,2,Math.atan2(RM.get(0,6) * -1.0d, RM.get(0,8)));
		
		return vo;
	}
	
	// Quaternion to Euler
	public static double[] getQuaternionConjugate(double[] quaternion) {
		double[] quaternion_conjugate = new double[] { quaternion[0], quaternion[1]*-1.0d, quaternion[2]*-1.0d, quaternion[3]*-1.0d };		
		return quaternion_conjugate;
	}
	
	

	
	public static double[] quaternionToEuler(double[] quaternion)
    {		
		double[] euler = new double[] { 0d, 0d, 0d };
		
		// Copiado de Matlab
		double r_1_1 = 2.0d * quaternion[0] * quaternion[0] - 1.0d + 2.0d * quaternion[1] * quaternion[1];		        
		double r_2_1 = 2.0d * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);		
	    double r_3_1 = 2.0d * (quaternion[1] * quaternion[3] + quaternion[0] * quaternion[2]);
	    double r_3_2 = 2.0d * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);
	    double r_3_3 = 2.0d * quaternion[0] * quaternion[0] - 1 + 2.0d * quaternion[3] * quaternion[3];
	
		// roll/phi
        euler[0] = Math.atan2(r_3_2, r_3_3);
        
        // pitch/theta
        euler[1] = -1.0d * Math.atan(r_3_1/Math.sqrt(1.0d - r_3_1 * r_3_1));
        
        // yaw/psi
        euler[2] = Math.atan2(r_2_1, r_1_1);
        
        return euler;
    }
	
	// Convertir euler a cuaternion
	public static double[] eulerToQuaternion(double[] euler)
	{
		//Yaw     Pitch   Roll
		//Heading Pitch   Bank
		//Y       X       Z		
		double[] quaternion = new double[] { 0d, 0d, 0d, 0d };
	
		double roll = euler[0];
		double pitch = euler[1];
		double yaw = euler[2];
				
		// Assuming the angles are in radians.
	    double c1 = Math.cos(yaw/2.0d);
	    double c2 = Math.cos(pitch/2.0d);
	    double c3 = Math.cos(roll/2.0d);
	    
	    double s1 = Math.sin(yaw/2.0d);	    
	    double s2 = Math.sin(pitch/2.0d);	    
	    double s3 = Math.sin(roll/2.0d);
	    	    	    	    	    	    
	    quaternion[0] = c1 * c2 * c3 + s1 * s2 * s3;
	    quaternion[1] = c1 * c2 * s3 - s1 * s2 * c3;
	    quaternion[2] = c1 * s2 * c3 + s1 * c2 * s3;
	    quaternion[3] = s1 * c2 * c3 - c1 * s2 * s3;
	    
	    return quaternion;
	}
	
	public static DenseMatrix getDirectCosineMatrix(double roll, double pitch, double yaw){
		DenseMatrix dcm = new DenseMatrix (new double [][]
			{
					{Math.cos(yaw)*Math.cos(pitch), Math.cos(yaw)*Math.sin(pitch)*Math.sin(roll)-Math.sin(yaw)*Math.cos(roll), Math.cos(yaw)*Math.sin(pitch)*Math.cos(roll)-Math.sin(yaw)*Math.sin(roll)},
					{Math.sin(yaw)*Math.cos(pitch), Math.sin(yaw)*Math.sin(pitch)*Math.sin(roll)-Math.cos(yaw)*Math.cos(roll)*Math.cos(pitch), Math.sin(yaw)*Math.sin(pitch)*Math.cos(roll)-Math.cos(yaw)*Math.sin(roll)},
					{-1.0d*Math.sin(pitch), Math.cos(pitch)*Math.sin(roll), Math.cos(pitch)*Math.cos(roll)}					
			}); 
		
		return dcm;
	}
	
	public static DenseMatrix getAngularVelocityMatrix(double wx, double wy, double wz)
	{
		DenseMatrix avm = new DenseMatrix (new double [][] 
			{
					{0,   -1.0d*wz,    wy},
					{wz,   0,  -1.0d*wx},
					{-1.0d*wy,   wx,     0}
			});
		return avm;
	}
	
	
	public static DenseMatrix invertibleMatrix(DenseMatrix m){
		double [][] _m = new double [m.rows][m.cols];
			
		for(int i = 0; i < m.rows; i++) {
			for(int j = 0; j < m.cols; j++) {
				_m[i][j] = m.get(i, j);
			}
		}
			
		double [][] _m_inv = UtilMatrix.inverse(_m);
		
		// z
		for(int i = 0; i < m.rows; i++) {
			for(int j = 0; j < m.cols; j++) {
				m.set(i, j, _m_inv[i][j]);
			}
		}
			
		return m;
	}
}