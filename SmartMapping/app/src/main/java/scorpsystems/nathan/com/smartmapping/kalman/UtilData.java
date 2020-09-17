package scorpsystems.nathan.com.smartmapping.kalman;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import scorpsystems.nathan.com.smartmapping.kalman.beans.Accelerometer;
import scorpsystems.nathan.com.smartmapping.kalman.beans.GPS;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gravity;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gyroscope;
import scorpsystems.nathan.com.smartmapping.kalman.beans.IMU;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Magnetometer;

public class UtilData {
	public static ArrayList<Double> csvToArrayDouble(String path) throws IOException {		
		// Read csv files							
		String line = "";
        String cvsSplitBy = " ";
        ArrayList<Double> data = new ArrayList<>();
        
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {

            while ((line = br.readLine()) != null) {            	
            	data.add(Double.parseDouble(line));
            	// use comma as separator                
            	//String[] country = line.split(cvsSplitBy);
                //System.out.println("Country [code= " + country[4] + " , name=" + country[5] + "]");
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
		        
		// TODO Auto-generated method stub
		return data;
	}
	
	// Load datas
	public static ArrayList<GPS> loadDataGPS(ArrayList<Double> latitude, ArrayList<Double> longitude){
		ArrayList<GPS> gps = new ArrayList<GPS>();
		
		for(int x=0; x<latitude.size();x++) {
			GPS value = new	GPS();
			value.setLatitude(latitude.get(x).doubleValue());
			value.setLongitude(longitude.get(x).doubleValue());
			gps.add(value);
		}
		
		return gps;
	}
	
	public static ArrayList<IMU> loadDataIMU(ArrayList<Double> accX, ArrayList<Double> accY, ArrayList<Double> accZ,
											 ArrayList<Double> magX, ArrayList<Double> magY, ArrayList<Double> magZ,
											 ArrayList<Double> gravX, ArrayList<Double> gravY, ArrayList<Double> gravZ,
											 ArrayList<Double> gyroX, ArrayList<Double> gyroY, ArrayList<Double> gyroZ){
		
	ArrayList<IMU> imu = new ArrayList<IMU>();		
		for(int x=0; x<accX.size();x++) {
			IMU value = new	IMU();
			
			Accelerometer acc = new Accelerometer(accX.get(x).doubleValue(), accY.get(x).doubleValue(), accZ.get(x).doubleValue());
			Magnetometer mag = new Magnetometer(magX.get(x).doubleValue(), magY.get(x).doubleValue(), magZ.get(x).doubleValue());
			Gravity grav = new Gravity(gravX.get(x).doubleValue(), gravY.get(x).doubleValue(), gravZ.get(x).doubleValue());
			Gyroscope gyro = new Gyroscope(gyroX.get(x).doubleValue(), gyroY.get(x).doubleValue(), gyroZ.get(x).doubleValue());
			
			value.setAccelerometer(acc);
			value.setMagnetometer(mag);
			value.setGravity(grav);
			value.setGyroscope(gyro);
									
			imu.add(value);
		}
		
		return imu;
	}	
}
