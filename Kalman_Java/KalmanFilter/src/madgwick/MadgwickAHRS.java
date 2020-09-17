package madgwick;

import jeigen.DenseMatrix;
import static jeigen.Shortcuts.*;

/**
 * MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms in
 * Java.
 * 
 */

public class MadgwickAHRS {

	private double samplePeriod;
	private double beta;
	private DenseMatrix quaternion;	

	/**
	 * Gets the sample period.
	 * 
	 * @return Sample Period
	 */
	public double getSamplePeriod() {
		return samplePeriod;
	}

	/**
	 * Sets the sample period.
	 * 
	 * @param samplePeriod
	 *            Sample period
	 */
	public void setSamplePeriod(double samplePeriod) {
		this.samplePeriod = samplePeriod;
	}

	/**
	 * Gets the sample algorithm gain beta.
	 * 
	 * @return Algorithm gain beta
	 */
	public double getBeta() {
		return beta;
	}

	/**
	 * Sets the algorithm gain beta.
	 * 
	 * @param samplePeriod
	 *            Algorithm gain beta
	 */
	public void setBeta(double beta) {
		this.beta = beta;
	}

	/**
	 * Gets the quaternion output.
	 * 
	 * @return Quaternion output
	 */
	public double[] getQuaternion() {
		//return quaternion;
		return new double[] {quaternion.get(0,0), quaternion.get(0,1), quaternion.get(0,2), quaternion.get(0,3)};
	}

	/**
	 * Converts a quaternion to its conjugate.	
	 */
	private DenseMatrix getQuaternionConjugate(DenseMatrix quaternion) {
		DenseMatrix quaternion_conjugate = new DenseMatrix(new double[][] 
																		{{	quaternion.get(0,0), 
																			quaternion.get(0,1)*-1.0d, 
																			quaternion.get(0,2)*-1.0d, 
																			quaternion.get(0,3)*-1.0d 
																		}});
		return quaternion_conjugate;			
	}

	/**
	 * Calculates the quaternion product of quaternion a and b.	 
	 */
	private DenseMatrix getQuaternionProduct(DenseMatrix a, DenseMatrix b) {
		DenseMatrix ab = zeros(1,4);
		
		ab.set(0, 0, a.get(0,0)*b.get(0,0) - a.get(0,1)*b.get(0,1) - a.get(0,2)*b.get(0,2) - a.get(0,3)*b.get(0,3));
	    ab.set(0, 1, a.get(0,0)*b.get(0,1) + a.get(0,1)*b.get(0,0) + a.get(0,2)*b.get(0,3) - a.get(0,3)*b.get(0,2));
	    ab.set(0, 2, a.get(0,0)*b.get(0,2) - a.get(0,1)*b.get(0,3) + a.get(0,2)*b.get(0,0) + a.get(0,3)*b.get(0,1));
	    ab.set(0, 3, a.get(0,0)*b.get(0,3) + a.get(0,1)*b.get(0,2) - a.get(0,2)*b.get(0,1) + a.get(0,3)*b.get(0,0));
				
		return ab;
	}
		
	/**
	 * Initializes a new instance of the {@link MadgwickAHRS} class.
	 * 
	 * @param samplePeriod
	 *            Sample period.
	 */
	public MadgwickAHRS(double samplePeriod) {
		this(samplePeriod, 1d);
	}

	/**
	 * Initializes a new instance of the {@link MadgwickAHRS} class.
	 * 
	 * @param samplePeriod
	 *            Sample period.
	 * @param beta
	 *            Algorithm gain beta.
	 */
	public MadgwickAHRS(double samplePeriod, double beta) {
		this.samplePeriod = samplePeriod;
		this.beta = beta;
		this.quaternion = new DenseMatrix(new double[][] {
															{ 1d, 0d, 0d, 0d }
														 });		
	}

	/**
	 * Algorithm AHRS update method. Requires only gyroscope and accelerometer
	 * data.
	 * <p>
	 * Optimised for minimal arithmetic. <br>
	 * Total ±: 160 <br>
	 * Total *: 172 <br>
	 * Total /: 5 <br>
	 * Total sqrt: 5 <br>
	 * 
	 * @param gx
	 *            Gyroscope x axis measurement in radians/s.
	 * @param gy
	 *            Gyroscope y axis measurement in radians/s.
	 * @param gz
	 *            Gyroscope z axis measurement in radians/s.
	 * @param ax
	 *            Accelerometer x axis measurement in any calibrated units.
	 * @param ay
	 *            Accelerometer y axis measurement in any calibrated units.
	 * @param az
	 *            Accelerometer z axis measurement in any calibrated units.
	 * @param mx
	 *            Magnetometer x axis measurement in any calibrated units.
	 * @param my
	 *            Magnetometer y axis measurement in any calibrated units.
	 * @param mz
	 *            Magnetometer z axis measurement in any calibrated units.
	 */
	public void update(double gx, double gy, double gz, double ax, double ay,
			double az, double mx, double my, double mz) {
		
		DenseMatrix q = this.quaternion; // short name local variable for readability
				
		// readability
		double norm;
		
		// Normalise accelerometer measurement
		norm = (double) Math.sqrt(ax * ax + ay * ay + az * az);		
		
		if (norm == 0d)
			return; // handle NaN
		norm = 1 / norm; // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = (double) Math.sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0d)
			return; // handle NaN
		norm = 1 / norm; // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;
		
		// Reference direction of Earth's magnetic feild
		DenseMatrix q_mag = new DenseMatrix(new double[][] 
														{{	0d, 
															mx, 
															my, 
															mz 
														}});
		
		DenseMatrix h = this.getQuaternionProduct(q, this.getQuaternionProduct(q_mag, this.getQuaternionConjugate(q))); 	
		DenseMatrix b = new DenseMatrix(new double[][] 
														{{	0d, 
															Math.sqrt(h.get(0,1)*h.get(0,1) + h.get(0,2)*h.get(0,2)),   
															0, 
															h.get(0,3)
														}});
		
		// Gradient decent algorithm corrective step
		DenseMatrix F = new DenseMatrix(new double[][]{
														{2*(q.get(0,1)*q.get(0,3) - q.get(0,0)*q.get(0,2)) - ax},
														{2*(q.get(0,0)*q.get(0,1) + q.get(0,2)*q.get(0,3)) - ay},
														{2*(0.5d - Math.pow(q.get(0,1),2) - Math.pow(q.get(0,2),2)) - az},
														{2*b.get(0,1)*(0.5d - Math.pow(q.get(0,2),2) - Math.pow(q.get(0,3),2)) + 2*b.get(0,3)*(q.get(0,1)*q.get(0,3) - q.get(0,0)*q.get(0,2)) - mx},
														{2*b.get(0,1)*(q.get(0,1)*q.get(0,2) - q.get(0,0)*q.get(0,3)) + 2*b.get(0,3)*(q.get(0,0)*q.get(0,1) + q.get(0,2)*q.get(0,3)) - my},
														{2*b.get(0,1)*(q.get(0,0)*q.get(0,2) + q.get(0,1)*q.get(0,3)) + 2*b.get(0,3)*(0.5d - Math.pow(q.get(0,1),2) - Math.pow(q.get(0,2),2)) - mz}
											 		  });
		
		DenseMatrix J = new DenseMatrix(new double[][]{
														{-2*q.get(0,2), 2*q.get(0,3), -2*q.get(0,0), 2*q.get(0,1)},
														{2*q.get(0,1), 2*q.get(0,0), 2*q.get(0,3), 2*q.get(0,2)},
														{0, -4*q.get(0,1), -4*q.get(0,2), 0},
														{-2*b.get(0,3)*q.get(0,2), 2*b.get(0,3)*q.get(0,3), -4*b.get(0,1)*q.get(0,2) - 2*b.get(0,3)*q.get(0,0), -4*b.get(0,1)*q.get(0,3) + 2*b.get(0,3)*q.get(0,1)},
														{-2*b.get(0,1)*q.get(0,3) + 2*b.get(0,3)*q.get(0,1), 2*b.get(0,1)*q.get(0,2) + 2*b.get(0,3)*q.get(0,0),	2*b.get(0,1)*q.get(0,1) + 2*b.get(0,3)*q.get(0,3), -2*b.get(0,1)*q.get(0,0)+2*b.get(0,3)*q.get(0,2)},
														{2*b.get(0,1)*q.get(0,2), 2*b.get(0,1)*q.get(0,3) - 4*b.get(0,3)*q.get(0,1), 2*b.get(0,1)*q.get(0,0) - 4*b.get(0,3)*q.get(0,2), 2*b.get(0,1)*q.get(0,1)}
													});		
		
		DenseMatrix step = J.t().mmul(F); 
		double norm_step = 1d / Math.sqrt(step.get(0, 0)*step.get(0, 0) + step.get(1, 0)*step.get(1, 0) + step.get(2, 0)*step.get(2, 0) + step.get(3, 0)*step.get(3, 0)); // normalise step magnitude
		
		step = step.mul(norm_step);		
		
		// Compute rate of change of quaternion
		DenseMatrix q_gyro  = new DenseMatrix(new double[][] 
															{{	0d, 
																gx, 
																gy, 
																gz 
															}}); 
		DenseMatrix qDot = this.getQuaternionProduct(q, q_gyro).mul(0.5d).sub(step.t().mul(this.beta));		
		
		// Integrate to yield quaternion
		q = q.add(qDot.mul(this.samplePeriod));
		Double norm_q = 1d / Math.sqrt(q.get(0, 0)*q.get(0, 0) + q.get(0, 1)*q.get(0, 1) + q.get(0, 2)*q.get(0, 2) + q.get(0, 3)*q.get(0, 3)); // normalise quaternion
		this.quaternion = q.mul(norm_q);		
	}

	/**
	 * Algorithm IMU update method. Requires only gyroscope and accelerometer
	 * data.
	 * <p>
	 * Optimised for minimal arithmetic. <br>
	 * Total ±: 45 <br>
	 * Total *: 85 <br>
	 * Total /: 3 <br>
	 * Total sqrt: 3
	 * 
	 * @param gx
	 *            Gyroscope x axis measurement in radians/s.
	 * @param gy
	 *            Gyroscope y axis measurement in radians/s.
	 * @param gz
	 *            Gyroscope z axis measurement in radians/s.
	 * @param ax
	 *            Accelerometer x axis measurement in any calibrated units.
	 * @param ay
	 *            Accelerometer y axis measurement in any calibrated units.
	 * @param az
	 *            Accelerometer z axis measurement in any calibrated units.
	 */
	public void updateIMU(double gx, double gy, double gz, double ax, double ay,
			double az) {
		
		DenseMatrix q = this.quaternion; // short name local variable for readability
		
		// readability
		double norm;
		
		// Normalise accelerometer measurement
		norm = (double) Math.sqrt(ax * ax + ay * ay + az * az);		
		
		if (norm == 0d)
			return; // handle NaN
		norm = 1 / norm; // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Gradient decent algorithm corrective step
		DenseMatrix F = new DenseMatrix(new double[][]{
														{2*(q.get(0,1)*q.get(0,3) - q.get(0,0)*q.get(0,2)) - ax},
														{2*(q.get(0,0)*q.get(0,1) + q.get(0,2)*q.get(0,3)) - ay},
														{2*(0.5d - Math.pow(q.get(0,1),2) - Math.pow(q.get(0,2),2)) - az}														
											 		  });
		
		DenseMatrix J = new DenseMatrix(new double[][]{
														{-2*q.get(0,2), 2*q.get(0,3), -2*q.get(0,0), 2*q.get(0,1)},
														{2*q.get(0,1), 2*q.get(0,0), 2*q.get(0,3), 2*q.get(0,2)},
														{0, -4*q.get(0,1), -4*q.get(0,2), 0}
													   });

		DenseMatrix step = J.t().mmul(F); 
		double norm_step = 1d / Math.sqrt(step.get(0, 0)*step.get(0, 0) + step.get(1, 0)*step.get(1, 0) + step.get(2, 0)*step.get(2, 0) + step.get(3, 0)*step.get(3, 0)); // normalise step magnitude
		
		step = step.mul(norm_step);
		
		// Compute rate of change of quaternion
		DenseMatrix q_gyro  = new DenseMatrix(new double[][] 
															{{	0d, 
																gx, 
																gy, 
																gz 
															}}); 
		
		DenseMatrix qDot = this.getQuaternionProduct(q, q_gyro).mul(0.5d).sub(step.t().mul(this.beta));		
		
		// Integrate to yield quaternion
		q = q.add(qDot.mul(this.samplePeriod));		
		Double norm_q = 1d / Math.sqrt(q.get(0, 0)*q.get(0, 0) + q.get(0, 1)*q.get(0, 1) + q.get(0, 2)*q.get(0, 2) + q.get(0, 3)*q.get(0, 3)); // normalise quaternion		
		this.quaternion = q.mul(norm_q);		
	}
}
