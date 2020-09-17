package dcm;

import jeigen.DenseMatrix;
import static jeigen.Shortcuts.*;

public class DCM_IMU {
	// Public Properties
	public static double _g0 = 9.8189d; // gravitation around Helsinki, Finland (change according to your area)
	public static DenseMatrix _x = new DenseMatrix(new double [][]{{0d}, 
																	  {0d}, 
																	  {1d}, 
																	  {0d}, 
																	  {0d}, 
																	  {0d}
																	  }); 	// States are lowest row of rotation matrix and gyroscope x y and z biases
    												// (C_31, C_32, C_33, w_b1, w_b2, w_b3)
	public static double _q_dcm2 = 0.0025d*0.0025d; // estimated variance of dcm states (gyro variance per second)
	public static double _q_gyro_bias2 = 0.00025d*0.00025d; // very small number to make bias change slowly
	public static double _r_acc2 = 2000d*2000d; // variance of calibrated accelerometer (g-component)
	public static double _r_a2 = 150d*150d; // large variance for some unknown acceleration (acc = a + g)
	public static double _q_dcm2_init = 0.0001d*0.0001d; // initial variance of dcm states (for attitude estimation)
	public static double _q_gyro_bias2_init = 0.00000001d*0.00000001d; // initial variance of bias states (for bias estimator)
	public static DenseMatrix _a = new DenseMatrix(new double [][]{{0d},{0d},{0d}}); // estimated non-gravitational accelerations
	public static double _yaw = 0d; // Yaw angle around z axis (in ZYX convention)
	public static double _pitch = 0d; // Pitch angle around y axis
	public static double _roll = 0d; // Roll angle around x axis
	public static DenseMatrix _P = new DenseMatrix(new double[][]{{0d, 0d, 0d, 0d, 0d, 0d}, // estimate covariance (these are initialized in constructor below)
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d}
									});
	
	public static DenseMatrix _H = new DenseMatrix(new double [][]{{0d, 0d, 0d, 0d, 0d, 0d}, // observation model (static)
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d}			
									});
	
	public static DenseMatrix _Q = new DenseMatrix(new double [][]{{0d, 0d, 0d, 0d, 0d, 0d}, // process noise covariance (static part)
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d},
									{0d, 0d, 0d, 0d, 0d, 0d}
									});
	
	
	// Properties
	double g0;
	DenseMatrix x;
	double q_dcm2;
	double q_gyro_bias2;
	double r_acc2;
	double r_a2;
	double q_dcm2_init;
	double q_gyro_bias2_init;
	DenseMatrix a;
	double yaw;
	double pitch;
	double roll;
	DenseMatrix P;
	DenseMatrix H;
	DenseMatrix Q;
	
	// Additional
	DenseMatrix first_row;
		
	public DCM_IMU() {
		super();
		
		this.g0 = _g0;
		this.x = _x;
		this.q_dcm2 = _q_dcm2;
		this.q_gyro_bias2 = _q_gyro_bias2;
		this.r_acc2 = _r_acc2;
		this.r_a2 = _r_a2;
		this.q_dcm2_init = _q_dcm2_init;
		this.q_gyro_bias2_init = _q_gyro_bias2_init;
		this.a = _a;
		this.yaw = _yaw;
		this.pitch = _pitch;
		this.roll = _roll;
		
		// P
		this.P = _P;
		for(int i = 0; i < 3; i++) {
			this.P.set(i, i, this.q_dcm2_init);
		}
		
		for(int i = 3; i < 6; i++) {
			this.P.set(i, i, this.q_gyro_bias2_init);
		}
				
		// H
		this.H = _H;
		for(int i = 0; i < 3; i++) {
			this.H.set(i, i, this.g0);
		}		
		
		// Q
		this.Q = new DenseMatrix(new double [][]{	{this.q_dcm2*1d, 	0d, 				0d, 				0d, 					0d, 					0d}, // process noise covariance (static part)
													{0d, 				this.q_dcm2*1d, 	0d, 				0d, 					0d, 					0d},
													{0d, 				0d, 				this.q_dcm2*1d, 	0d, 					0d, 					0d},
													{0d, 				0d, 				0d, 				this.q_gyro_bias2*1d, 	0d, 					0d},
													{0d, 				0d, 				0d, 				0d, 					this.q_gyro_bias2*1d, 	0d},
													{0d, 				0d, 				0d, 				0d, 					0d, 					this.q_gyro_bias2*1d}
			});
		
		// Additional
		first_row = new DenseMatrix(new double [][] {{1.0d},{0.0d},{0.0d}});
	}
					
	public void updateIMU(double gx, double gy, double gz, double ax, double ay, double az, double samplePeriod) {
		// Process noise covariance with time dependent noise		
		DenseMatrix Q_ = Q.mul(samplePeriod*samplePeriod);		
		DenseMatrix u = new DenseMatrix(new double [][] {{gx}, {gy}, {gz}}); 
				
		// Rotation operators
		DenseMatrix C3X = new DenseMatrix(new double [][]{	
					{0, -x.get(2,0), x.get(1,0)},
					{x.get(2,0), 0, -x.get(0,0)},
					{-x.get(1,0), -x.get(0,0), 0}
				});
				
		DenseMatrix UX = new DenseMatrix(new double [][] {
							{0, -(u.get(2, 0)-x.get(5,0)), u.get(1, 0)-x.get(4,0)},
							{u.get(2, 0)-x.get(5,0), 0, -u.get(0, 0)+x.get(3,0)},
							{-u.get(1, 0)+x.get(4,0), u.get(0, 0)-x.get(3,0), 0}
						});		
		
		DenseMatrix C3X_sp = C3X.mul(samplePeriod);				
		DenseMatrix UX_sp = UX.mul(samplePeriod);		
		
		// Model generation
		DenseMatrix A = zeros(6,6);
		for(int i=0; i<3; i++) {
			for(int j=3; j<6; j++) {
				A.set(i, j, -C3X_sp.get(i, j-3));
			}
		}
		
		DenseMatrix B = zeros(6,3);
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				B.set(i, j, C3X_sp.get(i, j));
			}
		}
		
		DenseMatrix F = zeros(6,6);		
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				F.set(i, j, -UX_sp.get(i, j));
			}
		}
		
		for(int i=0; i<3; i++) {
			for(int j=3; j<6; j++) {
				F.set(i, j, -C3X_sp.get(i, j-3));
			}
		}
		
		F = F.add(eye(6));		
		
		// Kalman a priori prediction
		DenseMatrix x_predict = zeros(6,1);
		x_predict = x.add(A.mmul(x).add(B.mmul(u)));;
		
		DenseMatrix P_predict = zeros(6,6);
		//P_predict = F.mmul(P.mmul(F.t())).add(Q_);
		P_predict = F.mmul(P).mmul(F.t()).add(Q_);
				
		// measurements/observations (acceleromeres)
		DenseMatrix z = new DenseMatrix(new double [][] {{ax}, {ay}, {az}});
				
		// recompute R using the error between acceleration and the model of g
		// (estimate of the magnitude of a0 in a = a0 + g)
		DenseMatrix a_predict = zeros(3,1);
				
		DenseMatrix x_predict_block = zeros(3,1);
		for(int i=0; i<3; i++) {
			for(int j=0; j<1; j++) {
				x_predict_block.set(i, j, x_predict.get(i, j)*g0);
			}
		}
		
		a_predict = z.sub(x_predict_block);		
		
		double a_len = Math.sqrt(a_predict.get(0,0)*a_predict.get(0,0) + a_predict.get(1,0)*a_predict.get(1,0) + a_predict.get(2,0)*a_predict.get(2,0));		
		
		DenseMatrix R = zeros(3,3);
		R = eye(3).mul(a_len*r_a2 + r_acc2);		
		
		// Kalman innovation
		DenseMatrix y = zeros(3,1);
		y = z.sub(H.mmul(x_predict));
		
		DenseMatrix S = zeros(3,3);
		S = H.mmul(P_predict).mmul(H.t()).add(R);		
		DenseMatrix S_inv = this.invertibleMatrix(S);
				
		// Kalman gain
		DenseMatrix K = zeros(6,3);
		K = P_predict.mmul(H.t()).mmul(S_inv);						
		
		//save previous x to memory
		DenseMatrix x_last = x;
				
		// update a posteriori
		x = x_predict.add(K.mmul(y));
		
		// update a posteriori covariance
		DenseMatrix IKH = zeros(6,6);
		IKH = eye(6).sub(K.mmul(H));		
		P = IKH.mmul(P_predict).mmul(IKH.t()).add(K.mmul(R).mmul(K.t()));		
		
		// normalization of x & P (divide by DCM vector length)
		double d = Math.sqrt(x.get(0,0)*x.get(0,0)) + x.get(1,0)*x.get(1,0) + x.get(2,0)*x.get(2,0);
		DenseMatrix J = zeros(6,6);
		DenseMatrix J_33 = new DenseMatrix(new double [][] {
			{x.get(1,0)*x.get(1,0)+x.get(2,0)*x.get(2,0), -x.get(0,0)*x.get(1, 0), -x.get(0, 0)*x.get(2, 0)}, 
			{-x.get(0,0)*x.get(1,0), x.get(0,0)*x.get(0,0)+x.get(2,0)*x.get(2,0), -x.get(1,0)*x.get(2,0)},
			{-x.get(0,0)*x.get(2,0), -x.get(1,0)*x.get(2,0), x.get(0,0)*x.get(0,0)+x.get(1,0)*x.get(1,0)}
		});

		double d_3 = d*d*d;
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				J.set(i, j, J_33.get(i, j)/d_3);
			}
		}

		for(int i=3; i<6; i++) {
			J.set(i, i, 1);			
		}
		
		// Laplace approximation of normalization function for x to P, J = Jacobian(f,x)					
		P = J.mmul(P).mmul(J.t());		
				
		for(int i=0; i<3; i++) {
			for(int j=0; j<1; j++) {
				x.set(i, j, x.get(i, j)/d);
			}
		}
		
		// compute Euler angles (not exactly a part of the extended Kalman filter)
		// yaw integration through full rotation matrix
		DenseMatrix u_nb = new DenseMatrix(new double [][] {{u.get(0, 0) - x.get(3, 0)},
															{u.get(1, 0) - x.get(4, 0)},
															{u.get(2, 0) - x.get(5, 0)}
															});		
		
		if(true) {
			// C code
			//compute rotation matrix from previous angular and state values
			//double cy = cos(yaw); //old angles (last state before integration)
			//double sy = sin(yaw);
			//double cp = cos(pitch);
			//double sp = sin(pitch);
			//double cr = cos(roll);
			//double sr = sin(roll);

			// compute needed parts of rotation matrix R (angle based version)
			//double R11 = cy*cp;
			//double R12 = cy*sp*sr-sy*cr;
			//double R13 = cy*sp*cr+sy*sr;
			//double R21 = sy*cp;
			//double R22 = sy*sp*sr+cy*cr;
			//double R23 = sy*sp*cr-cy*sr;
			
			// Versión 1
			/*
			double cy = Math.cos(yaw); //old angles (last state before integration)
			double sy = Math.sin(yaw);
			double d_ = Math.sqrt(x_last.get(1,0)*x_last.get(1,0) + x_last.get(2,0)*x_last.get(2,0));
			double d__inv = 1.0d/d;
			
			// compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
			double R11 = cy * d_;
			double R12 = -(x_last.get(2,0)*sy + x_last.get(0,0)*x_last.get(1,0)*cy) * d__inv;
			double R13 = (x_last.get(1,0)*sy - x_last.get(0,0)*x_last.get(2,0)*cy) * d__inv;
			double R21 = sy * d;
			double R22 = (x_last.get(2,0)*cy - x_last.get(0,0)*x_last.get(1,0)*sy) * d__inv;
			double R23 = -(x_last.get(1,0)*cy + x_last.get(0,0)*x_last.get(2,0)*sy) * d__inv;
			*/
			
			double cy = Math.cos(this.yaw); //old angles (last state before integration)
			double sy = Math.sin(this.yaw);
			double cp = Math.cos(this.pitch);
			double sp = Math.sin(this.pitch);
			double cr = Math.cos(this.roll);
			double sr = Math.sin(this.roll);

			// compute needed parts of rotation matrix R
			double R11 = cy*cp;
			double R12 = cy*sp*sr - sy*cr;
			double R13 = cy*sp*cr + sy*sr;
			double R21 = sy*cp;
			double R22 = sy*sp*sr + cy*cr;
			double R23 = sy*sp*cr - cy*sr;
									
			// update needed parts of R for yaw computation
			double R11_new = R11 + samplePeriod*(u_nb.get(2,0)*R12 - u_nb.get(1,0)*R13);			
			double R21_new = R21 + samplePeriod*(u_nb.get(2,0)*R22 - u_nb.get(1,0)*R23);		
			yaw = Math.atan2(R21_new,R11_new);			
		}else { //alternative method estimating the whole rotation matrix
			//integrate full rotation matrix (using first row estimate in memory)
			// oarrua 23/04/2019: sin verificar.
			DenseMatrix x1 = zeros(3,1);
			DenseMatrix x2 = zeros(3,1);
			
			x1 = first_row.add(UX.t().mmul(first_row).mul(samplePeriod)); //rotate x1 by x1 x u_nb
			x2 = C3X.mmul(x1); //second row x2 = (state x x1)
			x2 = x2.div(Math.sqrt(x2.get(0,0)*x2.get(0,0) + x2.get(1,0)*x2.get(1,0) + x2.get(2,0)*x2.get(2,0))); // normalize length of the second row
			x1 = C3X.t().mmul(x2); // recalculate first row x1 = (x2 * state) (ensure perpendicularity)
			first_row = x1.div(Math.sqrt(x1.get(0,0)*x1.get(0,0) + x1.get(1,0)*x1.get(1,0) + x1.get(2,0)*x1.get(2,0))); // normalize length
		}
		
		pitch = Math.asin(-x.get(0, 0));		
		roll = Math.atan2(x.get(1, 0), x.get(2, 0));
				
		// Alternativa para roll
		//roll = Math.asin(x.get(1, 0))/(Math.cos(pitch*(Math.PI/180)));
						
		// save the estimated non-gravitational acceleration
		for(int i=0; i<3; i++) {
			for(int j=0; j<1; j++) {
				a.set(i, j, z.get(i, j) - x.get(i, j)*g0);
			}
		} 		
	}
	
	public double [] getValues(){	
		return new double[] {roll, pitch, yaw};
	}
	
	public double [] getState(){	
		return new double[] {x.get(0, 0),
								x.get(1, 0),
								x.get(2, 0),
								x.get(3, 0),
								x.get(4, 0),
								x.get(5, 0)};
	}
	
	public double [] getCovariance(){	
		double [] covariance = new double [36];
		
		for(int i = 0; i < 6; i++) {
			for(int j = 0; j < 6; j++) {
				covariance[6*i + j] = P.get(i, j);
			}
		}
		
		return covariance;
	}
	
	public double [] getNGAcc() {
		return new double [] {a.get(0, 0),
								a.get(1, 0),
								a.get(2, 0)};		
	}
	
	// Función privada para convertir matrices
	private DenseMatrix invertibleMatrix(DenseMatrix m){
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
