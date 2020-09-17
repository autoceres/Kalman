package scorpsystems.nathan.com.smartmapping.kalman;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import jeigen.DenseMatrix;
import scorpsystems.nathan.com.smartmapping.dcm.DCM_IMU;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Accelerometer;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Coordinate;
import scorpsystems.nathan.com.smartmapping.kalman.beans.GPS;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gravity;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gyroscope;
import scorpsystems.nathan.com.smartmapping.kalman.beans.IMU;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Magnetometer;
import scorpsystems.nathan.com.smartmapping.madgwick.MadgwickAHRS;

import static jeigen.Shortcuts.*;

public class KalmanFilter {
    static final double dt = 0.02d; // amostragem (segundos)
    static final double cov_const = 0.05d; // porque este valor es constante para la covarianza
    static final double T = 0.2d;  // constante a ajustar

    // https://javatutoring.com/command-line-arguments-java-with-examples/
    // https://jcommander.org/
    // https://github.com/cbeust/jcommander
    public static void main(String[] args) throws IOException, InterruptedException {
        // Path
        //Parameters params = new Parameters();
        //String[] argv = { "-path", "/home", "-output", "/exit"};
        //JCommander.newBuilder().addObject(params).build().parse(args);

        String path = "";   //params.path;
        String output = "";  //params.output;
        boolean show = true;//params.show;

        // Formatter
        NumberFormat numFormat = new DecimalFormat("0.#####E0");

        // Read csv files
        // Variancia
        // GPS
        ArrayList<Double> latVariancia =  UtilData.csvToArrayDouble(path + File.separator + "latVariancia.csv");
        ArrayList<Double> lonVariancia =  UtilData.csvToArrayDouble(path + File.separator + "lonVariancia.csv");

        // Load data GPS
        ArrayList<GPS> gps_variancia = UtilData.loadDataGPS(latVariancia, lonVariancia);

        // IMU
        ArrayList<Double> accXVariancia =  UtilData.csvToArrayDouble(path + File.separator + "accXVariancia.csv");
        ArrayList<Double> accYVariancia =  UtilData.csvToArrayDouble(path + File.separator + "accYVariancia.csv");
        ArrayList<Double> accZVariancia =  UtilData.csvToArrayDouble(path + File.separator + "accZVariancia.csv");

        ArrayList<Double> magXVariancia =  UtilData.csvToArrayDouble(path + File.separator + "magXVariancia.csv");
        ArrayList<Double> magYVariancia =  UtilData.csvToArrayDouble(path + File.separator + "magYVariancia.csv");
        ArrayList<Double> magZVariancia =  UtilData.csvToArrayDouble(path + File.separator + "magZVariancia.csv");

        ArrayList<Double> gravXVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gravXVariancia.csv");
        ArrayList<Double> gravYVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gravYVariancia.csv");
        ArrayList<Double> gravZVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gravZVariancia.csv");

        ArrayList<Double> gyroXVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gyroXVariancia.csv");
        ArrayList<Double> gyroYVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gyroYVariancia.csv");
        ArrayList<Double> gyroZVariancia =  UtilData.csvToArrayDouble(path + File.separator + "gyroZVariancia.csv");

        ArrayList<IMU> imu_variancia = UtilData.loadDataIMU(accXVariancia, accYVariancia, accZVariancia, magXVariancia, magYVariancia, magZVariancia, gravXVariancia, gravYVariancia, gravZVariancia, gyroXVariancia, gyroYVariancia, gyroZVariancia);

        // Leidos
        // GPS
        ArrayList<Double> lat =  UtilData.csvToArrayDouble(path + File.separator + "lat.csv");
        ArrayList<Double> lon =  UtilData.csvToArrayDouble(path + File.separator + "lon.csv");

        // Load data GPS
        ArrayList<GPS> gps = UtilData.loadDataGPS(lat, lon);

        // IMU
        ArrayList<Double> accX =  UtilData.csvToArrayDouble(path + File.separator + "accX.csv");
        ArrayList<Double> accY =  UtilData.csvToArrayDouble(path + File.separator + "accY.csv");
        ArrayList<Double> accZ =  UtilData.csvToArrayDouble(path + File.separator + "accZ.csv");

        ArrayList<Double> magX =  UtilData.csvToArrayDouble(path + File.separator + "magX.csv");
        ArrayList<Double> magY =  UtilData.csvToArrayDouble(path + File.separator + "magY.csv");
        ArrayList<Double> magZ =  UtilData.csvToArrayDouble(path + File.separator + "magZ.csv");

        ArrayList<Double> gravX =  UtilData.csvToArrayDouble(path + File.separator + "gravX.csv");
        ArrayList<Double> gravY =  UtilData.csvToArrayDouble(path + File.separator + "gravY.csv");
        ArrayList<Double> gravZ =  UtilData.csvToArrayDouble(path + File.separator + "gravZ.csv");

        ArrayList<Double> gyroX =  UtilData.csvToArrayDouble(path + File.separator + "gyroX.csv");
        ArrayList<Double> gyroY =  UtilData.csvToArrayDouble(path + File.separator + "gyroY.csv");
        ArrayList<Double> gyroZ =  UtilData.csvToArrayDouble(path + File.separator + "gyroZ.csv");

        ArrayList<IMU> imu = UtilData.loadDataIMU(accX, accY, accZ, magX, magY, magZ, gravX, gravY, gravZ, gyroX, gyroY, gyroZ);

        // CALCULAR VARIANCIA

        // Angulos
        ArrayList<Double> angVar = new ArrayList<Double>();

        // Cargar ángulos
        for(int x=0; x<gyroZVariancia.size();x++) {
            angVar.add(-1d * (gyroZVariancia.get(x))*dt/2d);
        }

        ArrayList<Double> acc_substract_grav_X = Util.subtractArraysValues(accXVariancia, gravXVariancia);
        ArrayList<Double> acc_substract_grav_Y = Util.subtractArraysValues(accYVariancia, gravYVariancia);

        // Instancia de DCM_IMU
        DCM_IMU dcm = new DCM_IMU();

        Coordinate coordinate = new Coordinate(gps.get(0).getLatitude(), gps.get(0).getLongitude());
        Coordinate m2g = Util.convertMeterToGraus(coordinate);

        // Calcular varianzas
        double varLat = Util.getVariance(latVariancia);
        double varLon = Util.getVariance(lonVariancia);
        double varAccX = Util.getVariance(acc_substract_grav_X);
        double varAccY = Util.getVariance(acc_substract_grav_Y);
        double varAng = Util.getVariance(angVar);

        System.out.println("CALCULAR VARIÂNCIAS");
        System.out.println("\nLat: " + numFormat.format(varLat) +"\tLat: " + numFormat.format(varLon));
        System.out.println("\nAccX: " + varLat + "\tAccY: " + varLon);
        System.out.println("\nAng: " + varAng);

        // Matriz de transición A
        DenseMatrix A = new DenseMatrix (new double [][]
                {
                        {1, 	0, 		dt, 	0, 		Math.pow(dt,2)/2d, 	0},
                        {0, 	1, 		0, 		dt, 	0, 					Math.pow(dt,2)/2d},
                        {0, 	0, 		1, 		0, 		dt, 				0},
                        {0, 	0, 		0, 		1, 		0, 					dt},
                        {0, 	0, 		0, 		0, 		1, 					0},
                        {0, 	0, 		0, 		0, 		0, 					1}
                });


        // Covarianza estimada P
        DenseMatrix P = eye(6);

        // Matriz de covariancia de processo Q
        DenseMatrix Q = eye(6).mul(cov_const);

        // Matriz Identidad
        DenseMatrix I = eye(6);

        //
        DenseMatrix z = zeros(4,1);

        //
        DenseMatrix H = zeros(4,6);

        //
        DenseMatrix R = zeros(4,4);

        //
        DenseMatrix x_est;

        // Usado como auxiliar
        DenseMatrix x_actual;

        //
        DenseMatrix aux;

        //
        DenseMatrix K;

        //
        DenseMatrix y;

        // Vector de estado inicial
        // Tiene la primera lectura del GPS y IMU, y es considerado que el cuerpo estaba en el reposo
        DenseMatrix x_ant = new DenseMatrix (new double [][]
                {{gps.get(0).getLongitude()},
                        {gps.get(0).getLatitude()},
                        {0},
                        {0},
                        {0},
                        {0}});
        // Rotation Matrix
        DenseMatrix RM = zeros(1,9);
        // Direct Cosine Matrix
        DenseMatrix DCM = zeros(3,3);
        // Angular Velocity Matrix
        DenseMatrix AVM = zeros(3,3);

        // Vector de orientación
        DenseMatrix vo = zeros(1,3);

        // Vector para armacenar estados
        DenseMatrix estados = zeros(7,accX.size());
        DenseMatrix estimado = zeros(7,accX.size());
        DenseMatrix angulos = zeros(12,accX.size());

        // Variables
        double posLatAnt = 0;
        double posLonAnt = 0;
        double accXFinal = 0;
        double accYFinal = 0;

        int j = 0; // Cuenta las coordenadas UTM

        // DCM_IMU
        double [] values;

        double roll_dcm;
        double pitch_dcm;
        double yaw_dcm;

        double [] quaternion_dcm;

        // Métodos Android
        double roll_a;
        double pitch_a;
        double yaw_a;

        double [] quaternion_a;

        // Valores iniciales
        MadgwickAHRS ahrs = new MadgwickAHRS(1f/50f, 0.1f);

        ahrs.update((float) imu.get(0).getGyroscope().getX(),
                (float) imu.get(0).getGyroscope().getY(),
                (float) imu.get(0).getGyroscope().getZ(),
                (float) imu.get(0).getAccelerometer().getX(),
                (float) imu.get(0).getAccelerometer().getY(),
                (float) imu.get(0).getAccelerometer().getZ(),
                (float) imu.get(0).getMagnetometer().getX(),
                (float) imu.get(0).getMagnetometer().getY(),
                (float) imu.get(0).getMagnetometer().getZ());

        double [] quaternion = ahrs.getQuaternion();
        double [] euler = Util.quaternionToEuler(Util.getQuaternionConjugate(quaternion));

        // Valores euler
        double roll = euler[0];
        double pitch = euler[1];
        double yaw = euler[2];

        RM = Util.getRotationMatrix(imu.get(1).getAccelerometer(), imu.get(1).getMagnetometer(), imu.get(1).getGyroscope());
        vo = Util.getOrientation(RM);

        double roll_filtrado = 0;
        double pitch_filtrado = 0;
        double yaw_filtrado = 0;

        Gyroscope gyro_filtrado = new Gyroscope();
        Accelerometer acc_filtrado = new Accelerometer();
        Magnetometer mag_filtrado = new Magnetometer();
        Gravity grav_filtrado = new Gravity();

        Accelerometer acc_rot = new Accelerometer();
        Gyroscope gyro_rot = new Gyroscope();
        Magnetometer mag_rot = new Magnetometer();
        Gravity grav_rot = new Gravity();

        boolean chamaKalman = false;

        // Número de muestras
        int numAmostras = imu.size();
        int cont = 1; // contador para chamdas do GPS

        // Loop Principal
        Gyroscope gyro;
        Accelerometer acc;
        Magnetometer mag;
        Gravity grav;

        for(int i = 0; i<numAmostras; i++) {
            // Ajusta os valores de aceleração
            gyro = imu.get(i).getGyroscope();
            acc = imu.get(i).getAccelerometer();
            mag = imu.get(i).getMagnetometer();
            grav = imu.get(i).getGravity();
			/*//FILTRO-----------------------------------------------------------
			T-> constante a ajustar
			dt-> tempo de discretizacao
			ac_fitlrado-> sinal filtrado
			wc-> Valor a filtrar
			acx_filtrado=(1-(dt/T))*acx_filtrado+wcx(k)*(dt/T);
			acy_filtrado=(1-(dt/T))*acy_filtrado+wcy(k)*(dt/T);
			acz_filtrado=(1-(dt/T))*acz_filtrado+wcz(k)*(dt/T);
			-----------------------------------------------------------------*/
            // Ver que significa esta fórmula
            gyro_filtrado.setX((1d-(dt/T))*gyro_filtrado.getX() + gyro.getX()*(dt/T));
            gyro_filtrado.setY((1d-(dt/T))*gyro_filtrado.getY() + gyro.getY()*(dt/T));
            gyro_filtrado.setZ((1d-(dt/T))*gyro_filtrado.getZ() + gyro.getZ()*(dt/T));

            acc_filtrado.setX((1d-(dt/T))*acc_filtrado.getX() + acc.getX()*(dt/T));
            acc_filtrado.setY((1d-(dt/T))*acc_filtrado.getY() + acc.getY()*(dt/T));
            acc_filtrado.setZ((1d-(dt/T))*acc_filtrado.getZ() + acc.getZ()*(dt/T));

            mag_filtrado.setX((1d-(dt/T))*mag_filtrado.getX() + mag.getX()*(dt/T));
            mag_filtrado.setY((1d-(dt/T))*mag_filtrado.getY() + mag.getY()*(dt/T));
            mag_filtrado.setZ((1d-(dt/T))*mag_filtrado.getZ() + mag.getZ()*(dt/T));

            grav_filtrado.setX((1d-(dt/T))*grav_filtrado.getX() + grav.getX()*(dt/T));
            grav_filtrado.setY((1d-(dt/T))*grav_filtrado.getY() + grav.getY()*(dt/T));
            grav_filtrado.setZ((1d-(dt/T))*grav_filtrado.getZ() + grav.getZ()*(dt/T));

			/*--------------  Angulos de Euler  --------------------------------
		                      Metodo DCM										*/
            dcm.updateIMU(gyro_filtrado.getX(), gyro_filtrado.getY(), gyro_filtrado.getZ(),
                    acc_filtrado.getX(), acc_filtrado.getY(), acc_filtrado.getZ(), dt);

            values = dcm.getValues();

            roll_dcm = values[0];
            pitch_dcm = values[1];
            yaw_dcm = values[2] * -1.0d; // Cambia de signo

            quaternion_dcm = Util.eulerToQuaternion(new double [] {yaw_dcm, pitch_dcm, roll_dcm});

//			System.out.println("i: " + String.valueOf(i));
//			System.out.println("gyro_filtrado: " + gyro_filtrado);
//			System.out.println("acc_filtrado: " + acc_filtrado);
//			System.out.println("roll_dcm: " + String.valueOf(roll_dcm));
//			System.out.println("pitch_dcm: " + String.valueOf(pitch_dcm));
//			System.out.println("yaw_dcm: " + String.valueOf(yaw_dcm));
//			cont = cont;
            /* ------------------------------------------------------------------ */

            /* --------------- Angulos de Euler --------------------------------- */
            /*                      AHRS										  */
            ahrs.update(gyro_filtrado.getX(),
                    gyro_filtrado.getY(),
                    gyro_filtrado.getZ(),
                    acc_filtrado.getX(),
                    acc_filtrado.getY(),
                    acc_filtrado.getZ(),
                    mag_filtrado.getX(),
                    mag_filtrado.getY(),
                    mag_filtrado.getZ());

            quaternion = ahrs.getQuaternion();
            euler = Util.quaternionToEuler(Util.getQuaternionConjugate(quaternion));

            roll = euler[0];
            pitch = euler[1];
            yaw = euler[2];
            /* ------------------------------------------------------------------ */

            /* --------------- Angulos de Euler --------------------------------- */
            /*                Metodos Android									  */
            RM = Util.getRotationMatrix(acc_filtrado, mag_filtrado, gyro_filtrado);
            vo = Util.getOrientation(RM);

            yaw_a = vo.get(0, 0);
            pitch_a = vo.get(0, 1);
            roll_a = vo.get(0, 2);

            quaternion_a = Util.eulerToQuaternion(new double [] {yaw_a, pitch_a, roll_a});
            /* ------------------------------------------------------------------ */

            roll_filtrado = (1-(dt/T)) * roll_filtrado + roll * (dt/T);
            pitch_filtrado = (1-(dt/T)) * pitch_filtrado + pitch * (dt/T);
            yaw_filtrado = (1-(dt/T)) * yaw_filtrado + yaw * (dt/T);

            /* ----- Matriz de rotacao do sistema ---------------------------------- */

            DCM = Util.getDirectCosineMatrix(roll_filtrado, pitch_filtrado, yaw_filtrado);
            AVM = Util.getAngularVelocityMatrix(gyro_filtrado.getX(), gyro_filtrado.getY(), gyro_filtrado.getZ());
            RM = DCM.mmul(AVM);

            /* -------------------------------------------------------------------- */

            acc_rot.setX(RM.get(0,0) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,0) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,0) * (acc_filtrado.getZ() - grav_filtrado.getZ()));
            acc_rot.setY(RM.get(0,1) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,1) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,1) * (acc_filtrado.getZ() - grav_filtrado.getZ()));
            acc_rot.setZ(RM.get(0,2) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,2) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,2) * (acc_filtrado.getZ() - grav_filtrado.getZ()));

            gyro_rot.setX(RM.get(0,0) * (imu.get(i).getGyroscope().getX()) + RM.get(1,0) * (imu.get(i).getGyroscope().getY()) + RM.get(2,0) * (imu.get(i).getGyroscope().getZ()));
            gyro_rot.setY(RM.get(0,1) * (imu.get(i).getGyroscope().getX()) + RM.get(1,1) * (imu.get(i).getGyroscope().getY()) + RM.get(2,1) * (imu.get(i).getGyroscope().getZ()));
            gyro_rot.setZ(RM.get(0,2) * (imu.get(i).getGyroscope().getX()) + RM.get(1,2) * (imu.get(i).getGyroscope().getY()) + RM.get(2,2) * (imu.get(i).getGyroscope().getZ()));

            mag_rot.setX(RM.get(0,0) * (imu.get(i).getMagnetometer().getX()) + RM.get(1,0) * (imu.get(i).getMagnetometer().getY()) + RM.get(2,0) * (imu.get(i).getMagnetometer().getZ()));
            mag_rot.setY(RM.get(0,1) * (imu.get(i).getMagnetometer().getX()) + RM.get(1,1) * (imu.get(i).getMagnetometer().getY()) + RM.get(2,1) * (imu.get(i).getMagnetometer().getZ()));
            mag_rot.setZ(RM.get(0,2) * (imu.get(i).getMagnetometer().getX()) + RM.get(1,2) * (imu.get(i).getMagnetometer().getY()) + RM.get(2,2) * (imu.get(i).getMagnetometer().getZ()));

            accXFinal = acc_rot.getX();
            accYFinal = acc_rot.getY();

            accXFinal = accXFinal * m2g.getX();
            accYFinal = accYFinal * m2g.getY();

            /* ------------------------------------------------------------------- */
            if(cont == 49) {
                // Toma coordenadas UTM
                j = j + 1;

                if((gps.get(j).getLatitude() != posLatAnt) && (gps.get(j).getLongitude() != posLonAnt)) {
                    // Vector de lecturas
                    z = new DenseMatrix(new double [][]
                            {
                                    {gps.get(j).getLongitude()},
                                    {gps.get(j).getLatitude()},
                                    {accXFinal},
                                    {accYFinal}
                            });

                    H = new DenseMatrix(new double [][]
                            {{1,0,0,0,0,0},
                                    {0,1,0,0,0,0},
                                    {0,0,0,0,1,0},
                                    {0,0,0,0,0,1}}
                    );

                    R = new DenseMatrix(new double [][]
                            {
                                    {varLon,	0,			0,				0},
                                    {0,			varLat,		0,				0},
                                    {0,			0,			varAccX,		0},
                                    {0,			0,			0,				varAccY}
                            }
                    );

                    posLatAnt = gps.get(j).getLatitude();
                    posLonAnt = gps.get(j).getLongitude();

                    chamaKalman = true;
                } else {
                    // Vector de lecturas
                    z = new DenseMatrix(new double [][]
                            {
                                    {accXFinal},
                                    {accYFinal}
                            });

                    H = new DenseMatrix(new double [][]
                            {{0,0,0,0,1,0},
                                    {0,0,0,0,0,1}}
                    );

                    R = new DenseMatrix(new double [][]
                            {
                                    {varAccX,   0},
                                    {0,  varAccY}
                            }
                    );

                    cont = 0;
                }

                // Vector de lecturas
                z = new DenseMatrix(new double [][]
                        {
                                {gps.get(j).getLongitude()},
                                {gps.get(j).getLatitude()},
                                {accXFinal},
                                {accYFinal}
                        });

                H = new DenseMatrix(new double [][]
                        {{1,0,0,0,0,0},
                                {0,1,0,0,0,0},
                                {0,0,0,0,1,0},
                                {0,0,0,0,0,1}}
                );

                R = new DenseMatrix(new double [][]
                        {
                                {varLon,	0,			0,				0},
                                {0,			varLat,		0,				0},
                                {0,			0,			varAccX,		0},
                                {0,			0,			0,				varAccY}
                        }
                );

                cont = 0;
            } else {
                cont = cont;
                // Vector de lecturas
                z = new DenseMatrix(new double [][]
                        {
                                {accXFinal},
                                {accYFinal}
                        });

                H = new DenseMatrix(new double [][]
                        {{0,0,0,0,1,0},
                                {0,0,0,0,0,1}}
                );

                R = new DenseMatrix(new double [][]
                        {
                                {varAccX,   0},
                                {0,  varAccY}
                        }
                );
            }

            // Estima el nuevo estado a partir de matrix A y del estado anterior
            x_est = A.mmul(x_ant);

            // Actualiza la matriz P
            P = A.mmul(P).mmul(A.t()).add(Q);

            if(chamaKalman) {
                // Calcula o ganho de Kalman
                //  K =  P * transpose(H)
                //     --------------------
                //    H*P*transpose(H) + R
                //
                aux = H.mmul(P).mmul(H.t()).add(R);
                K = P.mmul(H.t()).mmul(Util.invertibleMatrix(aux));

                // Observa los valores de aceleración del IMU
                y = z.sub(H.mmul(x_est));

                // Actualiza el estado actual
                x_actual = x_est.add(K.mmul(y));
                x_ant = x_est.add(K.mmul(y));

                // Actualiza la matrix P
                P = I.sub(K.mmul(H)).mmul(P);

                // Debug
                estados.set(0, i, x_actual.get(0, 0));
                estados.set(1, i, x_actual.get(1, 0));
                estados.set(2, i, x_actual.get(2, 0));
                estados.set(3, i, x_actual.get(3, 0));
                estados.set(4, i, x_actual.get(4, 0));
                estados.set(5, i, x_actual.get(5, 0));

                chamaKalman = false;
            } else {
                x_ant = x_est;
            }

            estimado.set(0, i, x_est.get(0, 0));
            estimado.set(1, i, x_est.get(1, 0));
            estimado.set(2, i, x_est.get(2, 0));
            estimado.set(3, i, x_est.get(3, 0));
            estimado.set(4, i, x_est.get(4, 0));
            estimado.set(5, i, x_est.get(5, 0));
            estimado.set(6, i, yaw * 180.0/Math.PI);

            angulos.set(0, i, yaw * 180.0/Math.PI);
            angulos.set(1, i,yaw_a * 180.0/Math.PI);
            angulos.set(2, i,yaw_dcm * 180.0/Math.PI);
            angulos.set(3, i,yaw_filtrado * 180.0/Math.PI);

            angulos.set(4, i,roll * 180.0/Math.PI);
            angulos.set(5, i,roll_a * 180.0/Math.PI);
            angulos.set(6, i,roll_dcm * 180.0/Math.PI);
            angulos.set(7, i,roll_filtrado * 180.0/Math.PI);

            angulos.set(8, i,pitch * 180.0/Math.PI);
            angulos.set(9, i,pitch_a * 180.0/Math.PI);
            angulos.set(10, i,pitch_dcm * 180.0/Math.PI);
            angulos.set(11, i,pitch_filtrado * 180.0/Math.PI);

            if(show) {
                TimeUnit.SECONDS.sleep(1);

                // Print
                System.out.println("\n\nn = " + Integer.toString(i));
                System.out.println("\nVALORES ESTIMADOS:" );

                System.out.print("0: " + numFormat.format(estimado.get(0, i)) +"\t1: " + numFormat.format(estimado.get(1, i)));
                System.out.print("\t2: " + numFormat.format(estimado.get(2, i)) +"\t3: " + numFormat.format(estimado.get(3, i)));
                System.out.print("\t4: " + numFormat.format(estimado.get(4, i)) +"\t5: " + numFormat.format(estimado.get(5, i)));
                System.out.println("\t6: " + numFormat.format(estimado.get(6, i)));

                System.out.println("\nANGULOS:" );
                System.out.print("0: " + numFormat.format(angulos.get(0, i)) +"\t1: " + numFormat.format(angulos.get(1, i)));
                System.out.print("\t2: " + numFormat.format(angulos.get(2, i)) +"\t3: " + numFormat.format(angulos.get(3, i)));
                System.out.print("\t4: " + numFormat.format(angulos.get(4, i)) +"\t5: " + numFormat.format(angulos.get(5, i)));
                System.out.println("\t6: " + numFormat.format(angulos.get(6, i)));
                System.out.print("7: " + numFormat.format(angulos.get(7, i)) +"\t8: " + numFormat.format(angulos.get(8, i)));
                System.out.print("\t9: " + numFormat.format(angulos.get(9, i)) +"\t10: " + numFormat.format(angulos.get(10, i)));
                System.out.print("\t11: " + numFormat.format(angulos.get(11, i)));
            }

            cont = cont + 1;
        }

        // Guardar Datos
        FileWriter csvEstados = new FileWriter(output + File.separator + "estados_java.csv");
        FileWriter csvEstimados = new FileWriter(output + File.separator + "estimados_java.csv");
        FileWriter csvAngulos = new FileWriter(output + File.separator + "angulos_java.csv");

        int count = 0;
        for(int l=0; l<estados.rows; l++) {
            for(int c=0; c < estados.cols; c++) {
                count = count + 1;
                csvEstados.append(String.valueOf(estados.get(l, c)));
                if(count < estados.cols) {
                    csvEstados.append(",");
                } else {
                    csvEstados.append("\n");
                    count = 0;
                }
            }
        }
        csvEstados.flush();
        csvEstados.close();

        count = 0;
        for(int l=0; l<estimado.rows; l++) {
            for(int c=0; c < estimado.cols; c++) {
                count = count + 1;
                csvEstimados.append(String.valueOf(estimado.get(l, c)));
                if(count < estados.cols) {
                    csvEstimados.append(",");
                } else {
                    csvEstimados.append("\n");
                    count = 0;
                }
            }
        }
        csvEstimados.flush();
        csvEstimados.close();

        count = 0;
        for(int l=0; l < angulos.rows; l++) {
            for(int c=0; c < angulos.cols; c++) {
                count = count + 1;
                csvAngulos.append(String.valueOf(angulos.get(l, c)));
                if(count < angulos.cols) {
                    csvAngulos.append(",");
                } else {
                    csvAngulos.append("\n");
                    count = 0;
                }
            }
        }
        csvAngulos.flush();
        csvAngulos.close();

        System.out.println("DONE");
    }
}
