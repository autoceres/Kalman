package scorpsystems.nathan.com.smartmapping;

import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;
import java.text.DecimalFormat;
import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class CalibrarActivity extends AppCompatActivity implements SensorEventListener {

    DecimalFormat df = new DecimalFormat("#,##0.000000");

    /* Componentes da tela ----------------------------------------------------------------*/
    TextView tv_LeituraGPSE;
    TextView tv_LeituraGPSN;
    TextView tv_LeituraGPSD;
    TextView tv_LeituraAcelX;
    TextView tv_LeituraAcelY;
    TextView tv_LeituraAcelZ;

    TextView tv_VarGPSE;
    TextView tv_VarGPSN;
    TextView tv_VarGPSD;
    TextView tv_VarAcelX;
    TextView tv_VarAcelY;
    TextView tv_VarAcelZ;
    /*-------------------------------------------------------------------------------------*/

    /* Interrupcao ------------------------------------------------------------------------*/
    Timer temporizador;
    TaskIMU rotinaIMU;
    TaskGPS rotinaGPS;
    /*-------------------------------------------------------------------------------------*/


    /* GPS --------------------------------------------------------------------------------*/
    ModuloGPS gps = null; //new ModuloGPS(getApplicationContext());
    private double lat;
    private double lon;
    private double alt;
    private double firstLAT;
    private double firstLON;
    private double firstALT;
    private double firstECEFX;
    private double firstECEFY;
    private double firstECEFZ;
    private boolean primeiraLeitura = true;
    private double nedE;
    private double nedN;
    private double nedD;
    private double Ne;
    private double e = 0.08181919;
    private double gpsLat_r;
    private double gpsLon_r;
    private double ecefX;
    private double ecefY;
    private double ecefZ;
    private float []matrizECEF2NED = new float[9];
    private double ax_inercial;
    private double ay_inercial;
    private double az_inercial;
    private int contVarGPS = 0;
    /*-------------------------------------------------------------------------------------*/


    /* IMU --------------------------------------------------------------------------------*/
    private float[] gyro = new float[3];
    private float[] magnet = new float[3];
    private float[] accel = new float[3];
    private float[] geomag = new float[3];
    private float[] grav = new float[3];
    private float[] matrizRotacao = new float[9];
    Sensor sensorAcelerometro;
    SensorManager sensManager;
    Sensor sensorMagnetometro;
    Sensor sensorGiroscopio;
    Sensor sensorGeoMagnetico;
    Sensor sensorGravidade;
    private int contVarIMU = 0;
    /*-------------------------------------------------------------------------------------*/

    /* Variancia --------------------------------------------------------------------------*/
    private double[] var1 = new double[300];
    private double[] var2 = new double[300];
    private double[] var3 = new double[300];
    private double media1 = 0;
    private double media2 = 0;
    private double media3 = 0;
    private double varAcelX = 0;
    private double varAcelY = 0;
    private double varAcelZ = 0;
    private double varE = 0;
    private double varN = 0;
    private double varD = 0;
    private float aux;
    /*-------------------------------------------------------------------------------------*/

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calibrar);
        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        tv_LeituraGPSE = (TextView) findViewById(R.id.tv_LeituraGPSE);
        tv_LeituraGPSN = (TextView) findViewById(R.id.tv_LeituraGPSN);
        tv_LeituraGPSD = (TextView) findViewById(R.id.tv_LeituraGPSD);
        tv_LeituraAcelX = (TextView) findViewById(R.id.tv_LeituraAcelX);
        tv_LeituraAcelY = (TextView) findViewById(R.id.tv_LeituraAcelY);
        tv_LeituraAcelZ = (TextView) findViewById(R.id.tv_LeituraAcelZ);

        tv_VarGPSE = (TextView) findViewById(R.id.tv_VarGPSE);
        tv_VarGPSN = (TextView) findViewById(R.id.tv_VarGPSN);
        tv_VarGPSD = (TextView) findViewById(R.id.tv_VarGPSD);
        tv_VarAcelX = (TextView) findViewById(R.id.tv_VarAcelX);
        tv_VarAcelY = (TextView) findViewById(R.id.tv_VarAcelY);
        tv_VarAcelZ = (TextView) findViewById(R.id.tv_VarAcelZ);

        sensManager= (SensorManager)getSystemService(SENSOR_SERVICE);

        sensorAcelerometro = sensManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorGiroscopio = sensManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sensorMagnetometro = sensManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        sensorGeoMagnetico = sensManager.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);
        sensorGravidade = sensManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

        sensManager.registerListener(this, sensorAcelerometro, 0);
        sensManager.registerListener(this, sensorGiroscopio, 0);
        sensManager.registerListener(this, sensorMagnetometro, 0);
        sensManager.registerListener(this, sensorGeoMagnetico, 0);
        sensManager.registerListener(this, sensorGravidade, 0);

        gps = new ModuloGPS(getApplicationContext());

        temporizador = new Timer();
        rotinaIMU = new TaskIMU();
        rotinaGPS = new TaskGPS();

        temporizador.schedule(rotinaIMU,100,100);
        temporizador.schedule(rotinaGPS,1000,1000);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, accel, 0, accel.length);

        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            System.arraycopy(event.values, 0, gyro, 0, gyro.length);

        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, magnet, 0, magnet.length);

        } else if (event.sensor.getType() == Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) {
            System.arraycopy(event.values, 0, geomag, 0, geomag.length);

        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            System.arraycopy(event.values, 0, grav, 0, grav.length);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    class TaskGPS extends TimerTask {
        @Override
        public void run() {
            Location localGPS = gps.pegaLocalicazaoGPS();
            if (localGPS != null){
                //Caso positivo, pega os valores de latidude e longitude
                lat = localGPS.getLatitude();
                lon = localGPS.getLongitude();
                alt = localGPS.getAltitude();

                //Toast.makeText(getApplicationContext(),"Lat : "+lat+"   Lon : "+lon, Toast.LENGTH_LONG).show();
                gpsLat_r = (lat*Math.PI)/180;
                gpsLon_r = (lon*Math.PI)/180;

                Ne = 6378137/Math.sqrt((1 - (e*e*sin(gpsLat_r)*sin(gpsLat_r))));

                ecefX = (Ne + alt)*cos(gpsLat_r)*cos(gpsLon_r);
                ecefY = (Ne + alt)*cos(gpsLat_r)*sin(gpsLon_r);
                ecefZ = ((Ne*(1-(e*e)))+alt)*sin(gpsLat_r);

                if (primeiraLeitura){
                    firstLAT = lat;
                    firstLON = lon;
                    firstALT = alt;
                    firstECEFX = ecefX;
                    firstECEFY = ecefY;
                    firstECEFZ = ecefZ;
                    nedE = 0;
                    nedN = 0;
                    nedD = 0;

                            /* Primeira linha da matriz */
                    matrizECEF2NED[0] = (float) ((-1)*sin(gpsLat_r)*cos(gpsLon_r));
                    matrizECEF2NED[1] = (float) ((-1)*sin(gpsLat_r)*sin(gpsLon_r));
                    matrizECEF2NED[2] = (float) cos(gpsLat_r);
                            /* Segunda linha da matriz */
                    matrizECEF2NED[3] = (float) ((-1)*sin(gpsLon_r));
                    matrizECEF2NED[4] = (float) cos(gpsLon_r);
                    matrizECEF2NED[5] = 0;
                            /* Terceira linha da matriz */
                    matrizECEF2NED[6] = (float) ((-1)*cos(gpsLat_r)*cos(gpsLon_r));
                    matrizECEF2NED[7] = (float) ((-1)*cos(gpsLat_r)*sin(gpsLon_r));
                    matrizECEF2NED[8] = (float) ((-1)*sin(gpsLat_r));

                    primeiraLeitura = false;
                }
                else {
                    nedN = (matrizECEF2NED[0]*(ecefX-firstECEFX)) + (matrizECEF2NED[1]*(ecefY-firstECEFY)) + (matrizECEF2NED[2]*(ecefZ-firstECEFZ));
                    nedE = (matrizECEF2NED[3]*(ecefX-firstECEFX)) + (matrizECEF2NED[4]*(ecefY-firstECEFY)) + (matrizECEF2NED[5]*(ecefZ-firstECEFZ));
                    nedD = (matrizECEF2NED[6]*(ecefX-firstECEFX)) + (matrizECEF2NED[7]*(ecefY-firstECEFY)) + (matrizECEF2NED[8]*(ecefZ-firstECEFZ));

                }

                if ((contVarGPS < 300)&&(contVarIMU == 999)){
                    var1[contVarGPS] = nedE;
                    var2[contVarGPS] = nedN;
                    var3[contVarGPS] = nedD;
                    contVarGPS++;

                }else if (contVarGPS == 300){
                    for (contVarGPS = 0; contVarGPS<300; contVarGPS++){
                        media1 = media1 + var1[contVarGPS];
                        media2 = media2 + var2[contVarGPS];
                        media3 = media3 + var3[contVarGPS];
                    }
                    media1 = media1/300;
                    media2 = media2/300;
                    media3 = media3/300;
                    for (contVarGPS = 0; contVarGPS<300; contVarGPS++){
                        varE = varE + ((var1[contVarGPS]-media1)*(var1[contVarGPS]-media1));
                        varN = varN + ((var2[contVarGPS]-media2)*(var2[contVarGPS]-media2));
                        varD = varD + ((var3[contVarGPS]-media3)*(var3[contVarGPS]-media3));
                    }
                    varE = varE/299;
                    varN = varN/299;
                    varD = varD/299;

                    tv_VarGPSE.setText(df.format(varE));
                    tv_VarGPSN.setText(df.format(varN));
                    tv_VarGPSD.setText(df.format(varD));
                    contVarGPS = 999;

                }

                tv_LeituraGPSE.setText(df.format(nedE));
                tv_LeituraGPSN.setText(df.format(nedN));
                tv_LeituraGPSD.setText(df.format(nedD));
            }
        }
    }

    class TaskIMU extends TimerTask {

        @Override
        public void run() {
            sensManager.getRotationMatrix(matrizRotacao, null,
                    accel, magnet);

            ax_inercial = (matrizRotacao[0]*(accel[0]-grav[0])) + (matrizRotacao[1]*(accel[1]-grav[1])) + (matrizRotacao[2]*(accel[2]-grav[2]));
            ay_inercial = (matrizRotacao[3]*(accel[0]-grav[0])) + (matrizRotacao[4]*(accel[1]-grav[1])) + (matrizRotacao[5]*(accel[2]-grav[2]));
            az_inercial = (matrizRotacao[6]*(accel[0]-grav[0])) + (matrizRotacao[7]*(accel[1]-grav[1])) + (matrizRotacao[8]*(accel[2]-grav[2]));

            if (contVarIMU < 300){
                var1[contVarIMU] = ax_inercial;
                var2[contVarIMU] = ay_inercial;
                var3[contVarIMU] = az_inercial;
                contVarIMU++;

            }else if (contVarIMU == 300){
                for (contVarIMU = 0; contVarIMU<300; contVarIMU++){
                    media1 = media1 + var1[contVarIMU];
                    media2 = media2 + var2[contVarIMU];
                    media3 = media3 + var3[contVarIMU];
                }
                media1 = media1/300;
                media2 = media2/300;
                media3 = media3/300;
                for (contVarIMU = 0; contVarIMU<300; contVarIMU++){
                    varAcelX = varAcelX + ((var1[contVarIMU]-media1)*(var1[contVarIMU]-media1));
                    varAcelY = varAcelY + ((var2[contVarIMU]-media2)*(var2[contVarIMU]-media2));
                    varAcelZ = varAcelZ + ((var3[contVarIMU]-media3)*(var3[contVarIMU]-media3));
                }
                varAcelX = varAcelX/299;
                varAcelY = varAcelY/299;
                varAcelZ = varAcelZ/299;
                tv_VarAcelX.setText(df.format(varAcelX));
                tv_VarAcelY.setText(df.format(varAcelY));
                tv_VarAcelZ.setText(df.format(varAcelZ));
                contVarIMU = 999;

            }
            tv_LeituraAcelX.setText(df.format(ax_inercial));
            tv_LeituraAcelY.setText(df.format(ay_inercial));
            tv_LeituraAcelZ.setText(df.format(az_inercial));
        }
    }

}
