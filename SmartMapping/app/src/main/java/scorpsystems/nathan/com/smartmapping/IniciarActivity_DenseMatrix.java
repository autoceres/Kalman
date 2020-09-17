package scorpsystems.nathan.com.smartmapping;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.FragmentActivity;
import android.support.v7.app.AlertDialog;
import android.text.InputType;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;

import jeigen.DenseMatrix;
import scorpsystems.nathan.com.smartmapping.kalman.Util;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Accelerometer;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Coordinate;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gravity;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Gyroscope;
import scorpsystems.nathan.com.smartmapping.kalman.beans.Magnetometer;

import static jeigen.Shortcuts.eye;
import static jeigen.Shortcuts.zeros;

public class IniciarActivity_DenseMatrix extends FragmentActivity implements OnMapReadyCallback, SensorEventListener {
    private static final int MY_PERMISSION_REQUEST_CODE = 1001;

    boolean usarModuloArduino = false;
    ImageButton ibt_Coletar;
    ImageButton ibt_Finalizar;
    ImageButton ibt_Iniciar;

    boolean debugGeraRelatorioDados = false;
    int cont = 0;
    String dadosIMUacelInercialX = "";
    String dadosIMUacelInercialY= "";
    String dadosIMUacelInercialZ = "";
    String dadosIMUacelX = "accX = [";
    String dadosIMUacelY = "accY = [";
    String dadosIMUacelZ = "accZ = [";
    String dadosIMUmagX = "magX = [";
    String dadosIMUmagY = "magY = [";
    String dadosIMUmagZ = "magZ = [";
    String dadosIMUgyroX = "gyroX = [";
    String dadosIMUgyroY = "gyroY = [";
    String dadosIMUgyroZ = "gyroZ = [";
    String dadosGPSLat = "lat = [";
    String dadosGPSLon = "lon = [";
    String dadosGravX = "gravX = [";
    String dadosGravY = "gravY = [";
    String dadosGravZ = "gravZ = [";
    String fileNameIMUacelInercial = "IMUacelInercial.txt";
    String fileNameIMUacel = "IMUacel.txt";
    String fileNameIMUmag = "IMUmag.txt";
    String fileNameIMUgyro = "IMUgyro.txt";
    String fileNameGPS = "GPS.txt";
    String fileNameGrav = "Grav.txt";
    String folderDados = "RELATORIO";
    String folderCalibragem = "DADOS";
    String fileNamePasseio = "";
    String fileNameCalibragem = "Calibragem.txt";

    String valorLido;

    public TextView tv_LatitudeAtual;
    public TextView tv_LongitudeAtual;
    public TextView tv_ValorLido;

    /* Bluetooth --------------------------------------------------------------------------*/
    boolean conectado = false;
    BluetoothAdapter meuBluetoothAdapter = null;
    private static String MAC = null;
    BluetoothDevice meuDevice = null;
    BluetoothSocket meuSocket = null;
    UUID UUID_SERIAL_PORT = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
    ConnectedThread connectedThread;
    Handler meuHandler;
    StringBuilder dadosBluetooth = new StringBuilder();
    private static final int SOLICITA_ATIVACAO_BT = 1;
    private static final int SOLICITA_CONEXAO_BT = 2;
    private static final int MENSAGEM_RECEBIDA_BT = 3;
    /*-------------------------------------------------------------------------------------*/

    /* Interrupcao ------------------------------------------------------------------------*/
    Timer temporizador;
    TaskIMU rotinaIMU;
    TaskGPS rotinaGPS;
    TaskKalman rotinaKalman;
    /*-------------------------------------------------------------------------------------*/

    /* GPS --------------------------------------------------------------------------------*/
    private double lat;
    private double lon;

    /*
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
    */

    private SimpleMatrix F,Q,H;

    private double e = 0.08181919;
    private double cov_const = 0.05d; // porque este valor es constante para la covarianza

    private double gpsLat_r;
    private double gpsLon_r;
    private double ecefX;
    private double ecefY;
    private double ecefZ;
    private float[] matrizECEF2NED = new float[9];
    private double ax_inercial;
    private double ay_inercial;
    private double az_inercial;
    private int contVarGPS = 0;

    private GoogleMap meuMapa;
    private LocationManager managerGPS;
    private LocationListener listenerGPS;

    //private double m2gX;
    //private double m2gY;



    /* IMU --------------------------------------------------------------------------------*/
    //private double[] gyro = new double[3];
    //private double[] magnet = new double[3];
    //private double[] accel = new double[3];
    //private double[] grav = new double[3];

    /*
    private float[] gyro = new float[3];
    private float[] magnet = new float[3];
    private float[] accel = new float[3];
    private float[] grav = new float[3];
    private float[] gyro_filtrado = new float[3];
    private float[] magnet_filtrado = new float[3];
    private float[] accel_filtrado = new float[3];
    private float[] grav_filtrado = new float[3];
    */

    // Arrays
    private float[] gyro_array = new float[3];
    private float[] magnet_array = new float[3];
    private float[] accel_array = new float[3];
    private float[] grav_array = new float[3];

    Gyroscope gyro;
    Accelerometer acc;
    Magnetometer mag;
    Gravity grav;

    Gyroscope gyro_filtrado = new Gyroscope();
    Accelerometer acc_filtrado = new Accelerometer();
    Magnetometer mag_filtrado = new Magnetometer();
    Gravity grav_filtrado = new Gravity();

    Accelerometer acc_rot = new Accelerometer();
    Gyroscope gyro_rot = new Gyroscope();
    Magnetometer mag_rot = new Magnetometer();
    Gravity grav_rot = new Gravity();

    double accXFinal = 0;
    double accYFinal = 0;

    //private float[] matrizRotacao = new float[9];
    // Rotation Matrix
    DenseMatrix RM = zeros(1,9);
    // Direct Cosine Matrix
    DenseMatrix DCM = zeros(3,3);
    // Angular Velocity Matrix
    DenseMatrix AVM = zeros(3,3);

    //private float[] vetorOrientacao = new float[3];
    // Vector de orientación
    DenseMatrix vo = zeros(1,3);

    //private float roll;
    //private float pitch;
    //private float yaw;

    private double roll_a = 0;
    private double pitch_a = 0;
    private double yaw_a = 0;
    double [] quaternion_a;

    private double roll_filtrado = 0;
    private double pitch_filtrado = 0;
    private double yaw_filtrado = 0;

    /*
    private float[] DCM = new float[9];
    private double vxIMU;
    private double vyIMU;
    private double vzIMU;
    */

    Sensor sensorAcelerometro;
    SensorManager sensManager;
    Sensor sensorMagnetometro;
    Sensor sensorGiroscopio;
    Sensor sensorGeoMagnetico;
    Sensor sensorGravidade;
    private int contVarIMU = 0;
    /*-------------------------------------------------------------------------------------*/

    /* Variancia --------------------------------------------------------------------------*/
    /*
    private double varAcelX = 0;
    private double varAcelY = 0;
    private double varAcelZ = 0;
    private double varE = 0;
    private double varN = 0;
    private double varD = 0;
    */

    private double varLat = 0;
    private double varLon = 0;
    private double varAccX = 0;
    private double varAccY = 0;
    private double varAng = 0;

    /*-------------------------------------------------------------------------------------*/

    /* Kalman -----------------------------------------------------------------------------*/
    //double[][] A = new double[9][9];
    DenseMatrix A;

    //double[][] P = new double[9][9];
    DenseMatrix P; //  = eye(6);

    //double[][] Q = new double[9][9];
    DenseMatrix Q; // = eye(6).mul(cov_const);

    // Matriz Identidad
    DenseMatrix I;// = eye(6);

    //double[][] H = new double[3][9];
    DenseMatrix H; // = zeros(4,6);

    // double[][] R = new double[3][3];
    DenseMatrix R; // = zeros(4,4);

    // double[][] Z = new double[9][1];
    DenseMatrix Z; // = zeros(4,1);


    // Usado como auxiliar
    DenseMatrix x_actual;
    DenseMatrix estados;

    //double[][] state_ant = new double[9][1];
    DenseMatrix x_ant;

    //double[][] new_state = new double[9][1];

    //double[][] state_pred = new double[9][1];
    DenseMatrix x_est;

    //
    DenseMatrix aux;

    //
    DenseMatrix K;

    //
    DenseMatrix y;

    //double[][] observacao = new double[9][1];
    double[][] nova_P = new double[9][9];
    //double[][] aux = new double[9][9];
    //double[][] aux1 = new double[9][9];
    //double[][] K = new double[9][9];
    double millisTempoAgoraIMU;
    double millisTempoAnteriorIMU;
    double millisTempoAgoraGPS;
    double millisTempoAnteriorGPS;
    double dt;
    double T;

    Coordinate m2g;
    /*--------------------------------------------------------------------------------------*/


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(scorpsystems.nathan.com.smartmapping.R.layout.activity_iniciar);
        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        tv_LatitudeAtual = (TextView) findViewById(scorpsystems.nathan.com.smartmapping.R.id.tv_LatitudeAtual);
        tv_LongitudeAtual = (TextView) findViewById(scorpsystems.nathan.com.smartmapping.R.id.tv_LongitudeAtual);
        tv_ValorLido = (TextView) findViewById(scorpsystems.nathan.com.smartmapping.R.id.tv_ValorLido);

        millisTempoAgoraGPS = System.currentTimeMillis();
        millisTempoAnteriorGPS = System.currentTimeMillis();
        millisTempoAgoraIMU = System.currentTimeMillis();
        millisTempoAnteriorIMU = System.currentTimeMillis();

         /*Pega o adaptador bluetooth do aparelho*/
        meuBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        ibt_Coletar = (ImageButton) findViewById(scorpsystems.nathan.com.smartmapping.R.id.imageButton_Coletar);
        ibt_Finalizar = (ImageButton) findViewById(scorpsystems.nathan.com.smartmapping.R.id.imageButton_Finalizar);
        ibt_Iniciar = (ImageButton) findViewById(scorpsystems.nathan.com.smartmapping.R.id.imageButton_Iniciar);

        /* IMU */
        sensManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        sensorAcelerometro = sensManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorGiroscopio = sensManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sensorMagnetometro = sensManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        sensorGravidade = sensManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

        sensManager.registerListener((SensorEventListener) this, sensorAcelerometro, 0);
        sensManager.registerListener((SensorEventListener) this, sensorGiroscopio, 0);
        sensManager.registerListener((SensorEventListener) this, sensorMagnetometro, 0);
        sensManager.registerListener((SensorEventListener) this, sensorGravidade, 0);

        /* GPS */
        managerGPS = (LocationManager) getSystemService(LOCATION_SERVICE);
        listenerGPS = new LocationListener() {
            @Override
            public void onLocationChanged(Location location) {
                lat = location.getLatitude();
                lon = location.getLongitude();
                //alt = location.getAltitude();

                //Toast.makeText(IniciarActivity.this, lat + "\n" + lon + "\n" + alt, Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onStatusChanged(String provider, int status, Bundle extras) {

            }

            @Override
            public void onProviderEnabled(String provider) {

            }

            @Override
            public void onProviderDisabled(String provider) {

            }
        };

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                requestPermissions(new String[]{
                        Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.INTERNET
                }, 10);
            } else
                solicitaAtualizacaoGPS();
            return;
        } else
            solicitaAtualizacaoGPS();


        try {
            SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(scorpsystems.nathan.com.smartmapping.R.id.mapFrag);
            mapFragment.getMapAsync(this);
        } catch (SecurityException ex) {
            Log.e("PERMISSAO GPS", String.valueOf(ex));
        }

        setaMatrizesKalman();

        /* Pergunta se o usuário deseja utilizar o módulo Arduino com Sensor */
        AlertDialog.Builder solicitaConexaoArduino = new AlertDialog.Builder(this);
        solicitaConexaoArduino.setTitle("Conectar");
        solicitaConexaoArduino.setMessage("Desejar conectar o aplicativo via Bluetooth ao módulo com o sensor?");
        solicitaConexaoArduino.setPositiveButton("SIM", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int which) {
                // Do nothing but close the dialog
                usarModuloArduino = true;
                Intent abreListaDispositivos = new Intent(IniciarActivity_DenseMatrix.this, ListaDispositivosBluetooth.class);
                startActivityForResult(abreListaDispositivos, SOLICITA_CONEXAO_BT);
                dialog.dismiss();
            }
        });
        solicitaConexaoArduino.setNegativeButton("NÃO", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                Toast.makeText(getApplicationContext(), "Aplicativo no modo MANUAL.\nSerá solicitado os dados lidos pelo sensor a cada ponto!", Toast.LENGTH_LONG).show();
                usarModuloArduino = false;
                if (meuBluetoothAdapter.isEnabled())
                    meuBluetoothAdapter.disable();
                // Do nothing
                dialog.dismiss();
            }
        });
        AlertDialog alert = solicitaConexaoArduino.create();
        alert.show();

        temporizador = new Timer();
        rotinaIMU = new TaskIMU();
        rotinaGPS = new TaskGPS();
        rotinaKalman = new TaskKalman();

        temporizador.schedule(rotinaIMU, 20, 20);
        temporizador.schedule(rotinaGPS, 1000, 1000);
        temporizador.schedule(rotinaKalman, 20, 20);

        /* Filtragem dos sinais lidos */
        T = 0.2;

        /*
        accel_filtrado[0] = 0;
        accel_filtrado[1] = 0;
        accel_filtrado[2] = 0;
        grav_filtrado[0] = 0;
        grav_filtrado[1] = 0;
        grav_filtrado[2] = 0;
        gyro_filtrado[0] = 0;
        gyro_filtrado[1] = 0;
        gyro_filtrado[2] = 0;
        magnet_filtrado[0] = 0;
        magnet_filtrado[1] = 0;
        magnet_filtrado[2] = 0;
        */

        roll_filtrado = 0;
        pitch_filtrado = 0;
        yaw_filtrado = 0;


        /* Carrega as variancias geradas a partir da calibracao */
        carregaVariancias();

        ibt_Iniciar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                debugGeraRelatorioDados = true;
                AlertDialog.Builder builder = new AlertDialog.Builder(IniciarActivity_DenseMatrix.this);
                builder.setTitle("Digite o nome do arquivo onde serão armazenados os dados:");

                // Set up the input
                final EditText input = new EditText(IniciarActivity_DenseMatrix.this);
                // Specify the type of input expected; this, for example, sets the input as a password, and will mask the text
                input.setInputType(InputType.TYPE_CLASS_TEXT);
                builder.setView(input);

                // Set up the buttons
                builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        fileNamePasseio = input.getText().toString();
                        debugGeraRelatorioDados = true;
                        Toast.makeText(IniciarActivity_DenseMatrix.this, "ARMAZENAMENTO DE DADOS INICIADO!", Toast.LENGTH_SHORT).show();

                    }
                });
                builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.cancel();
                        debugGeraRelatorioDados = true;
                        Toast.makeText(IniciarActivity_DenseMatrix.this, "ARMAZENAMENTO DE DADOS NÃO FOI INICIADO!", Toast.LENGTH_SHORT).show();
                    }
                });

                builder.show();

            }
        });

        ibt_Coletar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                /*debugGeraRelatorioDados = true;
                Toast.makeText(IniciarActivity.this, "INICIANDO COLETA!", Toast.LENGTH_SHORT).show();
                cont++;
                //Valor lido pelo sensor
                if (usarModuloArduino) {
                    //valorLido = BT.LeituraArduino;
                } else {
                    valorLido = "Valor lido:\n1";
                }


                //Coordenadas obtidas após filtragem com Kalman
                LatLng localAtual = new LatLng(lat, lon);
                //Adiciona um marcador no mapa com o valor da leitura do sensor naquele ponto
                MarkerOptions marcador = new MarkerOptions();
                marcador.position(localAtual);
                marcador.title(valorLido);
                meuMapa.addMarker(marcador);*/
            }
        });


        ibt_Finalizar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (debugGeraRelatorioDados){
                    debugGeraRelatorioDados = false;
                    ModuloExterno ext = new ModuloExterno(getApplicationContext());

                    if (ext.verificaArmazenamentoExternoEscrita()) {
                        dadosGPSLat = dadosGPSLat + "];";
                        dadosGPSLon= dadosGPSLon + "];";
                        dadosIMUacelX = dadosIMUacelX + "];";
                        dadosIMUacelY = dadosIMUacelY + "];";
                        dadosIMUacelZ = dadosIMUacelZ + "];";
                        dadosIMUmagX = dadosIMUmagX + "];";
                        dadosIMUmagY = dadosIMUmagY + "];";
                        dadosIMUmagZ = dadosIMUmagZ + "];";
                        dadosGravX = dadosGravX + "];";
                        dadosGravY = dadosGravY + "];";
                        dadosGravZ = dadosGravZ + "];";
                        dadosIMUgyroX = dadosIMUgyroX + "];";
                        dadosIMUgyroY = dadosIMUgyroY + "];";
                        dadosIMUgyroZ = dadosIMUgyroZ + "];";

                        File patch = ext.pegaDiretorioArquivo(folderDados);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameGPS,dadosGPSLat+"\n"+dadosGPSLon);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameIMUacelInercial,dadosIMUacelInercialX+"\n"+dadosIMUacelInercialY+"\n"+dadosIMUacelInercialZ);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameIMUacel,dadosIMUacelX+"\n"+dadosIMUacelY+"\n"+dadosIMUacelZ);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameIMUmag,dadosIMUmagX+"\n"+dadosIMUmagY+"\n"+dadosIMUmagZ);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameIMUgyro,dadosIMUgyroX+"\n"+dadosIMUgyroY+"\n"+dadosIMUgyroZ);
                        ext.escreveArquivo(patch.getPath(),cont+fileNameGrav,dadosGravX+"\n"+dadosGravY+"\n"+dadosGravZ);

                    } else{
                        Toast.makeText(IniciarActivity_DenseMatrix.this, "Impossivel escrever na memoria do dispositivo!", Toast.LENGTH_SHORT).show();
                    }

                    dadosGPSLat = "lat = [";
                    dadosGPSLon = "lon = [";
                    dadosIMUacelInercialX = "";
                    dadosIMUacelInercialY = "";
                    dadosIMUacelInercialZ = "";
                    dadosIMUacelX = "accX = [";
                    dadosIMUacelY = "accY = [";
                    dadosIMUacelZ = "accZ = [";
                    dadosIMUgyroX = "gyroX = [";
                    dadosIMUgyroY = "gyroY = [";
                    dadosIMUgyroZ = "gyroZ = [";
                    dadosIMUmagX = "magX = [";
                    dadosIMUmagY = "magY = [";
                    dadosIMUmagZ = "magZ = [";
                    dadosGravX = "gravX = [";
                    dadosGravY = "gravY = [";
                    dadosGravZ = "gravZ = [";
                    cont++;


                }
            }
        });



        /* Handler para receber dados BLUETOOTH */
        meuHandler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                /* Verifica se a mensagem recebida veio do BT */
                if (msg.what == MENSAGEM_RECEBIDA_BT) {
                    String recebidos = (String) msg.obj;
                    /*Adiciona o caracter recebido a mensagem */
                    dadosBluetooth.append(recebidos);
                    /* Verifica se o caracter de fim de mensagem foi recebido */
                    int fimMsg = dadosBluetooth.indexOf("}");
                    if (fimMsg > 0) {
                        /* Pega tudo que foi recebido ate o caracter de fim de mensagem */
                        String msgCompleta = dadosBluetooth.substring(0, fimMsg);
                        int tamanhoMsg = msgCompleta.length();
                        /* Verifica se o primeiro caracter recebido foi de inicio de mensagem */
                        if (dadosBluetooth.charAt(0) == '{') {
                            String msgFinal = dadosBluetooth.substring(1, tamanhoMsg);
                            /*Escreve a string recebida na tela */
                            Toast.makeText(getApplicationContext(), msgFinal, Toast.LENGTH_SHORT).show();
                        }
                        dadosBluetooth.delete(0, dadosBluetooth.length());
                    }
                }
            }
        };

        ActivityCompat.requestPermissions(this, new String[]{
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
        }, MY_PERMISSION_REQUEST_CODE);
    }


    public void carregaVariancias() {
        /*
        ModuloExterno ext = new ModuloExterno(getApplicationContext());
        String linha;
        File patch = ext.pegaDiretorioArquivo(folderCalibragem);
        File arquivo = new File(patch.getPath(), fileNameCalibragem);

        Toast.makeText(getApplicationContext(), patch.toString(), Toast.LENGTH_LONG).show();
        Toast.makeText(getApplicationContext(), arquivo.toString(), Toast.LENGTH_LONG).show();

        try {
            ext.escreveArquivo(patch.getPath(),cont+fileNameGPS,dadosGPSLat+"\n"+dadosGPSLon);
            BufferedReader buffer = new BufferedReader(new FileReader(arquivo));

            while((linha = buffer.readLine()) != null){
                Toast.makeText(getApplicationContext(),linha,Toast.LENGTH_SHORT).show();
            }

        }catch  (IOException e1) {
            Toast.makeText(getApplicationContext(), "Não foi possível localizar o arquivo de calibração! Por favor realize a calibração antes de prosseguir!", Toast.LENGTH_SHORT).show();
            e1.printStackTrace();
        }
        */
        varLat = 2.044866720572520e-09;
        varLon = 0;
        varAccX = 0;
        varAccY = 0;
        varAng = 0;
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        switch (requestCode) {
            case 10:
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    solicitaAtualizacaoGPS();
                }

            case MY_PERMISSION_REQUEST_CODE:
                if(grantResults.length > 0 && grantResults[0] ==
                        PackageManager.PERMISSION_GRANTED)
                    //Do your work
                    break;
                return;
        }

    }

    private void solicitaAtualizacaoGPS() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        managerGPS.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, listenerGPS);

        //Toast.makeText(this, "REQUEST!", Toast.LENGTH_SHORT).show();
    }

    /* Funcao do android que e chamada quando uma nova tela/janela e chamada e a mesma retorna um inteiro (startActivityForResult) */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        /*Verifica qual requisicao foi feita e o valor retornado */
        switch (requestCode) {
            case SOLICITA_CONEXAO_BT:
                if (resultCode == Activity.RESULT_OK) {
                    /* Caso tenha recebido o endereco MAC para realizar a conexao */
                    /* Salva o MAC retornado */
                    MAC = data.getExtras().getString(ListaDispositivosBluetooth.ENDERECO_MAC);
                    /* Cria um dispositivo com o endereco retornado */
                    meuDevice = meuBluetoothAdapter.getRemoteDevice(MAC);
                    /* Tenta criar um socket para comunicacao */
                    try {
                        /* Cria um socket utilizando o dispositivo externo passando o UUID desejado (SERIAL) */
                        meuSocket = meuDevice.createRfcommSocketToServiceRecord(UUID_SERIAL_PORT);
                        /* Tenta estabilizar a conexao */
                        meuSocket.connect();
                        conectado = true;
                        /*Cria uma thread */
                        connectedThread = new ConnectedThread(meuSocket);
                        connectedThread.start();
                        Toast.makeText(getApplicationContext(), "Conectado !", Toast.LENGTH_SHORT).show();
                    } catch (IOException erro) {
                        /* Caso ocorra algum problema durante a conexao */
                        conectado = false;
                        Toast.makeText(getApplicationContext(), "Ocorreu um erro ao tentar conectar" + "\n" + "ERRO :" + erro, Toast.LENGTH_LONG).show();
                        usarModuloArduino = false;
                    }
                } else {
                    /* Caso nenhum endereco MAC tenha sido retornado */
                    Toast.makeText(getApplicationContext(), "Falha ao retornar o endereço MAC do dispositivo selecionado.", Toast.LENGTH_SHORT).show();
                }
        }
    }

    /* Mapa */
    @Override
    public void onMapReady(GoogleMap googleMap) {

        meuMapa = googleMap;
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        meuMapa.setMyLocationEnabled(true);
        meuMapa.getUiSettings().setMyLocationButtonEnabled(true);
        meuMapa.getUiSettings().setZoomControlsEnabled(true);
        //meuMapa.moveCamera();
        //Location localGPS = gps.pegaLocalicazaoGPS();
        //LatLng meuLocal = new LatLng(localGPS.getLatitude(),localGPS.getLongitude());
        //meuMapa.moveCamera(CameraUpdateFactory.newLatLng(meuLocal));

        //atualizaEscalaresConversao();

    }

    /*
    public void atualizaEscalaresConversao () {
        //Define as referencias antes de iniciar os calculos

        double refLat;
        double refLon;
        double novoX;
        double novoY;
        double difLon;
        double difLat;

        refLat = lat;
        refLon = lon;
        Deg2UTM refUTM = new Deg2UTM(refLat,refLon);

//        UTM2Deg pos = new UTM2Deg(refUTM.Zone+" "+refUTM.Letter+" "+refUTM.Easting+" "+refUTM.Northing);


        //Calcula ganho na direção do eixo X
        novoX = refUTM.Easting + 5000;
        UTM2Deg novoLon = new UTM2Deg(refUTM.Zone+" "+refUTM.Letter+" "+novoX+" "+refUTM.Northing);

        difLon = refLon - novoLon.longitude;
        m2gX = difLon/5000;


        //Calcula ganho na direção do eixo Y
        novoY = refUTM.Northing + 5000;
        UTM2Deg novoLat = new UTM2Deg(refUTM.Zone+" "+refUTM.Letter+" "+refUTM.Easting+" "+novoY);
        difLat = refLat - novoLat.latitude;
        m2gY = difLat/5000;
    }
    */

    private class Deg2UTM
    {
        double Easting;
        double Northing;
        int Zone;
        char Letter;
        private  Deg2UTM(double Lat,double Lon)
        {
            Zone= (int) Math.floor(Lon/6+31);
            if (Lat<-72)
                Letter='C';
            else if (Lat<-64)
                Letter='D';
            else if (Lat<-56)
                Letter='E';
            else if (Lat<-48)
                Letter='F';
            else if (Lat<-40)
                Letter='G';
            else if (Lat<-32)
                Letter='H';
            else if (Lat<-24)
                Letter='J';
            else if (Lat<-16)
                Letter='K';
            else if (Lat<-8)
                Letter='L';
            else if (Lat<0)
                Letter='M';
            else if (Lat<8)
                Letter='N';
            else if (Lat<16)
                Letter='P';
            else if (Lat<24)
                Letter='Q';
            else if (Lat<32)
                Letter='R';
            else if (Lat<40)
                Letter='S';
            else if (Lat<48)
                Letter='T';
            else if (Lat<56)
                Letter='U';
            else if (Lat<64)
                Letter='V';
            else if (Lat<72)
                Letter='W';
            else
                Letter='X';
            Easting=0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180))/(1-Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180)))*0.9996*6399593.62/Math.pow((1+Math.pow(0.0820944379, 2)*Math.pow(Math.cos(Lat*Math.PI/180), 2)), 0.5)*(1+ Math.pow(0.0820944379,2)/2*Math.pow((0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180))/(1-Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180)))),2)*Math.pow(Math.cos(Lat*Math.PI/180),2)/3)+500000;
            Easting=Math.round(Easting*100)*0.01;
            Northing = (Math.atan(Math.tan(Lat*Math.PI/180)/Math.cos((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))-Lat*Math.PI/180)*0.9996*6399593.625/Math.sqrt(1+0.006739496742*Math.pow(Math.cos(Lat*Math.PI/180),2))*(1+0.006739496742/2*Math.pow(0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))/(1-Math.cos(Lat*Math.PI/180)*Math.sin((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))),2)*Math.pow(Math.cos(Lat*Math.PI/180),2))+0.9996*6399593.625*(Lat*Math.PI/180-0.005054622556*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+4.258201531e-05*(3*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2))/4-1.674057895e-07*(5*(3*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2))/4+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2)*Math.pow(Math.cos(Lat*Math.PI/180),2))/3);
            if (Letter<'M')
                Northing = Northing + 10000000;
            Northing=Math.round(Northing*100)*0.01;
        }
    }

    private class UTM2Deg
    {
        double latitude;
        double longitude;
        private  UTM2Deg(String UTM)
        {
            String[] parts=UTM.split(" ");
            int Zone=Integer.parseInt(parts[0]);
            char Letter=parts[1].toUpperCase(Locale.ENGLISH).charAt(0);
            double Easting=Double.parseDouble(parts[2]);
            double Northing=Double.parseDouble(parts[3]);
            double Hem;
            if (Letter>'M')
                Hem='N';
            else
                Hem='S';
            double north;
            if (Hem == 'S')
                north = Northing - 10000000;
            else
                north = Northing;
            latitude = (north/6366197.724/0.9996+(1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)-0.006739496742*Math.sin(north/6366197.724/0.9996)*Math.cos(north/6366197.724/0.9996)*(Math.atan(Math.cos(Math.atan(( Math.exp((Easting - 500000) / (0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting - 500000) / (0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3))-Math.exp(-(Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*( 1 -  0.006739496742*Math.pow((Easting - 500000) / (0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3)))/2/Math.cos((north-0.9996*6399593.625*(north/6366197.724/0.9996-0.006739496742*3/4*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.pow(0.006739496742*3/4,2)*5/3*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996 )/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4-Math.pow(0.006739496742*3/4,3)*35/27*(5*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/3))/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2))+north/6366197.724/0.9996)))*Math.tan((north-0.9996*6399593.625*(north/6366197.724/0.9996 - 0.006739496742*3/4*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.pow(0.006739496742*3/4,2)*5/3*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996 )*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4-Math.pow(0.006739496742*3/4,3)*35/27*(5*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/3))/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2))+north/6366197.724/0.9996))-north/6366197.724/0.9996)*3/2)*(Math.atan(Math.cos(Math.atan((Math.exp((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3))-Math.exp(-(Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3)))/2/Math.cos((north-0.9996*6399593.625*(north/6366197.724/0.9996-0.006739496742*3/4*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.pow(0.006739496742*3/4,2)*5/3*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4-Math.pow(0.006739496742*3/4,3)*35/27*(5*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/3))/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2))+north/6366197.724/0.9996)))*Math.tan((north-0.9996*6399593.625*(north/6366197.724/0.9996-0.006739496742*3/4*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.pow(0.006739496742*3/4,2)*5/3*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4-Math.pow(0.006739496742*3/4,3)*35/27*(5*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/3))/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2))+north/6366197.724/0.9996))-north/6366197.724/0.9996))*180/Math.PI;
            latitude=Math.round(latitude*10000000);
            latitude=latitude/10000000;
            longitude =Math.atan((Math.exp((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3))-Math.exp(-(Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2)/3)))/2/Math.cos((north-0.9996*6399593.625*( north/6366197.724/0.9996-0.006739496742*3/4*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.pow(0.006739496742*3/4,2)*5/3*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2* north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4-Math.pow(0.006739496742*3/4,3)*35/27*(5*(3*(north/6366197.724/0.9996+Math.sin(2*north/6366197.724/0.9996)/2)+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/4+Math.sin(2*north/6366197.724/0.9996)*Math.pow(Math.cos(north/6366197.724/0.9996),2)*Math.pow(Math.cos(north/6366197.724/0.9996),2))/3)) / (0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2))))*(1-0.006739496742*Math.pow((Easting-500000)/(0.9996*6399593.625/Math.sqrt((1+0.006739496742*Math.pow(Math.cos(north/6366197.724/0.9996),2)))),2)/2*Math.pow(Math.cos(north/6366197.724/0.9996),2))+north/6366197.724/0.9996))*180/Math.PI+Zone*6-183;
            longitude=Math.round(longitude*10000000);
            longitude=longitude/10000000;
        }
    }


    /* BLUETOOTH */
    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            meuSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];  // buffer store for the stream
            int bytes; // bytes returned from read()

            // Keep listening to the InputStream until an exception occurs
            while (true) {
                try {
                    // Read from the InputStream
                    bytes = mmInStream.read(buffer);

                    String recebidos = new String(buffer, 0, bytes);

                    // Send the obtained bytes to the UI activity
                    meuHandler.obtainMessage(MENSAGEM_RECEBIDA_BT, bytes, -1, recebidos).sendToTarget();
                } catch (IOException e) {
                    break;
                }
            }
        }

        /* Call this from the main activity to send data to the remote device */
        public void enviar(String dado) {
            byte[] msgBuffer = dado.getBytes();
            try {
                mmOutStream.write(msgBuffer);
            } catch (IOException e) {
            }
        }

    }


    /* IMU */
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, accel_array, 0, accel_array.length);

            // Copy values into the bean
            acc = new Accelerometer();
            acc.setX((double) accel_array[0]);
            acc.setY((double) accel_array[1]);
            acc.setZ((double) accel_array[2]);

        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            System.arraycopy(event.values, 0, gyro_array, 0, gyro_array.length);

            // Copy values into the bean
            gyro = new Gyroscope();
            gyro.setX((double) gyro_array[0]);
            gyro.setY((double) gyro_array[1]);
            gyro.setZ((double) gyro_array[2]);

        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, magnet_array, 0, magnet_array.length);

            // Copy values into the bean
            mag = new Magnetometer();
            mag.setX((double) magnet_array[0]);
            mag.setY((double) magnet_array[1]);
            mag.setZ((double) magnet_array[2]);

        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            System.arraycopy(event.values, 0, grav_array, 0, grav_array.length);

            // Copy values into the bean
            grav = new Gravity();
            grav.setX((double) grav_array[0]);
            grav.setY((double) grav_array[1]);
            grav.setZ((double) grav_array[2]);
        }

    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /* Interrupcoes */

    class TaskKalman extends TimerTask {
        @Override
        public void run() {

            millisTempoAgoraGPS = System.currentTimeMillis();
            dt = millisTempoAgoraGPS - millisTempoAnteriorGPS;

            /* Filtragem dos valores obtidos com a IMU  */
            /*
            gyro_filtrado[0] = (float) ((1-(dt/T))*gyro_filtrado[0]+gyro[0]*(dt/T));
            gyro_filtrado[1] = (float) ((1-(dt/T))*gyro_filtrado[1]+gyro[1]*(dt/T));
            gyro_filtrado[2] = (float) ((1-(dt/T))*gyro_filtrado[2]+gyro[2]*(dt/T));
            //gyro_filtrado[3] = (float) ((1-(dt/T))*gyro_filtrado[3]+gyro[3]*(dt/T));

            accel_filtrado[0] = (float) ((1-(dt/T))*accel_filtrado[0]+accel[0]*(dt/T));
            accel_filtrado[1] = (float) ((1-(dt/T))*accel_filtrado[1]+accel[1]*(dt/T));
            accel_filtrado[2] = (float) ((1-(dt/T))*accel_filtrado[2]+accel[2]*(dt/T));
            //accel_filtrado[3] = (float) ((1-(dt/T))*accel_filtrado[3]+accel[3]*(dt/T));

            magnet_filtrado[0] = (float) ((1-(dt/T))*magnet_filtrado[0]+magnet[0]*(dt/T));
            magnet_filtrado[1] = (float) ((1-(dt/T))*magnet_filtrado[1]+magnet[1]*(dt/T));
            magnet_filtrado[2] = (float) ((1-(dt/T))*magnet_filtrado[2]+magnet[2]*(dt/T));
            //magnet_filtrado[3] = (float) ((1-(dt/T))*magnet_filtrado[3]+magnet[3]*(dt/T));

            grav_filtrado[0] = (float) ((1-(dt/T))*grav_filtrado[0]+grav[0]*(dt/T));
            grav_filtrado[1] = (float) ((1-(dt/T))*grav_filtrado[1]+grav[1]*(dt/T));
            grav_filtrado[2] = (float) ((1-(dt/T))*grav_filtrado[2]+grav[2]*(dt/T));
            //grav_filtrado[3] = (float) ((1-(dt/T))*grav_filtrado[3]+grav[3]*(dt/T));
            */

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

            /* Gera matriz de rotacao a partir dos valores fitlrados */

            //sensManager.getRotationMatrix(matrizRotacao, null, accel_filtrado, magnet_filtrado);
            //sensManager.getOrientation(matrizRotacao, vetorOrientacao);

            RM = Util.getRotationMatrix(acc_filtrado, mag_filtrado, gyro_filtrado);
            vo = Util.getOrientation(RM);

            /* Gera matriz de cossenos diretores */
            //geraDCM(roll_filtrado,pitch_filtrado,yaw_filtrado);
            yaw_a = vo.get(0, 0);
            pitch_a = vo.get(0, 1);
            roll_a = vo.get(0, 2);

            quaternion_a = Util.eulerToQuaternion(new double [] {yaw_a, pitch_a, roll_a});
            /* ------------------------------------------------------------------ */

            roll_filtrado = (1-(dt/T)) * roll_filtrado + roll_a * (dt/T);
            pitch_filtrado = (1-(dt/T)) * pitch_filtrado + pitch_a * (dt/T);
            yaw_filtrado = (1-(dt/T)) * yaw_filtrado + yaw_a * (dt/T);

            /* ----- Matriz de rotacao do sistema ---------------------------------- */
            DCM = Util.getDirectCosineMatrix(roll_filtrado, pitch_filtrado, yaw_filtrado);
            AVM = Util.getAngularVelocityMatrix(gyro_filtrado.getX(), gyro_filtrado.getY(), gyro_filtrado.getZ());
            RM = DCM.mmul(AVM);
        }
    }

/*
    protected void geraDCM(float r, float p, float y){
        DCM[0] = (float) (cos(y)*cos(p));
        DCM[1] = (float) (cos(y)*sin(p)*sin(r)-sin(y)*cos(r));
        DCM[2] = (float) (cos(y)*sin(p)*cos(r)-sin(y)*sin(r));

        DCM[3] = (float) (sin(y)*cos(p));
        DCM[4] = (float) (sin(y)*sin(p)*sin(r)-cos(y)*cos(r)*cos(p));
        DCM[5] = (float) (sin(y)*sin(p)*cos(r)-cos(y)*sin(r));

        DCM[6] = (float) (-sin(p));
        DCM[7] = (float) (cos(p)*sin(r));
        DCM[8] = (float) (cos(p)*cos(r));
    }
*/

    class TaskGPS extends TimerTask {

        @Override
        public void run() {

            Location localGPS = managerGPS.getLastKnownLocation(LocationManager.GPS_PROVIDER);

            if (localGPS != null) {
                //Caso positivo, pega os valores de latidude e longitude
                lat = localGPS.getLatitude();
                lon = localGPS.getLongitude();
                //alt = localGPS.getAltitude();

                Z = new DenseMatrix(new double [][]
                        {
                                {lon},
                                {lat},
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

                //Deg2UTM posUTM = new Deg2UTM(lat,lon);
                //UTM2Deg pos = new UTM2Deg(posUTM.Zone+" "+posUTM.Letter+" "+posUTM.Easting+" "+posUTM.Northing);

                //if (debugGeraRelatorioDados) {
                //    dadosGPSLat = dadosGPSLat + lat + "\n";
                //    dadosGPSLon = dadosGPSLon + lon + "\n";
                //}

                /*
                localGPS.set
                Toast.makeText(getApplicationContext(),"Lat : "+lat+"   Lon : "+lon, Toast.LENGTH_LONG).show();
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

                            //Primeira linha da matriz
                    matrizECEF2NED[0] = (float) ((-1)*sin(gpsLat_r)*cos(gpsLon_r));
                    matrizECEF2NED[1] = (float) ((-1)*sin(gpsLat_r)*sin(gpsLon_r));
                    matrizECEF2NED[2] = (float) cos(gpsLat_r);
                            //Segunda linha da matriz
                    matrizECEF2NED[3] = (float) ((-1)*sin(gpsLon_r));
                    matrizECEF2NED[4] = (float) cos(gpsLon_r);
                    matrizECEF2NED[5] = 0;
                            //Terceira linha da matriz
                    matrizECEF2NED[6] = (float) ((-1)*cos(gpsLat_r)*cos(gpsLon_r));
                    matrizECEF2NED[7] = (float) ((-1)*cos(gpsLat_r)*sin(gpsLon_r));
                    matrizECEF2NED[8] = (float) ((-1)*sin(gpsLat_r));

                    primeiraLeitura = false;
                }
                else {
                    nedN = (matrizECEF2NED[0]*(ecefX-firstECEFX)) + (matrizECEF2NED[1]*(ecefY-firstECEFY)) + (matrizECEF2NED[2]*(ecefZ-firstECEFZ));
                    nedE = (matrizECEF2NED[3]*(ecefX-firstECEFX)) + (matrizECEF2NED[4]*(ecefY-firstECEFY)) + (matrizECEF2NED[5]*(ecefZ-firstECEFZ));
                    nedD = (matrizECEF2NED[6]*(ecefX-firstECEFX)) + (matrizECEF2NED[7]*(ecefY-firstECEFY)) + (matrizECEF2NED[8]*(ecefZ-firstECEFZ));

                } */

                /* Chama funcao do filtro de Kalman */
                millisTempoAgoraGPS = System.currentTimeMillis();
                dt = millisTempoAgoraGPS - millisTempoAnteriorGPS;
                millisTempoAnteriorGPS = System.currentTimeMillis();

                FiltroKalman(1, dt); // -> caso 1 GPS
            } else {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        tv_LatitudeAtual.setText("NULO");
                        tv_LongitudeAtual.setText("NULO");
                        tv_ValorLido.setText("NULO");
                    }
                });
            }


            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    tv_LatitudeAtual.setText(String.valueOf(lat));
                    tv_LongitudeAtual.setText(String.valueOf(lon));
                    tv_ValorLido.setText("-");
                }
            });

        }
    }


    class TaskIMU extends TimerTask {

        @Override
        public void run() {

            //sensManager.getRotationMatrix(matrizRotacao, null,
                    //accel, magnet);
            /*
            ax_inercial = (matrizRotacao[0]*(accel[0]-grav[0])) + (matrizRotacao[1]*(accel[1]-grav[1])) + (matrizRotacao[2]*(accel[2]-grav[2]));
            ay_inercial = (matrizRotacao[3]*(accel[0]-grav[0])) + (matrizRotacao[4]*(accel[1]-grav[1])) + (matrizRotacao[5]*(accel[2]-grav[2]));
            az_inercial = (matrizRotacao[6]*(accel[0]-grav[0])) + (matrizRotacao[7]*(accel[1]-grav[1])) + (matrizRotacao[8]*(accel[2]-grav[2]));
            */

            acc_rot.setX(RM.get(0,0) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,0) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,0) * (acc_filtrado.getZ() - grav_filtrado.getZ()));
            acc_rot.setY(RM.get(0,1) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,1) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,1) * (acc_filtrado.getZ() - grav_filtrado.getZ()));
            acc_rot.setZ(RM.get(0,2) * (acc_filtrado.getX() - grav_filtrado.getX()) + RM.get(1,2) * (acc_filtrado.getY() - grav_filtrado.getY()) + RM.get(2,2) * (acc_filtrado.getZ() - grav_filtrado.getZ()));

            gyro_rot.setX(RM.get(0,0) * (gyro.getX()) + RM.get(1,0) * (gyro.getY()) + RM.get(2,0) * (gyro.getZ()));
            gyro_rot.setY(RM.get(0,1) * (gyro.getX()) + RM.get(1,1) * (gyro.getY()) + RM.get(2,1) * (gyro.getZ()));
            gyro_rot.setZ(RM.get(0,2) * (gyro.getX()) + RM.get(1,2) * (gyro.getY()) + RM.get(2,2) * (gyro.getZ()));

            mag_rot.setX(RM.get(0,0) * (mag.getX()) + RM.get(1,0) * (mag.getY()) + RM.get(2,0) * (mag.getZ()));
            mag_rot.setY(RM.get(0,1) * (mag.getX()) + RM.get(1,1) * (mag.getY()) + RM.get(2,1) * (mag.getZ()));
            mag_rot.setZ(RM.get(0,2) * (mag.getX()) + RM.get(1,2) * (mag.getY()) + RM.get(2,2) * (mag.getZ()));

            accXFinal = acc_rot.getX();
            accYFinal = acc_rot.getY();

            accXFinal = accXFinal * m2g.getX();
            accYFinal = accYFinal * m2g.getY();

            //sensManager.getOrientation(matrizRotacao, angulosOrientacao);
            /*
            if (debugGeraRelatorioDados) {
                //dadosIMUacelInercial = "";
                dadosIMUacelX = dadosIMUacelX + accel[0] + "\n";
                dadosIMUacelY = dadosIMUacelY + accel[1] + "\n";
                dadosIMUacelZ = dadosIMUacelZ + accel[2] + "\n";

                dadosIMUmagX = dadosIMUmagX + magnet[0] + "\n";
                dadosIMUmagY = dadosIMUmagY + magnet[1] + "\n";
                dadosIMUmagZ = dadosIMUmagZ + magnet[2] + "\n";

                dadosIMUgyroX = dadosIMUgyroX + gyro[0] +"\n";
                dadosIMUgyroY = dadosIMUgyroY + gyro[1] +"\n";
                dadosIMUgyroZ = dadosIMUgyroZ + gyro[2] +"\n";

                dadosGravX = dadosGravX + grav[0] + "\n";
                dadosGravY = dadosGravY + grav[1] + "\n";
                dadosGravZ = dadosGravZ + grav[2] + "\n";
            }
            */

            runOnUiThread(new Runnable() {
                @Override
                public void run() {

                    /*
                    if (Math.abs(ax_inercial) > 5){
                        tv_LatitudeAtual.setTextColor(Color.parseColor("#E50C0C"));
                    } else if (Math.abs(ax_inercial) > 3) {
                        tv_LatitudeAtual.setTextColor(Color.parseColor("#EBE812"));
                    } else if (Math.abs(ax_inercial) > 1) {
                        tv_LatitudeAtual.setTextColor(Color.parseColor("#4AF510"));
                    } else
                        tv_LatitudeAtual.setTextColor(Color.parseColor("#000000"));

                    if (Math.abs(ay_inercial) > 5){
                        tv_LongitudeAtual.setTextColor(Color.parseColor("#E50C0C"));
                    } else if (Math.abs(ay_inercial) > 3) {
                        tv_LongitudeAtual.setTextColor(Color.parseColor("#EBE812"));
                    } else if (Math.abs(ay_inercial) > 1) {
                        tv_LongitudeAtual.setTextColor(Color.parseColor("#4AF510"));
                    } else
                        tv_LongitudeAtual.setTextColor(Color.parseColor("#000000"));

                    if (Math.abs(az_inercial) > 5){
                        tv_ValorLido.setTextColor(Color.parseColor("#E50C0C"));
                    } else if (Math.abs(az_inercial) > 3) {
                        tv_ValorLido.setTextColor(Color.parseColor("#EBE812"));
                    } else if (Math.abs(az_inercial) > 1) {
                        tv_ValorLido.setTextColor(Color.parseColor("#4AF510"));
                    } else
                        tv_ValorLido.setTextColor(Color.parseColor("#000000"));
                    */
                    //tv_LatitudeAtual.setText(String.valueOf(angulosOrientacao[0]*180/Math.PI));  //Azimuth
                    // tv_LongitudeAtual.setText(String.valueOf(angulosOrientacao[1]*180/Math.PI)); //Pitch
                    //tv_ValorLido.setText(String.valueOf(angulosOrientacao[2]*180/Math.PI)); //Rol
                }
            });

            /* Chama funcao filtro de Kalman */
           /* millisTempoAgoraIMU = System.currentTimeMillis();
            dt = millisTempoAgoraIMU - millisTempoAnteriorIMU;
            FiltroKalman(2, varAcelX, varAcelY, varAcelZ); // -> caso 2 IMU
            millisTempoAnteriorIMU = System.currentTimeMillis();*/
        }
    }



    /* ------------------------------------------------------------------------------------------------

        IMPORTANTE !!!!!!!!!!!!!!!!

        Para chamda da IMU deve-se apensar atualizar o estado (predição)

        Para chamada do GPS corrige a prediçao e atualiza valores do ganho etc...

        -----------------------------------------------------------------------------------------------
     */
    //protected void FiltroKalman (int caso, double dt, double var1, double var2, double var3, double var4) {
    protected void FiltroKalman (int caso, double dt) {
        int i;
        int j;
        double det;

        /* 1 - (PREDICAO) Calcula o estado a partir do modelo
        *  X_estimado = A*xanterior
        */

        /* Define as matrizes H e R */
        /*
        if (caso == 1)
        {
            H[0][0]=0; H[0][1]=0; H[0][2]=0; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=1; H[0][7]=0; H[0][8]=0;
            H[0][0]=0; H[0][1]=0; H[0][2]=0; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=0; H[0][7]=1; H[0][8]=0;
            H[0][0]=0; H[0][1]=0; H[0][2]=0; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=0; H[0][7]=0; H[0][8]=1;
        }
        if (caso == 2)
        {
            H[0][0]=1; H[0][1]=0; H[0][2]=0; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=0; H[0][7]=0; H[0][8]=0;
            H[0][0]=0; H[0][1]=1; H[0][2]=0; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=0; H[0][7]=0; H[0][8]=0;
            H[0][0]=0; H[0][1]=0; H[0][2]=1; H[0][3]=0; H[0][4]=0; H[0][5]=0; H[0][6]=0; H[0][7]=0; H[0][8]=0;
        }

        R[0][0]=var1; R[0][1]=0;    R[0][2]=0;
        R[1][0]=0;    R[1][1]=var2; R[1][2]=0;
        R[2][0]=0;    R[2][1]=0;    R[2][2]=var3;
        */

        /* 1 - (PREDICAO) Calcula o estado a partir do modelo
        *  Xp = A*x
        */
        /*
        state_pred[0][0] = A[0][0]*state_ant[0][0] + A[0][1]*state_ant[1][0] + A[0][2]*state_ant[2][0] + A[0][3]*state_ant[3][0] + A[0][4]*state_ant[4][0] + A[0][5]*state_ant[5][0] + A[0][6]*state_ant[6][0] + A[0][7]*state_ant[7][0] + A[0][8]*state_ant[8][0];
        state_pred[1][0] = A[1][0]*state_ant[0][0] + A[1][1]*state_ant[1][0] + A[1][2]*state_ant[2][0] + A[1][3]*state_ant[3][0] + A[1][4]*state_ant[4][0] + A[1][5]*state_ant[5][0] + A[1][6]*state_ant[6][0] + A[1][7]*state_ant[7][0] + A[1][8]*state_ant[8][0];
        state_pred[2][0] = A[2][0]*state_ant[0][0] + A[2][1]*state_ant[1][0] + A[2][2]*state_ant[2][0] + A[2][3]*state_ant[3][0] + A[2][4]*state_ant[4][0] + A[2][5]*state_ant[5][0] + A[2][6]*state_ant[6][0] + A[2][7]*state_ant[7][0] + A[2][8]*state_ant[8][0];
        state_pred[3][0] = A[3][0]*state_ant[0][0] + A[3][1]*state_ant[1][0] + A[3][2]*state_ant[2][0] + A[3][3]*state_ant[3][0] + A[3][4]*state_ant[4][0] + A[3][5]*state_ant[5][0] + A[3][6]*state_ant[6][0] + A[3][7]*state_ant[7][0] + A[3][8]*state_ant[8][0];
        state_pred[4][0] = A[4][0]*state_ant[0][0] + A[4][1]*state_ant[1][0] + A[4][2]*state_ant[2][0] + A[4][3]*state_ant[3][0] + A[4][4]*state_ant[4][0] + A[4][5]*state_ant[5][0] + A[4][6]*state_ant[6][0] + A[4][7]*state_ant[7][0] + A[4][8]*state_ant[8][0];
        state_pred[5][0] = A[5][0]*state_ant[0][0] + A[5][1]*state_ant[1][0] + A[5][2]*state_ant[2][0] + A[5][3]*state_ant[3][0] + A[5][4]*state_ant[4][0] + A[5][5]*state_ant[5][0] + A[5][6]*state_ant[6][0] + A[5][7]*state_ant[7][0] + A[5][8]*state_ant[8][0];
        state_pred[6][0] = A[6][0]*state_ant[0][0] + A[6][1]*state_ant[1][0] + A[6][2]*state_ant[2][0] + A[6][3]*state_ant[3][0] + A[6][4]*state_ant[4][0] + A[6][5]*state_ant[5][0] + A[6][6]*state_ant[6][0] + A[6][7]*state_ant[7][0] + A[6][8]*state_ant[8][0];
        state_pred[7][0] = A[7][0]*state_ant[0][0] + A[7][1]*state_ant[1][0] + A[7][2]*state_ant[2][0] + A[7][3]*state_ant[3][0] + A[7][4]*state_ant[4][0] + A[7][5]*state_ant[5][0] + A[7][6]*state_ant[6][0] + A[7][7]*state_ant[7][0] + A[7][8]*state_ant[8][0];
        state_pred[8][0] = A[8][0]*state_ant[0][0] + A[8][1]*state_ant[1][0] + A[8][2]*state_ant[2][0] + A[8][3]*state_ant[3][0] + A[8][4]*state_ant[4][0] + A[8][5]*state_ant[5][0] + A[8][6]*state_ant[6][0] + A[8][7]*state_ant[7][0] + A[8][8]*state_ant[8][0];
        */
        x_est = A.mmul(x_ant);

        /* 2 -
        *  P = A * P * transpose(A) + Q
        */
            /*
            for (i=0; i<9; i++)
            {
                for (j=0; j<9; j++)
                {  // A * P
                    aux[i][j] = A[i][0]*P[0][j] + A[i][1]*P[1][j] + A[i][2]*P[2][j] + A[i][3]*P[3][j] + A[i][4]*P[4][j] + A[i][5]*P[5][j] + A[i][6]*P[6][j] + A[i][7]*P[7][j] + A[i][8]*P[8][j];
                }
            }

            for (i=0; i<9; i++)
            {
                for(j=0; j<9; j++)
                { // A * P * transpose(A)
                    P[i][j] = aux[i][0]*A[j][0] + aux[i][1]*A[j][1] + aux[i][2]*A[j][2] + aux[i][3]*A[j][3] + aux[i][4]*A[j][4] + aux[i][5]*A[j][5] + aux[i][6]*A[j][6] + aux[i][7]*A[j][7] + aux[i][8]*A[j][8];
                }
            }

            for (i=0; i<9; i++)
            {
                for (j=0; j<9; j++)
                {   // A * P * transpose(A) + Q
                    P[i][j] = P[i][j] + Q[i][j];
                }
            }
            */
        P = A.mmul(P).mmul(A.t()).add(Q);

        /* 3 - Calcula o ganho de Kalman
         *  K =  P * tranpose(H)
         *     --------------------
         *     H*P*transpose(H) + R
         */



        // H*P*transpose(H) + R
            /*
            if (caso == 1)
            {
                aux[0][0] = P[6][6] + R[0][0];  aux[0][1] = P[6][7] + R[0][1];   aux[0][2] = P[6][8] + R[0][2];
                aux[1][0] = P[7][6] + R[1][0];  aux[1][1] = P[7][7] + R[1][1];   aux[1][2] = P[7][8] + R[1][2];
                aux[2][0] = P[8][6] + R[2][0];  aux[2][1] = P[8][7] + R[2][1];   aux[2][2] = P[8][8] + R[2][2];
            }
            if (caso == 2)
            {
                aux[0][0] = P[0][0] + R[0][0];  aux[0][1] = P[0][1] + R[0][1];   aux[0][2] = P[0][2] + R[0][2];
                aux[1][0] = P[1][0] + R[1][0];  aux[1][1] = P[1][1] + R[1][1];   aux[1][2] = P[1][2] + R[1][2];
                aux[2][0] = P[2][0] + R[2][0];  aux[2][1] = P[2][1] + R[2][1];   aux[2][2] = P[2][2] + R[2][2];
            }

            // (H*P*transpose(H)+R)^-1
            det = (aux[0][0]*aux[1][1]*aux[2][2]) + (aux[0][1]*aux[1][2]*aux[2][0]) + (aux[1][0]*aux[2][1]*aux[0][2]) - (aux[0][2]*aux[1][1]*aux[2][0]) - (aux[0][1]*aux[1][0]*aux[2][2]) - (aux[0][0]*aux[1][2]*aux[2][1]);
            aux1[0][0] = ((aux[1][1]*aux[2][2])-(aux[1][2]*aux[2][1]))/det;
            aux1[0][1] = (((aux[1][0]*aux[2][2])-(aux[1][2]*aux[2][0]))*(-1))/det;
            aux1[0][2] = ((aux[1][0]*aux[2][1])-(aux[1][1]*aux[2][0]))/det;
            aux1[1][0] = (((aux[0][1]*aux[2][2])-(aux[0][2]*aux[2][1]))*(-1))/det;
            aux1[1][1] = ((aux[0][0]*aux[2][2])-(aux[0][2]*aux[2][0]))/det;
            aux1[1][2] = (((aux[1][0]*aux[2][1])-(aux[1][1]*aux[2][0]))*(-1))/det;
            aux1[2][0] = ((aux[0][1]*aux[1][2])-(aux[0][2]*aux[1][1]))/det;
            aux1[2][1] = (((aux[0][0]*aux[1][2])-(aux[0][2]*aux[1][0]))*(-1))/det;
            aux1[2][2] = ((aux[0][0]*aux[1][1])-(aux[0][1]*aux[1][0]))/det;

            // P*H^t
            aux[0][0] = P[0][0]*H[0][0] + P[0][1]*H[0][1] + P[0][2]*H[0][2] + P[0][3]*H[0][3] + P[0][4]*H[0][4] + P[0][5]*H[0][5] + P[0][6]*H[0][6] + P[0][7]*H[0][7] + P[0][8]*H[0][8];
            aux[0][1] = P[0][0]*H[1][0] + P[0][1]*H[1][1] + P[0][2]*H[1][2] + P[0][3]*H[1][3] + P[0][4]*H[1][4] + P[0][5]*H[1][5] + P[0][6]*H[1][6] + P[0][7]*H[1][7] + P[0][8]*H[1][8];
            aux[0][2] = P[0][0]*H[2][0] + P[0][1]*H[2][1] + P[0][2]*H[2][2] + P[0][3]*H[2][3] + P[0][4]*H[2][4] + P[0][5]*H[2][5] + P[0][6]*H[2][6] + P[0][7]*H[2][7] + P[0][8]*H[2][8];
            aux[1][0] = P[1][0]*H[0][0] + P[1][1]*H[0][1] + P[1][2]*H[0][2] + P[1][3]*H[0][3] + P[1][4]*H[0][4] + P[1][5]*H[0][5] + P[1][6]*H[0][6] + P[1][7]*H[0][7] + P[1][8]*H[0][8];
            aux[1][1] = P[1][0]*H[1][0] + P[1][1]*H[1][1] + P[1][2]*H[1][2] + P[1][3]*H[1][3] + P[1][4]*H[1][4] + P[1][5]*H[1][5] + P[1][6]*H[1][6] + P[1][7]*H[1][7] + P[1][8]*H[1][8];
            aux[1][2] = P[1][0]*H[2][0] + P[1][1]*H[2][1] + P[1][2]*H[2][2] + P[1][3]*H[2][3] + P[1][4]*H[2][4] + P[1][5]*H[2][5] + P[1][6]*H[2][6] + P[1][7]*H[2][7] + P[1][8]*H[2][8];
            aux[2][0] = P[2][0]*H[0][0] + P[2][1]*H[0][1] + P[2][2]*H[0][2] + P[2][3]*H[0][3] + P[2][4]*H[0][4] + P[2][5]*H[0][5] + P[2][6]*H[0][6] + P[2][7]*H[0][7] + P[2][8]*H[0][8];
            aux[2][1] = P[2][0]*H[1][0] + P[2][1]*H[1][1] + P[2][2]*H[1][2] + P[2][3]*H[1][3] + P[2][4]*H[1][4] + P[2][5]*H[1][5] + P[2][6]*H[1][6] + P[2][7]*H[1][7] + P[2][8]*H[1][8];
            aux[2][2] = P[2][0]*H[2][0] + P[2][1]*H[2][1] + P[2][2]*H[2][2] + P[2][3]*H[2][3] + P[2][4]*H[2][4] + P[2][5]*H[2][5] + P[2][6]*H[2][6] + P[2][7]*H[2][7] + P[2][8]*H[2][8];
            aux[3][0] = P[3][0]*H[0][0] + P[3][1]*H[0][1] + P[3][2]*H[0][2] + P[3][3]*H[0][3] + P[3][4]*H[0][4] + P[3][5]*H[0][5] + P[3][6]*H[0][6] + P[3][7]*H[0][7] + P[3][8]*H[0][8];
            aux[3][1] = P[3][0]*H[1][0] + P[3][1]*H[1][1] + P[3][2]*H[1][2] + P[3][3]*H[1][3] + P[3][4]*H[1][4] + P[3][5]*H[1][5] + P[3][6]*H[1][6] + P[3][7]*H[1][7] + P[3][8]*H[1][8];
            aux[3][2] = P[3][0]*H[2][0] + P[3][1]*H[2][1] + P[3][2]*H[2][2] + P[3][3]*H[2][3] + P[3][4]*H[2][4] + P[3][5]*H[2][5] + P[3][6]*H[2][6] + P[3][7]*H[2][7] + P[3][8]*H[2][8];
            aux[4][0] = P[4][0]*H[0][0] + P[4][1]*H[0][1] + P[4][2]*H[0][2] + P[4][3]*H[0][3] + P[4][4]*H[0][4] + P[4][5]*H[0][5] + P[4][6]*H[0][6] + P[4][7]*H[0][7] + P[4][8]*H[0][8];
            aux[4][1] = P[4][0]*H[1][0] + P[4][1]*H[1][1] + P[4][2]*H[1][2] + P[4][3]*H[1][3] + P[4][4]*H[1][4] + P[4][5]*H[1][5] + P[4][6]*H[1][6] + P[4][7]*H[1][7] + P[4][8]*H[1][8];
            aux[4][2] = P[4][0]*H[2][0] + P[4][1]*H[2][1] + P[4][2]*H[2][2] + P[4][3]*H[2][3] + P[4][4]*H[2][4] + P[4][5]*H[2][5] + P[4][6]*H[2][6] + P[4][7]*H[2][7] + P[4][8]*H[2][8];
            aux[5][0] = P[5][0]*H[0][0] + P[5][1]*H[0][1] + P[5][2]*H[0][2] + P[5][3]*H[0][3] + P[5][4]*H[0][4] + P[5][5]*H[0][5] + P[5][6]*H[0][6] + P[5][7]*H[0][7] + P[5][8]*H[0][8];
            aux[5][1] = P[5][0]*H[1][0] + P[5][1]*H[1][1] + P[5][2]*H[1][2] + P[5][3]*H[1][3] + P[5][4]*H[1][4] + P[5][5]*H[1][5] + P[5][6]*H[1][6] + P[5][7]*H[1][7] + P[5][8]*H[1][8];
            aux[5][2] = P[5][0]*H[2][0] + P[5][1]*H[2][1] + P[5][2]*H[2][2] + P[5][3]*H[2][3] + P[5][4]*H[2][4] + P[5][5]*H[2][5] + P[5][6]*H[2][6] + P[5][7]*H[2][7] + P[5][8]*H[2][8];
            aux[6][0] = P[6][0]*H[0][0] + P[6][1]*H[0][1] + P[6][2]*H[0][2] + P[6][3]*H[0][3] + P[6][4]*H[0][4] + P[6][5]*H[0][5] + P[6][6]*H[0][6] + P[6][7]*H[0][7] + P[6][8]*H[0][8];
            aux[6][1] = P[6][0]*H[1][0] + P[6][1]*H[1][1] + P[6][2]*H[1][2] + P[6][3]*H[1][3] + P[6][4]*H[1][4] + P[6][5]*H[1][5] + P[6][6]*H[1][6] + P[6][7]*H[1][7] + P[6][8]*H[1][8];
            aux[6][2] = P[6][0]*H[2][0] + P[6][1]*H[2][1] + P[6][2]*H[2][2] + P[6][3]*H[2][3] + P[6][4]*H[2][4] + P[6][5]*H[2][5] + P[6][6]*H[2][6] + P[6][7]*H[2][7] + P[6][8]*H[2][8];
            aux[7][0] = P[7][0]*H[0][0] + P[7][1]*H[0][1] + P[7][2]*H[0][2] + P[7][3]*H[0][3] + P[7][4]*H[0][4] + P[7][5]*H[0][5] + P[7][6]*H[0][6] + P[7][7]*H[0][7] + P[7][8]*H[0][8];
            aux[7][1] = P[7][0]*H[1][0] + P[7][1]*H[1][1] + P[7][2]*H[1][2] + P[7][3]*H[1][3] + P[7][4]*H[1][4] + P[7][5]*H[1][5] + P[7][6]*H[1][6] + P[7][7]*H[1][7] + P[7][8]*H[1][8];
            aux[7][2] = P[7][0]*H[2][0] + P[7][1]*H[2][1] + P[7][2]*H[2][2] + P[7][3]*H[2][3] + P[7][4]*H[2][4] + P[7][5]*H[2][5] + P[7][6]*H[2][6] + P[7][7]*H[2][7] + P[7][8]*H[2][8];
            aux[8][0] = P[8][0]*H[0][0] + P[8][1]*H[0][1] + P[8][2]*H[0][2] + P[8][3]*H[0][3] + P[8][4]*H[0][4] + P[8][5]*H[0][5] + P[8][6]*H[0][6] + P[8][7]*H[0][7] + P[8][8]*H[0][8];
            aux[8][1] = P[8][0]*H[1][0] + P[8][1]*H[1][1] + P[8][2]*H[1][2] + P[8][3]*H[1][3] + P[8][4]*H[1][4] + P[8][5]*H[1][5] + P[8][6]*H[1][6] + P[8][7]*H[1][7] + P[8][8]*H[1][8];
            aux[8][2] = P[8][0]*H[2][0] + P[8][1]*H[2][1] + P[8][2]*H[2][2] + P[8][3]*H[2][3] + P[8][4]*H[2][4] + P[8][5]*H[2][5] + P[8][6]*H[2][6] + P[8][7]*H[2][7] + P[8][8]*H[2][8];

            // K = P*H^t * [H*P*H^t + R]^-1
            K[0][0] = aux[0][0]*aux1[0][0] + aux[0][1]*aux1[1][0] + aux[0][2]*aux[2][0];
            K[0][1] = aux[0][0]*aux1[0][1] + aux[0][1]*aux1[1][1] + aux[0][2]*aux[2][1];
            K[0][2] = aux[0][0]*aux1[0][2] + aux[0][1]*aux1[1][2] + aux[0][2]*aux[2][2];
            K[1][0] = aux[1][0]*aux1[0][0] + aux[1][1]*aux1[1][0] + aux[1][2]*aux[2][0];
            K[1][1] = aux[1][0]*aux1[0][1] + aux[1][1]*aux1[1][1] + aux[1][2]*aux[2][1];
            K[1][2] = aux[1][0]*aux1[0][2] + aux[1][1]*aux1[1][2] + aux[1][2]*aux[2][2];
            K[2][0] = aux[2][0]*aux1[0][0] + aux[2][1]*aux1[1][0] + aux[2][2]*aux[2][0];
            K[2][1] = aux[2][0]*aux1[0][1] + aux[2][1]*aux1[1][1] + aux[2][2]*aux[2][1];
            K[2][2] = aux[2][0]*aux1[0][2] + aux[2][1]*aux1[1][2] + aux[2][2]*aux[2][2];
            K[3][0] = aux[3][0]*aux1[0][0] + aux[3][1]*aux1[1][0] + aux[3][2]*aux[2][0];
            K[3][1] = aux[3][0]*aux1[0][1] + aux[3][1]*aux1[1][1] + aux[3][2]*aux[2][1];
            K[3][2] = aux[3][0]*aux1[0][2] + aux[3][1]*aux1[1][2] + aux[3][2]*aux[2][2];
            K[4][0] = aux[4][0]*aux1[0][0] + aux[4][1]*aux1[1][0] + aux[4][2]*aux[2][0];
            K[4][1] = aux[4][0]*aux1[0][1] + aux[4][1]*aux1[1][1] + aux[4][2]*aux[2][1];
            K[4][2] = aux[4][0]*aux1[0][2] + aux[4][1]*aux1[1][2] + aux[4][2]*aux[2][2];
            K[5][0] = aux[5][0]*aux1[0][0] + aux[5][1]*aux1[1][0] + aux[5][2]*aux[2][0];
            K[5][1] = aux[5][0]*aux1[0][1] + aux[5][1]*aux1[1][1] + aux[5][2]*aux[2][1];
            K[5][2] = aux[5][0]*aux1[0][2] + aux[5][1]*aux1[1][2] + aux[5][2]*aux[2][2];
            K[6][0] = aux[6][0]*aux1[0][0] + aux[6][1]*aux1[1][0] + aux[6][2]*aux[2][0];
            K[6][1] = aux[6][0]*aux1[0][1] + aux[6][1]*aux1[1][1] + aux[6][2]*aux[2][1];
            K[6][2] = aux[6][0]*aux1[0][2] + aux[6][1]*aux1[1][2] + aux[6][2]*aux[2][2];
            K[7][0] = aux[7][0]*aux1[0][0] + aux[7][1]*aux1[1][0] + aux[7][2]*aux[2][0];
            K[7][1] = aux[7][0]*aux1[0][1] + aux[7][1]*aux1[1][1] + aux[7][2]*aux[2][1];
            K[7][2] = aux[7][0]*aux1[0][2] + aux[7][1]*aux1[1][2] + aux[7][2]*aux[2][2];
            K[8][0] = aux[8][0]*aux1[0][0] + aux[7][1]*aux1[1][0] + aux[8][2]*aux[2][0];
            K[8][1] = aux[8][0]*aux1[0][1] + aux[7][1]*aux1[1][1] + aux[8][2]*aux[2][1];
            K[8][2] = aux[8][0]*aux1[0][2] + aux[7][1]*aux1[1][2] + aux[8][2]*aux[2][2];
            */
        aux = H.mmul(P).mmul(H.t()).add(R);
        K = P.mmul(H.t()).mmul(Util.invertibleMatrix(aux));

        /* 4 - Pega os valores observados
        *  Y = H*X
        */
            /*
            observacao[0][0] = H[0][0]*nedE + H[0][1]*nedN + H[0][2]*nedD + H[0][3]*vxIMU + H[0][4]*vyIMU + H[0][5]*vzIMU + H[0][6]*ax_inercial + H[0][7]*ay_inercial + H[0][8]*az_inercial;
            observacao[1][0] = H[1][0]*nedE + H[1][1]*nedN + H[1][2]*nedD + H[1][3]*vxIMU + H[1][4]*vyIMU + H[1][5]*vzIMU + H[1][6]*ax_inercial + H[1][7]*ay_inercial + H[1][8]*az_inercial;
            observacao[2][0] = H[2][0]*nedE + H[2][1]*nedN + H[2][2]*nedD + H[2][3]*vxIMU + H[2][4]*vyIMU + H[2][5]*vzIMU + H[2][6]*ax_inercial + H[2][7]*ay_inercial + H[2][8]*az_inercial;
            */
        y = Z.sub(H.mmul(x_est));

        /* 5 - Calcula o novo vetor de estados
        *  x = x_pre + K * Y
        */

            /*
            new_state[0][0] = state_pred[0][0] + ( K[0][0]*observacao[0][0] + K[0][1]*observacao[1][0] + K[0][2]*observacao[2][0] );
            new_state[1][0] = state_pred[1][0] + ( K[1][0]*observacao[0][0] + K[1][1]*observacao[1][0] + K[1][2]*observacao[2][0] );
            new_state[2][0] = state_pred[2][0] + ( K[2][0]*observacao[0][0] + K[2][1]*observacao[1][0] + K[2][2]*observacao[2][0] );
            new_state[3][0] = state_pred[3][0] + ( K[3][0]*observacao[0][0] + K[3][1]*observacao[1][0] + K[3][2]*observacao[2][0] );
            new_state[4][0] = state_pred[4][0] + ( K[4][0]*observacao[0][0] + K[4][1]*observacao[1][0] + K[4][2]*observacao[2][0] );
            new_state[5][0] = state_pred[5][0] + ( K[5][0]*observacao[0][0] + K[5][1]*observacao[1][0] + K[5][2]*observacao[2][0] );
            new_state[6][0] = state_pred[6][0] + ( K[6][0]*observacao[0][0] + K[6][1]*observacao[1][0] + K[6][2]*observacao[2][0] );
            new_state[7][0] = state_pred[7][0] + ( K[7][0]*observacao[0][0] + K[7][1]*observacao[1][0] + K[7][2]*observacao[2][0] );
            new_state[8][0] = state_pred[8][0] + ( K[8][0]*observacao[0][0] + K[8][1]*observacao[1][0] + K[8][2]*observacao[2][0] );
            */
        // Actualiza el estado actual
        x_actual = x_est.add(K.mmul(y));
        x_ant = x_est.add(K.mmul(y));

        /* 6 - Atualiza matriz P
        *  P = (I - K*H)*P
        *///(I - K*H)

            /*
            aux[0][0] = 1 - K[0][0]*H[0][0] + K[0][1]*H[1][0] + K[0][2]*H[2][0];
            aux[0][1] = 0 - K[0][0]*H[0][1] + K[0][1]*H[1][1] + K[0][2]*H[2][1];
            aux[0][2] = 0 - K[0][0]*H[0][2] + K[0][1]*H[1][2] + K[0][2]*H[2][2];
            aux[0][3] = 0 - K[0][0]*H[0][3] + K[0][1]*H[1][3] + K[0][2]*H[2][3];
            aux[0][4] = 0 - K[0][0]*H[0][4] + K[0][1]*H[1][4] + K[0][2]*H[2][4];
            aux[0][5] = 0 - K[0][0]*H[0][5] + K[0][1]*H[1][5] + K[0][2]*H[2][5];
            aux[0][6] = 0 - K[0][0]*H[0][6] + K[0][1]*H[1][6] + K[0][2]*H[2][6];
            aux[0][7] = 0 - K[0][0]*H[0][7] + K[0][1]*H[1][7] + K[0][2]*H[2][7];
            aux[0][8] = 0 - K[0][0]*H[0][8] + K[0][1]*H[1][8] + K[0][2]*H[2][8];

            aux[1][0] = 0 - K[1][0]*H[0][0] + K[1][1]*H[1][0] + K[0][2]*H[2][0];
            aux[1][1] = 1 - K[1][0]*H[0][1] + K[1][1]*H[1][1] + K[0][2]*H[2][1];
            aux[1][2] = 0 - K[1][0]*H[0][2] + K[1][1]*H[1][2] + K[0][2]*H[2][2];
            aux[1][3] = 0 - K[1][0]*H[0][3] + K[1][1]*H[1][3] + K[0][2]*H[2][3];
            aux[1][4] = 0 - K[1][0]*H[0][4] + K[1][1]*H[1][4] + K[0][2]*H[2][4];
            aux[1][5] = 0 - K[1][0]*H[0][5] + K[1][1]*H[1][5] + K[0][2]*H[2][5];
            aux[1][6] = 0 - K[1][0]*H[0][6] + K[1][1]*H[1][6] + K[0][2]*H[2][6];
            aux[1][7] = 0 - K[1][0]*H[0][7] + K[1][1]*H[1][7] + K[0][2]*H[2][7];
            aux[1][8] = 0 - K[1][0]*H[0][8] + K[1][1]*H[1][8] + K[0][2]*H[2][8];

            aux[2][0] = 0 - K[2][0]*H[0][0] + K[2][1]*H[1][0] + K[0][2]*H[2][0];
            aux[2][1] = 0 - K[2][0]*H[0][1] + K[2][1]*H[1][1] + K[0][2]*H[2][1];
            aux[2][2] = 1 - K[2][0]*H[0][2] + K[2][1]*H[1][2] + K[0][2]*H[2][2];
            aux[2][3] = 0 - K[2][0]*H[0][3] + K[2][1]*H[1][3] + K[0][2]*H[2][3];
            aux[2][4] = 0 - K[2][0]*H[0][4] + K[2][1]*H[1][4] + K[0][2]*H[2][4];
            aux[2][5] = 0 - K[2][0]*H[0][5] + K[2][1]*H[1][5] + K[0][2]*H[2][5];
            aux[2][6] = 0 - K[2][0]*H[0][6] + K[2][1]*H[1][6] + K[0][2]*H[2][6];
            aux[2][7] = 0 - K[2][0]*H[0][7] + K[2][1]*H[1][7] + K[0][2]*H[2][7];
            aux[2][8] = 0 - K[2][0]*H[0][8] + K[2][1]*H[1][8] + K[0][2]*H[2][8];

            aux[3][0] = 0 - K[3][0]*H[0][0] + K[3][1]*H[1][0] + K[0][2]*H[2][0];
            aux[3][1] = 0 - K[3][0]*H[0][1] + K[3][1]*H[1][1] + K[0][2]*H[2][1];
            aux[3][2] = 0 - K[3][0]*H[0][2] + K[3][1]*H[1][2] + K[0][2]*H[2][2];
            aux[3][3] = 1 - K[3][0]*H[0][3] + K[3][1]*H[1][3] + K[0][2]*H[2][3];
            aux[3][4] = 0 - K[3][0]*H[0][4] + K[3][1]*H[1][4] + K[0][2]*H[2][4];
            aux[3][5] = 0 - K[3][0]*H[0][5] + K[3][1]*H[1][5] + K[0][2]*H[2][5];
            aux[3][6] = 0 - K[3][0]*H[0][6] + K[3][1]*H[1][6] + K[0][2]*H[2][6];
            aux[3][7] = 0 - K[3][0]*H[0][7] + K[3][1]*H[1][7] + K[0][2]*H[2][7];
            aux[3][8] = 0 - K[3][0]*H[0][8] + K[3][1]*H[1][8] + K[0][2]*H[2][8];

            aux[4][0] = 0 - K[4][0]*H[0][0] + K[4][1]*H[1][0] + K[0][2]*H[2][0];
            aux[4][1] = 0 - K[4][0]*H[0][1] + K[4][1]*H[1][1] + K[0][2]*H[2][1];
            aux[4][2] = 0 - K[4][0]*H[0][2] + K[4][1]*H[1][2] + K[0][2]*H[2][2];
            aux[4][3] = 0 - K[4][0]*H[0][3] + K[4][1]*H[1][3] + K[0][2]*H[2][3];
            aux[4][4] = 1 - K[4][0]*H[0][4] + K[4][1]*H[1][4] + K[0][2]*H[2][4];
            aux[4][5] = 0 - K[4][0]*H[0][5] + K[4][1]*H[1][5] + K[0][2]*H[2][5];
            aux[4][6] = 0 - K[4][0]*H[0][6] + K[4][1]*H[1][6] + K[0][2]*H[2][6];
            aux[4][7] = 0 - K[4][0]*H[0][7] + K[4][1]*H[1][7] + K[0][2]*H[2][7];
            aux[4][8] = 0 - K[4][0]*H[0][8] + K[4][1]*H[1][8] + K[0][2]*H[2][8];

            aux[5][0] = 0 - K[5][0]*H[0][0] + K[5][1]*H[1][0] + K[0][2]*H[2][0];
            aux[5][1] = 0 - K[5][0]*H[0][1] + K[5][1]*H[1][1] + K[0][2]*H[2][1];
            aux[5][2] = 0 - K[5][0]*H[0][2] + K[5][1]*H[1][2] + K[0][2]*H[2][2];
            aux[5][3] = 0 - K[5][0]*H[0][3] + K[5][1]*H[1][3] + K[0][2]*H[2][3];
            aux[5][4] = 0 - K[5][0]*H[0][4] + K[5][1]*H[1][4] + K[0][2]*H[2][4];
            aux[5][5] = 1 - K[5][0]*H[0][5] + K[5][1]*H[1][5] + K[0][2]*H[2][5];
            aux[5][6] = 0 - K[0][0]*H[0][6] + K[5][1]*H[1][6] + K[0][2]*H[2][6];
            aux[5][7] = 0 - K[5][0]*H[0][7] + K[5][1]*H[1][7] + K[0][2]*H[2][7];
            aux[5][8] = 0 - K[5][0]*H[0][8] + K[5][1]*H[1][8] + K[0][2]*H[2][8];

            aux[6][0] = 0 - K[6][0]*H[0][0] + K[6][1]*H[1][0] + K[0][2]*H[2][0];
            aux[6][1] = 0 - K[6][0]*H[0][1] + K[6][1]*H[1][1] + K[0][2]*H[2][1];
            aux[6][2] = 0 - K[6][0]*H[0][2] + K[6][1]*H[1][2] + K[0][2]*H[2][2];
            aux[6][3] = 0 - K[6][0]*H[0][3] + K[6][1]*H[1][3] + K[0][2]*H[2][3];
            aux[6][4] = 0 - K[6][0]*H[0][4] + K[6][1]*H[1][4] + K[0][2]*H[2][4];
            aux[6][5] = 0 - K[6][0]*H[0][5] + K[6][1]*H[1][5] + K[0][2]*H[2][5];
            aux[6][6] = 1 - K[6][0]*H[0][6] + K[6][1]*H[1][6] + K[0][2]*H[2][6];
            aux[6][7] = 0 - K[6][0]*H[0][7] + K[6][1]*H[1][7] + K[0][2]*H[2][7];
            aux[6][8] = 0 - K[6][0]*H[0][8] + K[6][1]*H[1][8] + K[0][2]*H[2][8];

            aux[7][0] = 0 - K[7][0]*H[0][0] + K[7][1]*H[1][0] + K[0][2]*H[2][0];
            aux[7][1] = 0 - K[7][0]*H[0][1] + K[7][1]*H[1][1] + K[0][2]*H[2][1];
            aux[7][2] = 0 - K[7][0]*H[0][2] + K[7][1]*H[1][2] + K[0][2]*H[2][2];
            aux[7][3] = 0 - K[7][0]*H[0][3] + K[7][1]*H[1][3] + K[0][2]*H[2][3];
            aux[7][4] = 0 - K[7][0]*H[0][4] + K[7][1]*H[1][4] + K[0][2]*H[2][4];
            aux[7][5] = 0 - K[7][0]*H[0][5] + K[7][1]*H[1][5] + K[0][2]*H[2][5];
            aux[7][6] = 0 - K[7][0]*H[0][6] + K[7][1]*H[1][6] + K[0][2]*H[2][6];
            aux[7][7] = 1 - K[7][0]*H[0][7] + K[7][1]*H[1][7] + K[0][2]*H[2][7];
            aux[7][8] = 0 - K[7][0]*H[0][8] + K[7][1]*H[1][8] + K[0][2]*H[2][8];

            aux[8][0] = 0 - K[8][0]*H[0][0] + K[8][1]*H[1][0] + K[0][2]*H[2][0];
            aux[8][1] = 0 - K[8][0]*H[0][1] + K[8][1]*H[1][1] + K[0][2]*H[2][1];
            aux[8][2] = 0 - K[8][0]*H[0][2] + K[8][1]*H[1][2] + K[0][2]*H[2][2];
            aux[8][3] = 0 - K[8][0]*H[0][3] + K[8][1]*H[1][3] + K[0][2]*H[2][3];
            aux[8][4] = 0 - K[8][0]*H[0][4] + K[8][1]*H[1][4] + K[0][2]*H[2][4];
            aux[8][5] = 0 - K[8][0]*H[0][5] + K[8][1]*H[1][5] + K[0][2]*H[2][5];
            aux[8][6] = 0 - K[8][0]*H[0][6] + K[8][1]*H[1][6] + K[0][2]*H[2][6];
            aux[8][7] = 0 - K[8][0]*H[0][7] + K[8][1]*H[1][7] + K[0][2]*H[2][7];
            aux[8][8] = 1 - K[8][0]*H[0][8] + K[8][1]*H[1][8] + K[0][2]*H[2][8];

            // (I-K*H)*P
            /*
            for (i=0; i<9; i++)
            {
                for (j=0; j<9; j++)
                {
                    aux1[i][j] = aux[i][0]*P[0][j] + aux[i][1]*P[1][j] + aux[i][2]*P[2][j] + aux[i][3]*P[3][j] + aux[i][4]*P[4][j] + aux[i][5]*P[5][j] + aux[i][6]*P[6][j] + aux[i][7]*P[7][j] + aux[i][8]*P[8][j];
                }
            }

            for (i=0; i<9; i++)
            {
                for (j=0; j<9; j++)
                {
                    P[i][j] = aux1[i][j];
                }
            }
            */
        P = I.sub(K.mmul(H)).mmul(P);


        /* Atualiza o vetor de estados anterior */
        /*
        for (i=0; i<9; i++)
        {
            state_ant[i][0] = new_state[i][0];
        }
        */
        /*
        estados.set(0, i, x_actual.get(0, 0));
        estados.set(1, i, x_actual.get(1, 0));
        estados.set(2, i, x_actual.get(2, 0));
        estados.set(3, i, x_actual.get(3, 0));
        estados.set(4, i, x_actual.get(4, 0));
        estados.set(5, i, x_actual.get(5, 0));
        */
    }


    public void setaMatrizesKalman(){
        /*
        A[0][0]=1; A[0][1]=0; A[0][2]=0; A[0][3]=dt; A[0][4]=0;  A[0][5]=0;  A[0][6]=dt*dt/2; A[0][7]=0;       A[0][8]=0;
        A[1][0]=0; A[1][1]=1; A[1][2]=0; A[1][3]=0;  A[1][4]=dt; A[1][5]=0;  A[1][6]=0;       A[1][7]=dt*dt/2; A[1][8]=0;
        A[2][0]=0; A[2][1]=0; A[2][2]=1; A[2][3]=0;  A[2][4]=0;  A[2][5]=dt; A[2][6]=0;       A[2][7]=0;       A[2][8]=dt*dt/2;
        A[3][0]=0; A[3][1]=0; A[3][2]=0; A[3][3]=1;  A[3][4]=0;  A[3][5]=0;  A[3][6]=dt;      A[3][7]=0;       A[3][8]=0;
        A[4][0]=0; A[4][1]=0; A[4][2]=0; A[4][3]=0;  A[4][4]=1;  A[4][5]=0;  A[4][6]=0;       A[4][7]=dt;      A[4][8]=0;
        A[5][0]=0; A[5][1]=0; A[5][2]=0; A[5][3]=0;  A[5][4]=0;  A[5][5]=1;  A[5][6]=0;       A[5][7]=0;       A[5][8]=dt;
        A[6][0]=0; A[6][1]=0; A[6][2]=0; A[6][3]=0;  A[6][4]=0;  A[6][5]=0;  A[6][6]=1;       A[6][7]=0;       A[6][8]=0;
        A[7][0]=0; A[7][1]=0; A[7][2]=0; A[7][3]=0;  A[7][4]=0;  A[7][5]=0;  A[7][6]=0;       A[7][7]=1;       A[7][8]=0;
        A[8][0]=0; A[8][1]=0; A[8][2]=0; A[8][3]=0;  A[8][4]=0;  A[8][5]=0;  A[8][6]=0;       A[8][7]=0;       A[8][8]=1;
        */
        // Matriz de transición A
        A = new DenseMatrix (new double [][]
                {
                        {1, 	0, 		dt, 	0, 		Math.pow(dt,2)/2d, 	0},
                        {0, 	1, 		0, 		dt, 	0, 					Math.pow(dt,2)/2d},
                        {0, 	0, 		1, 		0, 		dt, 				0},
                        {0, 	0, 		0, 		1, 		0, 					dt},
                        {0, 	0, 		0, 		0, 		1, 					0},
                        {0, 	0, 		0, 		0, 		0, 					1}
                });

        /*
        P[0][0]=1; P[0][1]=0; P[0][2]=0; P[0][3]=0; P[0][4]=0; P[0][5]=0; P[0][6]=0; P[0][7]=0; P[0][8]=0;
        P[1][0]=0; P[1][1]=1; P[1][2]=0; P[1][3]=0; P[1][4]=0; P[1][5]=0; P[1][6]=0; P[1][7]=0; P[1][8]=0;
        P[2][0]=0; P[2][1]=0; P[2][2]=1; P[2][3]=0; P[2][4]=0; P[2][5]=0; P[2][6]=0; P[2][7]=0; P[2][8]=0;
        P[3][0]=0; P[3][1]=0; P[3][2]=0; P[3][3]=1; P[3][4]=0; P[3][5]=0; P[3][6]=0; P[3][7]=0; P[3][8]=0;
        P[4][0]=0; P[4][1]=0; P[4][2]=0; P[4][3]=0; P[4][4]=1; P[4][5]=0; P[4][6]=0; P[4][7]=0; P[4][8]=0;
        P[5][0]=0; P[5][1]=0; P[5][2]=0; P[5][3]=0; P[5][4]=0; P[5][5]=1; P[5][6]=0; P[5][7]=0; P[5][8]=0;
        P[6][0]=0; P[6][1]=0; P[6][2]=0; P[6][3]=0; P[6][4]=0; P[6][5]=0; P[6][6]=1; P[6][7]=0; P[6][8]=0;
        P[7][0]=0; P[7][1]=0; P[7][2]=0; P[7][3]=0; P[7][4]=0; P[7][5]=0; P[7][6]=0; P[7][7]=1; P[7][8]=0;
        P[8][0]=0; P[8][1]=0; P[8][2]=0; P[8][3]=0; P[8][4]=0; P[8][5]=0; P[8][6]=0; P[8][7]=0; P[8][8]=1;
        */
        P = eye(6);

        /*
        Q[0][0]=0.05; Q[0][1]=0.00; Q[0][2]=0.00; Q[0][3]=0.00; Q[0][4]=0.00; Q[0][5]=0.00; Q[0][6]=0.00; Q[0][7]=0.00; Q[0][8]=0.00;
        Q[1][0]=0.00; Q[1][1]=0.05; Q[1][2]=0.00; Q[1][3]=0.00; Q[1][4]=0.00; Q[1][5]=0.00; Q[1][6]=0.00; Q[1][7]=0.00; Q[1][8]=0.00;
        Q[2][0]=0.00; Q[2][1]=0.00; Q[2][2]=0.05; Q[2][3]=0.00; Q[2][4]=0.00; Q[2][5]=0.00; Q[2][6]=0.00; Q[2][7]=0.00; Q[2][8]=0.00;
        Q[3][0]=0.00; Q[3][1]=0.00; Q[3][2]=0.00; Q[3][3]=0.05; Q[3][4]=0.00; Q[3][5]=0.00; Q[3][6]=0.00; Q[3][7]=0.00; Q[3][8]=0.00;
        Q[4][0]=0.00; Q[4][1]=0.00; Q[4][2]=0.00; Q[4][3]=0.00; Q[4][4]=0.05; Q[4][5]=0.00; Q[4][6]=0.00; Q[4][7]=0.00; Q[4][8]=0.00;
        Q[5][0]=0.00; Q[5][1]=0.00; Q[5][2]=0.00; Q[5][3]=0.00; Q[5][4]=0.00; Q[5][5]=0.05; Q[5][6]=0.00; Q[5][7]=0.00; Q[5][8]=0.00;
        Q[6][0]=0.00; Q[6][1]=0.00; Q[6][2]=0.00; Q[6][3]=0.00; Q[6][4]=0.00; Q[6][5]=0.00; Q[6][6]=0.05; Q[6][7]=0.00; Q[6][8]=0.00;
        Q[7][0]=0.00; Q[7][1]=0.00; Q[7][2]=0.00; Q[7][3]=0.00; Q[7][4]=0.00; Q[7][5]=0.00; Q[7][6]=0.00; Q[7][7]=0.05; Q[7][8]=0.00;
        Q[8][0]=0.00; Q[8][1]=0.00; Q[8][2]=0.00; Q[8][3]=0.00; Q[8][4]=0.00; Q[8][5]=0.00; Q[8][6]=0.00; Q[8][7]=0.00; Q[8][8]=0.05;
        */
        Q = eye(6).mul(cov_const);

        I = eye(6);

        H = zeros(4,6);

        R = zeros(4,4);

        Z = zeros(4,1);

        // Gera fator de conversão linear entre graus e metros a partir da posicao atual
        Location localGPS = managerGPS.getLastKnownLocation(LocationManager.GPS_PROVIDER);

        Coordinate coordinate = new Coordinate(localGPS.getLatitude(), localGPS.getLongitude());
        m2g = Util.convertMeterToGraus(coordinate);

        // First information
        x_ant = new DenseMatrix (new double [][]
                {{localGPS.getLongitude()},
                        {localGPS.getLatitude()},
                        {0},
                        {0},
                        {0},
                        {0}});

        gyro = new Gyroscope();
        acc = new Accelerometer();
        mag = new Magnetometer();
        grav = new Gravity();
    }
}
