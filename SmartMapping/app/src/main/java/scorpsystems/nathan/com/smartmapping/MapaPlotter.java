package scorpsystems.nathan.com.smartmapping;

import android.Manifest;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Build;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.FragmentActivity;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.gms.maps.model.TileOverlay;
import com.google.android.gms.maps.model.TileOverlayOptions;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import static android.R.attr.drawable;
import static android.R.attr.src;

public class MapaPlotter extends FragmentActivity implements OnMapReadyCallback {


    private GoogleMap meuMapa;
    Button bt1;
    Button bt2;
    Button bt3;
    Button bt4;
    Button bt5;
    Button bt6;
    boolean mapaReady = false;
    boolean canRead = false;
    int i;

    ModuloExterno ext;
    String sFolder = "DADOS";


    /* HEAT MAP */
    /**
     * Alternative radius for convolution
     */
    private static final int ALT_HEATMAP_RADIUS = 10;

    /**
     * Alternative opacity of heatmap overlay
     */
    private static final double ALT_HEATMAP_OPACITY = 0.4;

    /**
     * Alternative heatmap gradient (blue -> red)
     * Copied from Javascript version
     */
    private static final int[] ALT_HEATMAP_GRADIENT_COLORS = {
            Color.argb(0, 0, 255, 255),// transparent
            Color.argb(255 / 3 * 2, 0, 255, 255),
            Color.rgb(0, 191, 255),
            Color.rgb(0, 0, 127),
            Color.rgb(255, 0, 0)
    };

    public static final float[] ALT_HEATMAP_GRADIENT_START_POINTS = {
            0.0f, 0.10f, 0.20f, 0.60f, 1.0f
    };

    public static final Gradient ALT_HEATMAP_GRADIENT = new Gradient(ALT_HEATMAP_GRADIENT_COLORS,
            ALT_HEATMAP_GRADIENT_START_POINTS);

    private HeatmapTileProvider mProvider;
    private TileOverlay mOverlay;

    private boolean mDefaultGradient = true;
    private boolean mDefaultRadius = true;
    private boolean mDefaultOpacity = true;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mapa_plotter);
        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        bt1 = (Button) findViewById(R.id.bt1);
        bt2 = (Button) findViewById(R.id.bt2);
        bt3 = (Button) findViewById(R.id.bt3);
        bt4 = (Button) findViewById(R.id.bt4);
        bt5 = (Button) findViewById(R.id.bt5);
        bt6 = (Button) findViewById(R.id.bt6);

        try {
            SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(scorpsystems.nathan.com.smartmapping.R.id.mapFrag);
            mapFragment.getMapAsync(this);
        } catch (SecurityException ex) {
            Log.e("PROBLEMA MAPA", String.valueOf(ex));
        }

        /* Adiciona os pontos conhecidos que foram passados durante o trajeto */
        bt1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ArrayList<LatLng> pontos = new ArrayList<LatLng>();
                MarkerOptions marcador = new MarkerOptions();

                if (mapaReady){
                    ext = new ModuloExterno(getApplicationContext());
                    canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                    if (canRead){
                        File patch = ext.pegaDiretorioArquivo(sFolder);

                        try{
                            FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosConhecidosLat.txt");
                            FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosConhecidosLon.txt");

                            BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                            BufferedReader bufReadLon = new BufferedReader(fReaderLon);

                            String linhaLat;
                            String linhaLon;

                            linhaLat = bufReadLat.readLine();
                            linhaLon = bufReadLon.readLine();



                            while (linhaLat != null && linhaLon != null){
                                pontos.add(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)));

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();
                            }

                            bufReadLat.close();
                            bufReadLon.close();

                        }catch (IOException e) {
                            e.printStackTrace();
                        }

                        i = 1;
                        float zoom = (float) 18.0;
                        for (LatLng ponto : pontos){
                            meuMapa.moveCamera(CameraUpdateFactory.newLatLngZoom(ponto,zoom));
                            adicionarMarcador(ponto,"Ponto Conhecido "+i, BitmapDescriptorFactory.HUE_RED);
                            i = i+1;
                        }

                    }
               }
            }
        });

        /* Adiciona as leituras do GPS */
        bt2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ArrayList<LatLng> pontos = new ArrayList<LatLng>();
                MarkerOptions marcador = new MarkerOptions();

                if (mapaReady){
                    ext = new ModuloExterno(getApplicationContext());
                    canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                    if (canRead){
                        File patch = ext.pegaDiretorioArquivo(sFolder);

                        try{
                            FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosGPSLat.txt");
                            FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosGPSLon.txt");

                            BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                            BufferedReader bufReadLon = new BufferedReader(fReaderLon);

                            String linhaLat;
                            String linhaLon;

                            linhaLat = bufReadLat.readLine();
                            linhaLon = bufReadLon.readLine();



                            while (linhaLat != null && linhaLon != null){
                                pontos.add(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)));

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();
                            }

                            bufReadLat.close();
                            bufReadLon.close();

                        }catch (IOException e) {
                            e.printStackTrace();
                        }

                        i = 1;
                        float zoom = (float) 18.0;
                        for (LatLng ponto : pontos){
                            meuMapa.moveCamera(CameraUpdateFactory.newLatLngZoom(ponto,zoom));
                            adicionarMarcadorGPS(ponto,"Ponto Conhecido "+i);
                            i = i+1;
                        }

                    }
                }
            }

        });

        /*Traça o trajeto da IMU*/
        bt3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                    ArrayList<LatLng> pontos = new ArrayList<LatLng>();
                    MarkerOptions marcador = new MarkerOptions();

                    if (mapaReady){
                        ext = new ModuloExterno(getApplicationContext());
                        canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                        if (canRead){
                            File patch = ext.pegaDiretorioArquivo(sFolder);

                            try{
                                FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosIMULat.txt");
                                FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosIMULon.txt");

                                BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                                BufferedReader bufReadLon = new BufferedReader(fReaderLon);

                                String linhaLat;
                                String linhaLon;

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();



                                while (linhaLat != null && linhaLon != null){
                                    pontos.add(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)));

                                    linhaLat = bufReadLat.readLine();
                                    linhaLon = bufReadLon.readLine();
                                }

                                bufReadLat.close();
                                bufReadLon.close();

                            }catch (IOException e) {
                                e.printStackTrace();
                            }

                            i = 1;
                            PolylineOptions linha = new PolylineOptions();
                            linha.addAll(pontos);
                            linha
                                    .width(5)
                                    .color(Color.BLUE);

                            meuMapa.addPolyline(linha);

                        }
                    }
                }

        });

        /*Traça trajeto Kalman*/
        bt4.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                ArrayList<LatLng> pontos = new ArrayList<LatLng>();
                MarkerOptions marcador = new MarkerOptions();

                if (mapaReady){
                    ext = new ModuloExterno(getApplicationContext());
                    canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                    if (canRead){
                        File patch = ext.pegaDiretorioArquivo(sFolder);

                        try{
                            FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosKalmanLat.txt");
                            FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosKalmanLon.txt");

                            BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                            BufferedReader bufReadLon = new BufferedReader(fReaderLon);

                            String linhaLat;
                            String linhaLon;

                            linhaLat = bufReadLat.readLine();
                            linhaLon = bufReadLon.readLine();



                            while (linhaLat != null && linhaLon != null){
                                pontos.add(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)));

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();
                            }

                            bufReadLat.close();
                            bufReadLon.close();

                        }catch (IOException e) {
                            e.printStackTrace();
                        }

                        i = 1;
                        PolylineOptions linha = new PolylineOptions();
                        linha.addAll(pontos);
                        linha
                                .width(5)
                                .color(Color.GREEN);

                        meuMapa.addPolyline(linha);

                    }
                }
            }
        });

        bt5.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ArrayList<WeightedLatLng> pontos = new ArrayList<WeightedLatLng>();
                MarkerOptions marcador = new MarkerOptions();
                String lastLat;
                String lastLon;
                if (mapaReady){
                    ext = new ModuloExterno(getApplicationContext());
                    canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                    if (canRead){
                        File patch = ext.pegaDiretorioArquivo(sFolder);

                        try{
                            FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosKalmanLat.txt");
                            FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosKalmanLon.txt");
                            FileReader fReaderVal = new FileReader(patch.getPath()+"/"+"LeiturasPontosCompac.txt");

                            BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                            BufferedReader bufReadLon = new BufferedReader(fReaderLon);
                            BufferedReader bufReadVal = new BufferedReader(fReaderVal);

                            String linhaLat;
                            String linhaLon;
                            String linhaVal;

                            linhaLat = bufReadLat.readLine();
                            linhaLon = bufReadLon.readLine();
                            linhaVal = bufReadVal.readLine();

                            lastLat = "0.0";
                            lastLon = "0.0";

                            while (linhaLat != null && linhaLon != null){

                                if (lastLat == linhaLat && lastLon == linhaLon){
                                    pontos.add(new WeightedLatLng(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)),0.0001));
                                } else {
                                    pontos.add(new WeightedLatLng(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)),Double.parseDouble(linhaVal)));
                                    lastLat = linhaLat;
                                    lastLon = linhaLon;
                                }

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();
                                linhaVal = bufReadVal.readLine();

                            }

                            bufReadLat.close();
                            bufReadLon.close();
                            bufReadVal.close();

                        }catch (IOException e) {
                            e.printStackTrace();
                        }
                        int raio = 1;
                        int[] cores = {                     //  VALOR MAXIMO = 4.5 * 100%
                                //Color.rgb(255,255,255),     //Valor inicial  0%     - Branco
                                Color.rgb(182,252,171),     //              20%     - Branco/Verde
                                //Color.rgb(58,248,31),       //              50%     - Verde
                                //Color.rgb(32,203,7),        //              80%     - Verde/Escuro
                                Color.rgb(19,121,4)         //             100%     - Verde Escuro
                        };

                        float[] inicioCores ={
                                //0.01f,
                                0.2f,
                                //0.5f,
                                0.8f,
                                //1.0f
                        };

                        Gradient gradienteCores = new Gradient(cores,inicioCores);
                        mProvider = new HeatmapTileProvider.Builder().weightedData(pontos).gradient(gradienteCores).radius(10).build();
                        mOverlay = meuMapa.addTileOverlay(new TileOverlayOptions().tileProvider(mProvider));



                    }
                }


            }
        });


        bt6.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ArrayList<WeightedLatLng> pontos = new ArrayList<WeightedLatLng>();
                MarkerOptions marcador = new MarkerOptions();
                String lastLat;
                String lastLon;
                if (mapaReady){
                    ext = new ModuloExterno(getApplicationContext());
                    canRead = ext.verificaArmazenamentoExternoLeituraApenas();

                    if (canRead){
                        File patch = ext.pegaDiretorioArquivo(sFolder);

                        try{
                            FileReader fReaderLat = new FileReader(patch.getPath()+"/"+"PontosKalmanLat.txt");
                            FileReader fReaderLon = new FileReader(patch.getPath()+"/"+"PontosKalmanLon.txt");
                            FileReader fReaderVal = new FileReader(patch.getPath()+"/"+"LeiturasPontosUmidade.txt");

                            BufferedReader bufReadLat = new BufferedReader(fReaderLat);
                            BufferedReader bufReadLon = new BufferedReader(fReaderLon);
                            BufferedReader bufReadVal = new BufferedReader(fReaderVal);

                            String linhaLat;
                            String linhaLon;
                            String linhaVal;

                            linhaLat = bufReadLat.readLine();
                            linhaLon = bufReadLon.readLine();
                            linhaVal = bufReadVal.readLine();

                            lastLat = "0.0";
                            lastLon = "0.0";

                            while (linhaLat != null && linhaLon != null){

                                if (lastLat == linhaLat && lastLon == linhaLon){
                                    pontos.add(new WeightedLatLng(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)),0.0001));
                                } else {
                                    pontos.add(new WeightedLatLng(new LatLng(Double.parseDouble(linhaLat),Double.parseDouble(linhaLon)),Double.parseDouble(linhaVal)));
                                    lastLat = linhaLat;
                                    lastLon = linhaLon;
                                }

                                linhaLat = bufReadLat.readLine();
                                linhaLon = bufReadLon.readLine();
                                linhaVal = bufReadVal.readLine();

                            }

                            bufReadLat.close();
                            bufReadLon.close();
                            bufReadVal.close();

                        }catch (IOException e) {
                            e.printStackTrace();
                        }
                        int raio = 1;
                        int[] cores = {                     //  VALOR MAXIMO = 4.5 * 100%
                                //Color.rgb(255,255,255),     //Valor inicial  0%     - Branco
                                Color.rgb(159,159,255),     //              20%     - Branco/Verde
                                //Color.rgb(58,248,31),       //              50%     - Verde
                                //Color.rgb(32,203,7),        //              80%     - Verde/Escuro
                                Color.rgb(0,0,170)         //             100%     - Verde Escuro
                        };

                        float[] inicioCores ={
                                //0.01f,
                                0.2f,
                                //0.5f,
                                0.8f,
                                //1.0f
                        };

                        Gradient gradienteCores = new Gradient(cores,inicioCores);
                        mProvider = new HeatmapTileProvider.Builder().weightedData(pontos).gradient(gradienteCores).radius(10).build();
                        mOverlay = meuMapa.addTileOverlay(new TileOverlayOptions().tileProvider(mProvider));



                    }
                }


            }
        });


    }

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
        mapaReady = true;
        meuMapa.setMapType(GoogleMap.MAP_TYPE_SATELLITE);
    }


    public void adicionarMarcador(LatLng latlng, String titulo, float cor){
        MarkerOptions opcoes = new MarkerOptions();
        opcoes
                .position(latlng)
                .title(titulo)
                .draggable(false);

        ;
        Marker marcador = meuMapa.addMarker(opcoes);
    }

    public void adicionarMarcadorGPS(LatLng latlng, String titulo){
        MarkerOptions opcoes = new MarkerOptions();
        BitmapDescriptor icone = BitmapDescriptorFactory.fromResource(R.drawable.satelite_icone_resize);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            opcoes
                    .position(latlng)
                    .title(titulo)
                    .draggable(false)
                    .icon(icone);
        }

        Marker marcador = meuMapa.addMarker(opcoes);

    }

}
