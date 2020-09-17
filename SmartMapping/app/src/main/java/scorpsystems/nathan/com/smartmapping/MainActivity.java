package scorpsystems.nathan.com.smartmapping;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Application;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.location.LocationManager;
import android.media.Image;
import android.provider.Settings;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.Toast;

import com.google.android.gms.maps.SupportMapFragment;


/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class MainActivity extends AppCompatActivity {

    private static final int MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 1;
    private static final int MY_PERMISSIONS_REQUEST_ACCESS_COARSE_LOCATION = 2;

    ImageButton ibt_Iniciar;
    ImageButton ibt_Calibrar;
    ImageButton ibt_Opcoes;
    ImageButton ibt_Sair;

    BluetoothAdapter meuBluetoothAdapter = null;

    LocationManager lm = null;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);
        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        ibt_Iniciar = (ImageButton) findViewById(R.id.imageButton_Iniciar);
        ibt_Calibrar = (ImageButton) findViewById(R.id.imageButton_Calibrar);
        ibt_Opcoes = (ImageButton) findViewById(R.id.imageButton_Opcoes);
        ibt_Sair = (ImageButton) findViewById(R.id.imageButton_Sair);

        /*Pega o adaptador bluetooth do aparelho*/
        meuBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        meuBluetoothAdapter.enable();


        ibt_Iniciar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startActivity(new Intent(MainActivity.this, IniciarActivity.class));
            }
        });


        ibt_Calibrar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startActivity(new Intent(MainActivity.this, CalibrarActivity.class));
            }
        });

        ibt_Opcoes.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startActivity(new Intent(MainActivity.this, MapaPlotter.class));
            }
        });

        ibt_Sair.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

            }
        });



    }


}
