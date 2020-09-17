package scorpsystems.nathan.com.smartmapping;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.widget.Toast;

import static android.content.Context.LOCATION_SERVICE;

/**
 * Created by Nathan on 20/07/2017.
 */

//Ao criar um objeto do tipo ModuloGPS um objeto do tipo LocationListener e implementado e retornado
public class ModuloGPS implements LocationListener {

    //Variaveis
    Context context;

    //Ao criar um objeto do tipo ModuloGPS, o parametro Context e passado
    public ModuloGPS(Context c) {
        super();
        context = c;
    }

    //Pega a localizacao do GPS
    public Location pegaLocalicazaoGPS(){
        //Verifica se o APP tem permissao para acessar a localizacao do dispositivo
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED){
            //Toast.makeText(context, "Permissão para acessar o GPS não concedida!", Toast.LENGTH_SHORT).show();
            Log.e("first", "error");
            return null;
        }
        //Tenta receber a localizao do dispositivo
        try{
            //Define um objeto LocationManager que recebe o serviço de localizacao do dispositivo
            LocationManager lm = (LocationManager) context.getSystemService(LOCATION_SERVICE);
            //Verifica se o provedor de dados de localizacao esta ativo
            boolean isGPSAtivo = lm.isProviderEnabled(LocationManager.GPS_PROVIDER);
            if(isGPSAtivo){
                //Caso afirmativo, solicita uma atualizacao sobre a localizacao
                lm.requestLocationUpdates(LocationManager.GPS_PROVIDER,0,0,this);
                //Retorna a ultima posicao conhecida do dispositivo
                Location local = lm.getLastKnownLocation(LocationManager.GPS_PROVIDER);
                return local;
            }else{
                //Avisa que o provedor nao esta ativo
               // Toast.makeText(context,"GPS Inativo, por favor ative o GPS!",Toast.LENGTH_SHORT).show();
                Log.e("second","error");
            }
        }catch (Exception e){
            e.printStackTrace();
        }

        return null;

    }

    //Pega a localizacao da Rede
    public Location pegaLocalicazaoRede(){
        //Verifica se o APP tem permissao para acessar a localizacao do dispositivo
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED){
            //Toast.makeText(context, "Permissão para acessar a Rede/GPS não concedida!", Toast.LENGTH_SHORT).show();
            //Log.e("first", "error");
            return null;
        }
        //Tenta receber a localizao do dispositivo
        try{
            //Define um objeto LocationManager que recebe o serviço de localizacao do dispositivo
            LocationManager lm = (LocationManager) context.getSystemService(LOCATION_SERVICE);
            //Verifica se o provedor de dados de localizacao esta ativo
            boolean isGPSAtivo = lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
            if(isGPSAtivo){
                //Caso afirmativo, solicita uma atualizacao sobre a localizacao
                lm.requestLocationUpdates(LocationManager.NETWORK_PROVIDER,0,0,this);
                //Retorna a ultima posicao conhecida do dispositivo
                Location local = lm.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
                return local;
            }else{
                //Avisa que o provedor nao esta ativo
                //Toast.makeText(context,"Rede/GPS Inativo!",Toast.LENGTH_SHORT).show();
                //Log.e("second","error");
            }
        }catch (Exception e){
            e.printStackTrace();
        }
        return null;

    }

    //Pega a localizacao Passiva (mista)
    public Location pegaLocalicazaoMista(){
        //Verifica se o APP tem permissao para acessar a localizacao do dispositivo
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED){
            //Toast.makeText(context, "Permissão para acessar a Rede/GPS mista não concedida!", Toast.LENGTH_SHORT).show();
            //Log.e("first", "error");
            return null;
        }
        //Tenta receber a localizao do dispositivo
        try{
            //Define um objeto LocationManager que recebe o serviço de localizacao do dispositivo
            LocationManager lm = (LocationManager) context.getSystemService(LOCATION_SERVICE);
            //Verifica se o provedor de dados de localizacao esta ativo
            boolean isGPSAtivo = lm.isProviderEnabled(LocationManager.PASSIVE_PROVIDER);
            if(isGPSAtivo){
                //Caso afirmativo, solicita uma atualizacao sobre a localizacao
                lm.requestLocationUpdates(LocationManager.PASSIVE_PROVIDER,0,0,this);
                //Retorna a ultima posicao conhecida do dispositivo
                Location local = lm.getLastKnownLocation(LocationManager.PASSIVE_PROVIDER);
                return local;
            }else{
                //Avisa que o provedor nao esta ativo
                //Toast.makeText(context,"Rede/GPS mista Inativo!",Toast.LENGTH_SHORT).show();
                //Log.e("second","error");
            }
        }catch (Exception e){
            e.printStackTrace();
        }
        return null;

    }

    //Metodos obrigatorios de um objeto do tipo LocationListener
    @Override
    public void onLocationChanged(Location location) {

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
}
