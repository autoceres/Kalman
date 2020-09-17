package scorpsystems.nathan.com.smartmapping;

import android.app.ListActivity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import java.util.Set;

/**
 * Created by Nathan on 17/08/2017.
 */

public class ListaDispositivosBluetooth extends ListActivity {

    /* Bluetooth */
    private BluetoothAdapter adaptadorBluetooth = null;

    /* Variaveis */
    static String ENDERECO_MAC = null;


    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        /*Criar um array de string com o estilo lista simples */
        ArrayAdapter<String> ArrayBluetooth = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);

        /*Pega o adaptador disponivel no aparelho */
        adaptadorBluetooth = BluetoothAdapter.getDefaultAdapter();

        /*Pega todos dispositivos pareados */
        Set<BluetoothDevice> dispositivosDisponiveis = adaptadorBluetooth.getBondedDevices();

        /* Se houver mais de um dispositivo disponivel */
        if (dispositivosDisponiveis.size() >0 ){
            /* Organiza todos dispositivos disponiveis */
            for (BluetoothDevice dispositivos : dispositivosDisponiveis) {
                String nomeDispositivo = dispositivos.getName();
                String macDispositivo = dispositivos.getAddress();
                ArrayBluetooth.add(nomeDispositivo + "\n" + macDispositivo);
            }
        }
        setListAdapter(ArrayBluetooth);

    }

    @Override
    protected void onListItemClick(ListView l, View v, int position, long id) {
        super.onListItemClick(l, v, position, id);

        /* Pega o item selecionado na lista */
        String informacaoGeral = ((TextView) v).getText().toString();

        /* Separa so o endereco do dispositivo */
        String enderecoMac = informacaoGeral.substring(informacaoGeral.length() - 17);

        /* Retorna o valor para a tela anterior */
        Intent retornaMac = new Intent();
        retornaMac.putExtra(ENDERECO_MAC, enderecoMac);
        setResult(RESULT_OK,retornaMac);
        finish();
    }
}
