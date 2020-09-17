package scorpsystems.nathan.com.smartmapping;

import android.content.Context;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by Nathan on 21/07/2017.
 */

public class ModuloExterno {

    //Variaveis
    Context context;

    //Ao criar um objeto do tipo ModuloExterno, o parametro Context e passado
    public ModuloExterno(Context c) {
        super();
        context = c;
    }

    //Verifica se pode escrever no armazenamento externo
    public boolean verificaArmazenamentoExternoEscrita(){
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)){
            return true;
        }
        return false;
    }

    //Verifica se ao menos a leitura no armazenamento externo esta acessivel
    public boolean verificaArmazenamentoExternoLeituraApenas(){
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) || Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)){
            return true;
        }
        return false;

    }

    //Tenta receber um path que ficar dentro da pasta do APP
    // E passado por paramentro o contexto do APP e qual tipo de arquivo que sera utilizado
    public File pegaDiretorioArquivo (String tipoDiretorio){
        //Pega o path da pasta do APP
        File file = new File(context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS), tipoDiretorio);

        if (!file.mkdirs()){
            Log.e("dir_prob","Diretorio nao foi criado com sucesso!");
            Toast.makeText(context,"Nao criou patch!",Toast.LENGTH_SHORT).show();
        }
        return file;

    }

    public void escreveArquivo(String patch, String arquivo, String dado){
        try {
            File file = new File(patch,arquivo);

            FileWriter fwriter = new FileWriter(file);
            fwriter.append(dado);
            fwriter.flush();
            fwriter.close();

            Toast.makeText(context,"TEXTO SALVO!",Toast.LENGTH_SHORT).show();
        } catch (IOException e) {
            Toast.makeText(context,"DEU PAU NA ESCRITA!",Toast.LENGTH_SHORT);
            e.printStackTrace();
        }

    }



}
