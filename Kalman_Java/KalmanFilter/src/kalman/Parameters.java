package kalman;

import java.util.HashMap;
import java.util.Map;

import com.beust.jcommander.DynamicParameter;
import com.beust.jcommander.Parameter;

public class Parameters {
	   @Parameter(names = "-path", description = "Data path")
	    public String path;
	 	
	   @Parameter(names = "-output", description = "Output path")
	    public String output;
	   
	   @Parameter(names = "-show", description = "Show values in the terminal")
	    public boolean show = false;
	 
	   @DynamicParameter(names = "-D", description = "Dynamic parameters go here")
	    public Map<String, String> dynamicParams = new HashMap<String, String>();
}
