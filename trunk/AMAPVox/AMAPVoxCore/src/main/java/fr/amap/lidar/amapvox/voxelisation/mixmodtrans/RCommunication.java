package fr.amap.lidar.amapvox.voxelisation.mixmodtrans;

import org.rosuda.JRI.Rengine;

import java.io.File;
import java.net.URISyntaxException;

/**
 * @author Jimmy Lopez
 */
public class RCommunication {

    private Rengine r;

    /**
     * Pour ouvrir la connection vers le langage R
     */
    public RCommunication() {

        try {
            // Ne fonctionne pas
            //String RLIB = RCommunication.class.getClassLoader().getResource("lib/libjri.so").toURI().getPath();
            //System.load(RLIB);
            //RLIB = RLIB.substring(0, RLIB.length() - 9);
            //System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + RLIB);
            //System.out.println("LIB.so : " + RLIB);
            r = new Rengine(new String[]{"--no-save"}, false, null);
            String Rcode = RCommunication.class.getClassLoader().getResource("codeR/AMAPglme.R").toURI().getPath();
            r.eval("source('" + Rcode + "')");
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }

    }

    /**
     * Pour appeler la fonction glmer de R
     * @param prop
     * @param trials
     * @param ijk
     * @return
     */
    public double[] glme(double[] prop, double[] trials, String[] ijk) {

        r.assign("prop", prop);
        r.assign("trials", trials);
        r.assign("ijk", ijk);

        double[] res = r.eval("Jcode()").asDoubleArray();

        return res;
    }

    /**
     * Pour fermet la connection avec R
     */
    public void endConnection() {
        r.end();
    }

}
