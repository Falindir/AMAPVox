/**
 *
 */
package fr.amap.amapvox.simulation.hemi;

import fr.amap.amapvox.commons.util.MatrixAndFile;
import fr.amap.amapvox.commons.util.MatrixUtility;
import fr.amap.amapvox.io.tls.rsp.Rsp;
import fr.amap.amapvox.io.tls.rsp.Scans;
import fr.amap.amapvox.io.tls.rxp.RxpExtraction;
import fr.amap.amapvox.io.tls.rxp.Shot;
import fr.amap.amapvox.jeeb.workspace.sunrapp.geometry.CoordinatesConversion;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import javax.vecmath.Point3f;
import javax.vecmath.Point3i;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

import fr.amap.amapvox.jeeb.workspace.sunrapp.geometry.Transformations;
import fr.amap.amapvox.math.matrix.Mat3D;
import fr.amap.amapvox.math.matrix.Mat4D;
import fr.amap.amapvox.math.vector.Vec3D;
import fr.amap.amapvox.math.vector.Vec4D;
import fr.amap.amapvox.simulation.transmittance.TransmittanceParameters;
import fr.amap.amapvox.simulation.transmittance.TransmittanceSim;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import javax.imageio.ImageIO;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import org.apache.commons.math3.util.FastMath;
import org.apache.log4j.Logger;
import org.jdom2.JDOMException;

/**
 * @author dauzat
 *
 */
public class HemiScanView {

    private final static Logger logger = Logger.getLogger(HemiScanView.class);
    
    private final float skyLuminance = 1f;
    private final float canopyLuminance = 0.1f;
    private final Point3f rgbSky;
    private final Point3f rgbCan;

    private final int nbPixels;
    private int decimation; // the proportion of shots used for computation is 1/decimation

    private final int minSampling;	// minimum number of shots sampling a sector for calculating its gap fraction

    private final int nbAzimuts;
    private final int nbZeniths;
    private Pixel[][] pixTab;
    private Sector[][] sectorTable;
    private final boolean random;	// the color of pixels is drawn randomly depending on the gap fraction
    private Mat4D transformation;
    private Mat3D rotationFromTransf;

    private class Pixel {

        int nbShots;
        float brightness;

        public Pixel() {
            super();
            this.nbShots = 0;
            this.brightness = 0;
        }

        protected void updatePixel(float luminance) {
            if (nbShots == 0) {
                brightness = luminance;
                nbShots++;
            } else {
                float newBrightness = (brightness * nbShots) + (luminance);
                nbShots++;
                brightness = newBrightness / (float) nbShots;
            }
        }
    }

    class Sector {

        int nbShots;
        float brightness;

        public Sector() {
            super();
            this.nbShots = 0;
            this.brightness = 0;
        }

        protected void updateSector(float luminance) {
            if (nbShots == 0) {
                brightness = luminance;
                nbShots++;
            } else {
                float newBrightness = (brightness * nbShots) + (luminance);
                nbShots++;
                brightness = newBrightness / (float) nbShots;
            }
        }
    }
    
    public HemiScanView(){
        
        decimation = 100;
        nbPixels = 800;
        nbZeniths = 9; //6;
        nbAzimuts = 36; //24;
        random = true;
        minSampling = 50;
        rgbSky = new Point3f(0, 0, 255);
        rgbCan = new Point3f(0, 255, 0);
        
        transformation = Mat4D.identity();
        rotationFromTransf = getRotationFromMatrix(transformation);

        initArrays();
    }
    
    public HemiScanView(HemiParameters parameters){
        
        nbPixels = parameters.getPixelNumber();
        nbZeniths = parameters.getZenithsNumber(); //6;
        nbAzimuts = parameters.getAzimutsNumber(); //24;
        random = true;
        minSampling = 50;
        rgbSky = new Point3f(0, 0, 255);
        rgbCan = new Point3f(0, 255, 0);
        
        initArrays();
    }
    
    private void initArrays(){
        
        pixTab = new Pixel[nbPixels][];
        for (int x = 0; x < nbPixels; x++) {
            pixTab[x] = new Pixel[nbPixels];
            for (int y = 0; y < nbPixels; y++) {
                pixTab[x][y] = new Pixel();
            }
        }
        // table of azimuthAngle/zenith sectors
        sectorTable = new Sector[nbZeniths][];
        for (int z = 0; z < nbZeniths; z++) {
            sectorTable[z] = new Sector[nbAzimuts];
            for (int a = 0; a < nbAzimuts; a++) {
                sectorTable[z][a] = new Sector();
            }
        }
    }
    
    public static void launchSimulation(HemiParameters parameters){
        
        
        HemiScanView hemiScanView = new HemiScanView(parameters);
        
        switch(parameters.getMode()){
            case ECHOS:
                
                for(MatrixAndFile scan : parameters.getRxpScansList()){
                
                    hemiScanView.setTransformation(MatrixUtility.convertMatrix4dToMat4D(scan.matrix));
                    hemiScanView.setScan(scan.file);
                }
                
                break;
                
            case PAD:
                hemiScanView.hemiFromPAD(parameters.getVoxelFile(), parameters.getSensorPosition());
                break;
            default:
                return;
        }
        
        if(parameters.isGenerateBitmapFile()){
                
            switch(parameters.getBitmapMode()){
                case PIXEL:
                    hemiScanView.writeHemiPhoto(parameters.getOutputBitmapFile());
                    break;
                case COLOR:
                    hemiScanView.sectorTable(parameters.getOutputBitmapFile());
                    break;
            }
        }
        
    }
    
    public void setTransformation(Mat4D matrix){
        
        transformation = matrix;
        rotationFromTransf = getRotationFromMatrix(transformation);
    }
    
    public Mat3D getRotationFromMatrix(Mat4D matrix){
        
        Mat3D rotation = new Mat3D();
        
        rotation.mat = new double[]{
            transformation.mat[0],transformation.mat[1],transformation.mat[2],
            transformation.mat[4],transformation.mat[5],transformation.mat[6],
            transformation.mat[8],transformation.mat[9],transformation.mat[10]
        };
        
        return rotation;
    }
    
    public void setTransformation(File matrixFile){
        
        BufferedReader reader;        
        try {
            reader = new BufferedReader(new FileReader(matrixFile));
            transformation = readMatrix(reader);
            rotationFromTransf = getRotationFromMatrix(transformation);
        
        } catch (FileNotFoundException ex) {
            logger.error(ex);
        }
    }
    
    public void setScan(File scan){
        RxpExtraction extraction = new RxpExtraction();
        try {
            extraction.openRxpFile(scan, RxpExtraction.SIMPLE_SHOT);
            
            Iterator<Shot> iterator = extraction.iterator();
            
            int count = 0;
            int totalShots = 0;
            
            while(iterator.hasNext()){
                
                Shot shot = iterator.next();
                
                Vec4D locVector = Mat4D.multiply(transformation, new Vec4D(shot.origin.x, shot.origin.y, shot.origin.z, 1.0d));
                Vec3D uVector = Mat3D.multiply(rotationFromTransf, new Vec3D(shot.direction.x, shot.direction.y, shot.direction.z));

                
                shot.setOriginAndDirection(new Point3d(locVector.x, locVector.y, locVector.z), new Vector3d(uVector.x, uVector.y, uVector.z));
                
                //shot.direction.sub(shot.origin);
                //shot.direction.normalize();    
                
                transform(shot);
                
                count++;
                totalShots++;
                
                if(count == 1000000){
                    System.out.println("Shots processed : "+totalShots);
                    //logger.info("Shots processed : "+totalShots);
                    count = 0;
                }
            }
            
            //logger.info("Shots processed : "+count);
            logger.info("Total shots processed : "+totalShots);
            
            extraction.close();
        } catch (Exception ex) {
            logger.error(ex);
        }
    }

    /**
     * @param args
     * @throws IOException
     */
    public static void main(String[] args) throws IOException {

        HemiParameters parameters = new HemiParameters();
        parameters.setZenithsNumber(9);
        parameters.setAzimutsNumber(36);
        parameters.setPixelNumber(800);
        
        HemiScanView hemiScanView = new HemiScanView(parameters);
        hemiScanView.hemiFromPAD(new File("/home/calcul/Documents/Julien/Test_photos_hemispheriques/tmp/tls_paracou_2013_non_pondere.vox"), new Point3d(0, 80, 29));
        hemiScanView.writeHemiPhoto(new File("/home/calcul/Documents/Julien/Test_photos_hemispheriques/tmp/hemiFromPAD.png"));
    }

    private void sectorTable(File outputFile) {

        float radius = nbPixels / 2f;
        float zenithWidth = (float) ((Math.PI / 2) / nbZeniths);
        float azimWidth = (float) ((Math.PI * 2) / nbAzimuts);

        for (int z = 0; z < nbZeniths; z++) {
            System.out.println();
            for (int a = 0; a < nbAzimuts; a++) {
                System.out.print("\t" + sectorTable[z][a].brightness);
            }
        }

        for (int x = 0; x < nbPixels; x++) {
            for (int y = 0; y < nbPixels; y++) {
                Vector2f v = new Vector2f((x - radius) / radius, (y - radius) / radius);
                float zen = (float) (v.length() * Math.PI / 2);
                if (zen < Math.PI / 2) {
                    float azim = xyAzimuthNW(v.x, v.y);
                    int sx = (int) (zen / zenithWidth);
                    int sy = (int) (azim / azimWidth);

                    if (sectorTable[sx][sy].nbShots > minSampling) {
                        if (random) {
                            float gf = (sectorTable[sx][sy].brightness - canopyLuminance) / (skyLuminance - canopyLuminance);
                            if (Math.random() > gf) {
                                pixTab[x][y].brightness = canopyLuminance;
                            } else {
                                pixTab[x][y].brightness = skyLuminance;
                            }
                        } else {
                            pixTab[x][y].brightness = sectorTable[sx][sy].brightness;
                        }
                    }
                }
            }
        }

        writeHemiPhoto(outputFile);
    }
    
    private void hemiFromPAD(File inputFile, Point3d position){
        
        TransmittanceSim sim;
        try {
            
            sim = new TransmittanceSim();
            sim.readData(inputFile);
                
        } catch (IOException ex) {
            java.util.logging.Logger.getLogger(HemiScanView.class.getName()).log(Level.SEVERE, null, ex);
            return;
        }catch (Exception ex) {
            java.util.logging.Logger.getLogger(HemiScanView.class.getName()).log(Level.SEVERE, null, ex);
            return;
        }

        
        float center = nbPixels / 2;
        
        for (int i = 0; i < nbPixels; i++) {
            for (int j = 0; j < nbPixels; j++) {
                
                float deltaX = i + 0.5f - center;
                float deltaY = j + 0.5f - center;
                double distToCenter = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
                
                if (distToCenter < center) {
                    
                    float zenithAngle = (float) ((distToCenter/center)*Math.PI/2);
                    float azimuthAngle = 0;
                    
                    if (deltaY != 0) {
                        azimuthAngle = (float) Math.atan(deltaX / deltaY);
                        if (deltaY < 0) {
                            azimuthAngle += Math.PI;
                        } else if (deltaX < 0) {
                            azimuthAngle += Math.PI * 2;
                        }
                    } else if (deltaX < 0) {
                        azimuthAngle = (float) (Math.PI / 2);
                    }
                    
                    Vector3f direction = CoordinatesConversion.polarToCartesian(zenithAngle, azimuthAngle);
                    
                    /*Vector3f direction = new Vector3f(0,0,1);
                    Transformations transform = new Transformations();
                    transform.setRotationAroundX(zenithAngle);
                    transform.setRotationAroundZ(azimuthAngle);
                    transform.apply(direction);*/
                    
                    /*if(direction.x != rayDirection.x || direction.y != rayDirection.y || direction.z != rayDirection.z){
                        System.out.println("test");
                    }*/
                    Vector3d directionD = new Vector3d(direction.x, direction.y, direction.z);
                    
                    List<Double> distances = sim.distToVoxelWalls(position, directionD);
                    double transmittance = sim.directionalTransmittance(position, distances, directionD);
                    
                    pixTab[i][j].updatePixel((float)transmittance);
                }
            }
        }
    }

    private static Mat4D readMatrix(BufferedReader br) {

        Matrix4d mat = new Matrix4d();

        try {
            String line = br.readLine();
            int r = 0;
            while (line != null) {
                String[] els = line.split(" ");
                for (int c = 0; c < els.length; c++) {
                    mat.setElement(r, c, Double.valueOf(els[c]));
                }
                line = br.readLine();
                r++;
            }
            
            return MatrixUtility.convertMatrix4dToMat4D(mat);
            
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        return null;
    }

    private void transform(String line, Transformations tr) {
        String[] st = line.split(" ");
        int echocount = Integer.valueOf(st[1]);
        Point3f origin = new Point3f(Float.valueOf(st[2]), Float.valueOf(st[3]), Float.valueOf(st[4]));
        Vector3f direction = new Vector3f(Float.valueOf(st[5]), Float.valueOf(st[6]), Float.valueOf(st[7]));
        float range = -9999;
        if (echocount > 0) {
            range = Float.valueOf(st[8]);
        }
        tr.apply(direction);
        tr.apply(origin);
        direction.sub(origin);
        direction.normalize();
        float zenith = (float) FastMath.acos(direction.z);
        float azimuth = xyAzimuthNW(direction.x, direction.y);

        if (zenith < Math.PI / 2) {
            updatePixTab(zenith, azimuth, range);
            updateSectorTab(zenith, azimuth, range);
        }
    }

    private void transform(Shot shot) {

        float zenith = (float) FastMath.acos(shot.direction.z);
        float azimuth = xyAzimuthNW((float) shot.direction.x, (float) shot.direction.y);

        float range = -9999;
        if (shot.nbEchos > 0) {
            range = (float) shot.ranges[shot.nbEchos-1];
        }

        if (zenith < Math.PI / 2) {
            updatePixTab(zenith, azimuth, range);
            updateSectorTab(zenith, azimuth, range);
        }
    }

    public void writeHemiPhoto(File outputFile) {

        int border = 30;
        int nbPixImage = pixTab.length + (2 * border); //= 600;
        float center = nbPixImage / 2;
        float radius = center - border;

        BufferedImage bimg = new BufferedImage(nbPixImage, nbPixImage, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = bimg.createGraphics();

        // background
        g.setColor(new Color(80, 30, 0));
        g.fillRect(0, 0, nbPixImage, nbPixImage);

        // black sky vault
        g.setColor(new Color(0, 0, 0));
        g.fillOval((int) (center - radius), (int) (center - radius), (int) (2 * radius), (int) (2 * radius));

        // draw points
        for (int x = 0; x < pixTab.length; x++) {
            for (int y = 0; y < pixTab[x].length; y++) {
                int yn = pixTab.length - 1 - y; // North in Y+
                if (pixTab[x][y].brightness > 0) {
                    float gf = (pixTab[x][y].brightness - canopyLuminance) / (skyLuminance - canopyLuminance);
                    Point3f rgbr = new Point3f(rgbCan);
                    rgbr.scale(1 - gf);
                    Point3f rgbb = new Point3f(rgbSky);
                    rgbb.scale(gf);
                    Point3f rgb = new Point3f(rgbb);
                    rgb.add(rgbr);
                    rgb.x = Math.max(rgb.x, 0);
                    rgb.y = Math.max(rgb.y, 0);
                    rgb.z = Math.max(rgb.z, 0);
                    rgb.x = Math.min(rgb.x, 255);
                    rgb.y = Math.min(rgb.y, 255);
                    rgb.z = Math.min(rgb.z, 255);
                    Point3i c = new Point3i((int) rgb.x, (int) rgb.y, (int) rgb.z);
                    g.setColor(new Color(c.x, c.y, c.z));
                    g.drawRect(x + border, yn + border, 1, 1);
                }
            }
        }

        // parallels
        g.setColor(new Color(220, 240, 255));
        float rad = radius / nbZeniths;
        for (int i = 1; i <= nbZeniths; i++) {
            g.drawOval((int) (center - rad * i), (int) (center - rad * i), (int) (2 * rad * i), (int) (2 * rad * i));
        }
        // meridians
        for (int i = 1; i <= nbAzimuts; i++) {
            double azimuth = i * (Math.PI * 2 / nbAzimuts);
            double x = radius * Math.sin(azimuth);
            double y = radius * Math.cos(azimuth);
            g.drawLine((int) (center - x), (int) (center - y), (int) (center + x), (int) (center + y));
        }

        // cardinal points
        g.drawString("N", (int) center, border / 2);
        g.drawString("S", (int) center, nbPixImage - (border / 2));
        g.drawString("E", nbPixImage - (border / 2), center);
        g.drawString("W", border / 2, center);
        
        try {
            ImageIO.write(bimg, "png", outputFile);
        } catch (IOException ex) {
            logger.error(ex);
        }
    }

    /**
     * @param	x, y	2D coordinates
     * @return	azimuthAngle	[radian] clockwise from Y axis
     */
    public static float xyAzimuthNW(float x, float y) {

        float azimuth = 0;
        if(y != 0) {
            
            azimuth = (float) FastMath.atan(x/y);
            if(y < 0){
                azimuth += Math.PI;
            }
            else if(x < 0){
                azimuth += Math.PI*2;
            }
        }
        else if (x < 0){
            azimuth = (float) (Math.PI / 2);
        }
        
        return azimuth;
    }

    private void updatePixTab(float zenith, float azimut, float distance) {
        
        float radius = nbPixels / 2f;
        float rad = (float) ((zenith / (Math.PI / 2)) * radius);
        
        float x = (float) (rad * Math.cos(azimut));
        float y = (float) (rad * Math.sin(azimut));
        
        int indexX = (int) (x + radius);
        int indexY = (int) (y + radius);
        
        indexX = Math.min(indexX, nbPixels - 1);
        indexY = Math.min(indexY, nbPixels - 1);

        if (distance < 0) {
            pixTab[indexX][indexY].updatePixel(skyLuminance);
        } else {
            pixTab[indexX][indexY].updatePixel(canopyLuminance);
        }
    }

    private void updateSectorTab(float zenith, float azimuth, float distance) {

        int indexZn = (int) ((zenith * nbZeniths) / (Math.PI / 2));
        int indexAz = (int) ((azimuth * nbAzimuts) / (Math.PI * 2));

        if (distance < 0) {
            sectorTable[indexZn][indexAz].updateSector(skyLuminance);
        } else {
            sectorTable[indexZn][indexAz].updateSector(canopyLuminance);
        }
    }

}