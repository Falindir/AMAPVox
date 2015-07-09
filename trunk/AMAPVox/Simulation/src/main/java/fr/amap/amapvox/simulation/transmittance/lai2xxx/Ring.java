/*
This software is distributed WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

This program is open-source LGPL 3 (see copying.txt).
Authors:
    Gregoire Vincent    gregoire.vincent@ird.fr
    Julien Heurtebize   julienhtbe@gmail.com
    Jean Dauzat         jean.dauzat@cirad.fr
    Rémi Cresson        cresson.r@gmail.com

For further information, please contact Gregoire Vincent.
 */

package fr.amap.amapvox.simulation.transmittance.lai2xxx;

/**
 *
 * @author calcul
 */


public class Ring {
    
    private final float lowerZenithalAngle;
    private final float upperZenithalAngle;
    private final float meanAngle;
    private final float width;
    private final float weightingFactor;
    private final float solidAngle;
    private int nbDirections;
    private float sumTrans;
    private float acfs;
    private float cntct;
    private float stdev;
    private float dist;
    private float gap;
    

    public Ring(float lowerZenithalAngle, float upperZenithalAngle) {
        
        this.lowerZenithalAngle = lowerZenithalAngle;
        this.upperZenithalAngle = upperZenithalAngle;
        
        this.meanAngle = (lowerZenithalAngle+upperZenithalAngle) / 2.0f;
        this.width = lowerZenithalAngle - upperZenithalAngle;
        this.weightingFactor = (float) (this.width * this.meanAngle);
        
        //calcul de l'angle solide
        solidAngle = (float) (2* Math.PI * (Math.cos(Math.toRadians(upperZenithalAngle)) - Math.cos(Math.toRadians(lowerZenithalAngle))));
    }   

    public float getLowerZenithalAngle() {
        return lowerZenithalAngle;
    }

    public float getUpperZenithalAngle() {
        return upperZenithalAngle;
    }    

    public float getSolidAngle() {
        return solidAngle;
    }

    public float getMeanAngle() {
        return meanAngle;
    }

    public float getAvgtrans() {
        
        return sumTrans/nbDirections;
    }

    public float getAcfs() {
        return acfs;
    }

    public float getCntct() {
        return cntct;
    }

    public float getStdev() {
        return stdev;
    }

    public float getDist() {
        return dist;
    }

    public float getGap() {
        return gap;
    }

    public void setTrans(float transmittance) {
        
        if(!Float.isNaN(transmittance)){
            this.sumTrans = transmittance;
        }
    }

    public void setAcfs(float acfs) {
        this.acfs = acfs;
    }

    public void setCntct(float cntct) {
        this.cntct = cntct;
    }

    public void setStdev(float stdev) {
        this.stdev = stdev;
    }

    public void setDist(float dist) {
        this.dist = dist;
    }

    public void setGap(float gap) {
        this.gap = gap;
    }  

    public void setNbDirections(int nbDirections) {
        this.nbDirections = nbDirections;
    }

    public float getWidth() {
        return width;
    }    

    public float getWeightingFactor() {
        return weightingFactor;
    }
}