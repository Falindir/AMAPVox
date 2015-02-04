/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar.lidar.format.voxelspace;

import fr.ird.voxelidar.math.point.Point2F;
import fr.ird.voxelidar.math.vector.Vec3F;
import java.util.Map;

/**
 *
 * @author Julien
 */
public class Voxel {
    
    private Map<String,Float> attributs;
    public int type;
    public final float x, y, z;
    public final int indiceX, indiceY, indiceZ;
    public float attributValue;
    public Vec3F color;
    public float alpha;
    public Map<String, Point2F> minMax;

    public Map<String, Float> getAttributs() {
        return attributs;
    }
    
    public Voxel(int indiceX, int indiceY, int indiceZ, float x, float y, float z, float attributValue){
        
        this.indiceX = indiceX;
        this.indiceY = indiceY;
        this.indiceZ = indiceZ;
        
        this.x = x;
        this.y = y;
        this.z = z;
        
        this.alpha = 1.0f;
        
        this.attributValue = attributValue;
        
        this.type = 6;
    }
    
    public Voxel(int indiceX, int indiceY, int indiceZ, float x, float y, float z, Map<String,Float> attributs, float alpha){
        
        this.indiceX = indiceX;
        this.indiceY = indiceY;
        this.indiceZ = indiceZ;
        
        this.x = x;
        this.y = y;
        this.z = z;
        
        this.alpha = alpha;
        this.attributs = attributs;
        
        this.type = 6;
    }
    
}