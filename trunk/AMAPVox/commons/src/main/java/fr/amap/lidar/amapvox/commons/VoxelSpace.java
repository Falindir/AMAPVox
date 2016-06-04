/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.amap.lidar.amapvox.commons;

import fr.amap.commons.math.point.Point3F;
import fr.amap.commons.math.point.Point3I;
import java.util.List;

/**
 *
 * @author calcul
 */
public class VoxelSpace{
    
    protected final VoxelSpaceInfos voxelSpaceInfos;
    public List voxels;

    public VoxelSpace(VoxelSpaceInfos voxelSpaceInfos) {
        this.voxelSpaceInfos = voxelSpaceInfos;
    }

    public VoxelSpaceInfos getVoxelSpaceInfos() {
        return voxelSpaceInfos;
    }
    
    protected int get1DFrom3D(int i, int j, int k){
        
        return (i*voxelSpaceInfos.getSplit().y*voxelSpaceInfos.getSplit().z) + (j*voxelSpaceInfos.getSplit().z) +  k;
    }
    
    public Point3I getIndicesFromPoint(float x, float y ,float z){
        
        // shift to scene Min
        Point3F pt = new Point3F (x, y, z);
        pt.x -= voxelSpaceInfos.getMinCorner().x;
        pt.y -= voxelSpaceInfos.getMinCorner().y;
        pt.z -= voxelSpaceInfos.getMinCorner().z;

        if ((pt.z < 0) || (pt.z >= voxelSpaceInfos.getSplit().z)){
            return null;
        }
        if ((pt.x < 0) || (pt.x >= voxelSpaceInfos.getSplit().x)){
            return null;
        }
        if ((pt.y < 0) || (pt.y >= voxelSpaceInfos.getSplit().y)){
            return null;
        }
        pt.x /= voxelSpaceInfos.getResolution();
        pt.y /= voxelSpaceInfos.getResolution();
        pt.z /= voxelSpaceInfos.getResolution();

        Point3I indices = new Point3I();
        
        indices.x = (int) Math.floor ((double) (pt.x % voxelSpaceInfos.getSplit().x)); if (indices.x<0) indices.x += voxelSpaceInfos.getSplit().x;
        indices.y = (int) Math.floor ((double) (pt.y % voxelSpaceInfos.getSplit().y)); if (indices.y<0) indices.y += voxelSpaceInfos.getSplit().y;
        indices.z = (int) Math.min (pt.z, voxelSpaceInfos.getSplit().z-1);
                
        return indices;
    }
    
    public Object getVoxel(int i, int j, int k){
        
        if(i > voxelSpaceInfos.getSplit().x -1 || j > voxelSpaceInfos.getSplit().y -1 || k > voxelSpaceInfos.getSplit().z -1){
            return null;
        }
        
        int index = get1DFrom3D(i, j, k);
        
        if(index>voxels.size()-1){
            return null;
        }
        
        return voxels.get(index);
    }
    
    public Voxel[][] getLayerX(int xIndex){
        
        if(xIndex < 0 || xIndex >= voxelSpaceInfos.getSplit().x){
            return null;
        }
        
        Voxel[][] result = new Voxel[voxelSpaceInfos.getSplit().y][voxelSpaceInfos.getSplit().z];
        
        for (int j = 0; j < voxelSpaceInfos.getSplit().y; j++) {
            for (int k = 0;  k< voxelSpaceInfos.getSplit().z; k++) {
                result[j][k] = (Voxel) getVoxel(xIndex, j, k);
            }
        }
        
        return result;
    }
    
    public Voxel[][] getLayerY(int yIndex){
        
        if(yIndex < 0 || yIndex >= voxelSpaceInfos.getSplit().y){
            return null;
        }
        
        Voxel[][] result = new Voxel[voxelSpaceInfos.getSplit().x][voxelSpaceInfos.getSplit().z];
        
        for (int i = 0; i < voxelSpaceInfos.getSplit().x; i++) {
            for (int k = 0;  k< voxelSpaceInfos.getSplit().z; k++) {
                result[i][k] = (Voxel) getVoxel(i, yIndex, k);
            }
        }
        
        return result;
    }
    
    public Voxel[][] getLayerZ(int zIndex){
        
        if(zIndex < 0 || zIndex >= voxelSpaceInfos.getSplit().z){
            return null;
        }
        
        Voxel[][] result = new Voxel[voxelSpaceInfos.getSplit().x][voxelSpaceInfos.getSplit().y];
        
        for (int i = 0; i < voxelSpaceInfos.getSplit().x; i++) {
            for (int j = 0; j < voxelSpaceInfos.getSplit().y; j++) {
                result[i][j] = (Voxel) getVoxel(i, j, zIndex);
            }
        }
        
        return result;
    }
    
}
