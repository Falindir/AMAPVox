/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar.voxelisation.raytracing.voxel;

import java.util.EventListener;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */
public interface VoxelAnalysisListener extends EventListener{
    
    void voxelisationFinished();
}