/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar.voxelisation.extraction;

import java.util.EventListener;

/**
 *
 * @author Julien
 */
public interface LasPointListener extends EventListener{
    
    void pointExtracted(LasPoint point);
}