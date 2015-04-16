/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar.engine3d.object.lighting;

import java.nio.FloatBuffer;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */
public interface TexturedMesh extends Mesh{
    
    public FloatBuffer textureCoordinatesBuffer = null;
    
    public void attachTexture(Texture texture);
}
