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

package fr.amap.amapvox.voxviewer.mesh;

import com.jogamp.opengl.GL3;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */


public class SimpleGLMesh extends GLMesh{

    public SimpleGLMesh() {
        
        totalBuffersSize = 0;
        offset = 0;
        offsets = new ArrayList<>();
        buffersSizes = new ArrayList<>();
    }
    
    public SimpleGLMesh(GL3 gl) {
        super(gl);
    }

    @Override
    public void initBuffers(GL3 gl, long maximumTotalBufferSize) {
        
        initVBOAndIBO(gl);
        
        bindBuffer(gl);
        
        List<FloatBuffer> floatBuffers = new ArrayList<>();
        floatBuffers.add(vertexBuffer);
        if(colorBuffer != null){
            floatBuffers.add(colorBuffer);
        }
        if(normalBuffer != null){
            floatBuffers.add(normalBuffer);
        }
        
        if(maximumTotalBufferSize == DEFAULT_SIZE){
            for (FloatBuffer buffer : floatBuffers) {
                
                if(buffer != null){
                    totalBuffersSize += buffer.capacity()*FLOAT_SIZE;
                }
                
            }
        }else{
            totalBuffersSize = maximumTotalBufferSize;
        }
        
        gl.glBufferData(GL3.GL_ARRAY_BUFFER, totalBuffersSize, null, GL3.GL_STATIC_DRAW);
        
        for (FloatBuffer buffer : floatBuffers) {
            addSubBuffer(gl, buffer);
        }
        
        sendIBOData(gl);
        
        unbindBuffer(gl);
    }

    @Override
    public void draw(GL3 gl) {
        gl.glDrawElements(drawType, vertexCount, GL3.GL_UNSIGNED_INT, 0);
    }
    
}
