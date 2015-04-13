/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar;

import java.io.File;
import java.io.Serializable;
import javax.vecmath.Matrix4d;

/**
 *
 * @author calcul
 */
public class MatrixAndFile implements Serializable{
    
    public File file;
    public Matrix4d matrix;

    public MatrixAndFile(File file, Matrix4d matrix) {
        this.file = file;
        this.matrix = matrix;
    }
    
    @Override
    public String toString(){
        return file.getAbsolutePath();
    }
}