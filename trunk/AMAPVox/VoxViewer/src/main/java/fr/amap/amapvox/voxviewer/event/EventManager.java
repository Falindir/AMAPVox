/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.amap.amapvox.voxviewer.event;

import com.jogamp.opengl.util.FPSAnimator;
import fr.amap.amapvox.voxviewer.renderer.JoglListener;

/**
 * Abstract class that describes user input behavior
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */
public abstract class EventManager {
    
    /**
     * 
     */
    protected final FPSAnimator animator;

    /**
     *
     */
    protected final JoglListener joglContext;
    
    EventManager(FPSAnimator animator, JoglListener context)
    {
        this.animator = animator;
        this.joglContext = context;
    }
    
    /**
     * update events
     */
    void updateEvents(){};
}
