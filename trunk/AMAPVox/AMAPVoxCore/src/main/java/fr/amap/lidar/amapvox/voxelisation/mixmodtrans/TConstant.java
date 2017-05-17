package fr.amap.lidar.amapvox.voxelisation.mixmodtrans;

/**
 * @author Jimmy Lopez
 */
public class TConstant {

    /**
     * Nombre minimum de cellules échantillonnées pour appliquer le modèle glmer
     */
    public static int seuil_echantillonnage = 10;

    /**
     * Distance de voisinage (en cellules)
     */
    public static int minivox = 5;

    /**
     * Pas de discrétisation des volumes élémentaires
     */
    public static double  EP = 0.01;
}
