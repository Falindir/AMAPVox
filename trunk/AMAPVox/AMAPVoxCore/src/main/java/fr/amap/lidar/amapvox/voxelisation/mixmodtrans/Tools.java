package fr.amap.lidar.amapvox.voxelisation.mixmodtrans;

import fr.amap.lidar.amapvox.commons.Voxel;

import java.util.concurrent.TimeUnit;

/**
 * @author Jimmy Lopez
 */
public class Tools {

    /**
     * Pour avoir la même fonction round que dans le langage R
     * @param d
     * @return
     */
    public static float round(float d) {
        if(Double.isNaN(d))
            return Float.NaN;

        int temp = (int )d;

        if(d < 1 && d > 0.5)
            return 1;

        if(d <= 0.5 && d >= 0)
            return 0;

        if(temp % 2 == 0) {
            if(d > 0) {
                float temp2 = (float) (temp + 0.5);
                if(d > temp2) {
                    return Math.round(d) ;
                }
                else if(d < temp2) {
                    return Math.round(d);
                }
                else {
                    return Math.round(d) - 1;
                }
            }
            else {
                return Math.round(d);
            }
        }
        else if(temp % 2 != 0) {
            if(d > 0) {
                return Math.round(d);
            }
            else {
                float temp2 = (float) (temp - 0.5);

                if(d > temp2) {
                    return Math.round(d);
                }
                else if(d < temp2) {
                    return Math.round(d);
                }
                else {
                    return Math.round(d) - 1 ;
                }
            }
        }
        return 0;
    }

    /**
     * Permets de créer un ID à partir des coordonnées ijk d'un Voxel
     * @param voxel
     * @return $i_$j_$k
     */
    public static String createIJKVoxel(Voxel voxel) {
        return voxel.$i + "_" + voxel.$j + "_" + voxel.$k;
    }

    public static String elapsed(long duration) {
        final TimeUnit scale = TimeUnit.MILLISECONDS;

        long hours = scale.toHours(duration);
        duration -= TimeUnit.HOURS.toMillis(hours);
        long minutes = scale.toMinutes(duration);
        duration -= TimeUnit.MINUTES.toMillis(minutes);
        long seconds = scale.toSeconds(duration);
        duration -= TimeUnit.SECONDS.toMillis(seconds);
        long millis = scale.toMillis(duration);

        return String.format(
                "%d hours, %d minutes, %d seconds, %d millis",
                hours, minutes, seconds, millis );
    }

}
