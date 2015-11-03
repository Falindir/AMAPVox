package fr.amap.amapvox.gui;

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


import fr.amap.amapvox.commons.util.MatrixFileParser;
import fr.amap.amapvox.commons.util.MatrixUtility;
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;
import org.apache.log4j.Logger;

/**
 * FXML Controller class
 *
 * @author calcul
 */


public class TransformationFrameController implements Initializable {
    
    final Logger logger = Logger.getLogger(TransformationFrameController.class);
    
    private final static String MATRIX_FORMAT_ERROR_MSG = "Matrix file has to look like this: \n\n\t1.0 0.0 0.0 0.0\n\t0.0 1.0 0.0 0.0\n\t0.0 0.0 1.0 0.0\n\t0.0 0.0 0.0 1.0\n";
    
    private boolean confirmed;
    
    private boolean modifyingAxisRotationPanel;
    private boolean modifyingEulerRotationPanel;
    private boolean modifyingTranslationPanel;
    private boolean modifyingTransformationMatrixPanel;
    
    private Stage stage;
    //private MainFrameController parent;
    
    private FileChooser fileChooserOpenMatrixFile;
    private File lastMatrixFile;
    
    @FXML
    private TextField labelAxisRotationAngle;
    @FXML
    private TextField labelEulerRotationX;
    @FXML
    private TextField labelEulerRotationY;
    @FXML
    private TextField labelEulerRotationZ;
    @FXML
    private TextField labelTranslationX;
    @FXML
    private TextField labelTranslationY;
    @FXML
    private TextField labelTranslationZ;
    @FXML
    private TextField matrixM00;
    @FXML
    private TextField matrixM01;
    @FXML
    private TextField matrixM02;
    @FXML
    private TextField matrixM03;
    @FXML
    private TextField matrixM10;
    @FXML
    private TextField matrixM11;
    @FXML
    private TextField matrixM12;
    @FXML
    private TextField matrixM13;
    @FXML
    private TextField matrixM20;
    @FXML
    private TextField matrixM21;
    @FXML
    private TextField matrixM22;
    @FXML
    private TextField matrixM23;
    @FXML
    private TextField matrixM30;
    @FXML
    private TextField matrixM31;
    @FXML
    private TextField matrixM32;
    @FXML
    private TextField matrixM33;
    @FXML
    private CheckBox checkboxInverseTransformation;
    @FXML
    private Button buttonSetIdentity;
    @FXML
    private Button buttonOpenMatrixFile;
    @FXML
    private Button buttonConfirm;
    @FXML
    private TextField labelAxisRotationX;
    @FXML
    private TextField labelAxisRotationY;
    @FXML
    private TextField labelAxisRotationZ;
    @FXML
    private Button buttonAcceptExtremumsPoints;
    @FXML
    private TextField textFieldPoint1X;
    @FXML
    private TextField textFieldPoint1Y;
    @FXML
    private TextField textFieldPoint1Z;
    @FXML
    private TextField textFieldPoint2X;
    @FXML
    private TextField textFieldPoint2Y;
    @FXML
    private TextField textFieldPoint2Z;
    
    /**
     * Initializes the controller class.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        
        setToIdentity();
        reset();
        
        labelTranslationX.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                try{
                    double value = Double.valueOf(newValue);
                    matrixM03.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        labelTranslationY.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                try{
                    double value = Double.valueOf(newValue);
                    matrixM13.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        labelTranslationZ.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                try{
                    double value = Double.valueOf(newValue);
                    matrixM23.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        checkboxInverseTransformation.selectedProperty().addListener(new ChangeListener<Boolean>() {

            @Override
            public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue) {
                
                Matrix4d matrix = getMatrix();
                matrix.invert();
                fillMatrix(matrix);
            }
        });
        
        matrixM03.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                try{
                    double value = Double.valueOf(newValue);
                    labelTranslationX.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        matrixM13.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                try{
                    double value = Double.valueOf(newValue);
                    labelTranslationY.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        matrixM23.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                try{
                    double value = Double.valueOf(newValue);
                    labelTranslationZ.setText(String.valueOf(value));
                    
                }catch(Exception e){}
            }
        });
        
        labelEulerRotationX.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                modifyingEulerRotationPanel = true;
                
                if(!modifyingTransformationMatrixPanel){
                    try{
                        double value = Double.valueOf(labelEulerRotationX.getText());
                        matrixM11.setText(String.valueOf(Math.cos(value)));
                        matrixM12.setText(String.valueOf(-Math.sin(value)));
                        matrixM21.setText(String.valueOf(Math.sin(value)));
                        matrixM22.setText(String.valueOf(Math.cos(value)));
                    }catch(Exception e){}
                }
                
                modifyingEulerRotationPanel = false;
                                
            }
        });
        
        labelEulerRotationY.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                modifyingEulerRotationPanel = true;
                
                if(!modifyingTransformationMatrixPanel){
                    try{
                        double value = Double.valueOf(labelEulerRotationY.getText());
                        matrixM00.setText(String.valueOf(Math.cos(value)));
                        matrixM02.setText(String.valueOf(Math.sin(value)));
                        matrixM20.setText(String.valueOf(-Math.sin(value)));
                        matrixM22.setText(String.valueOf(Math.cos(value)));
                    }catch(Exception e){}
                }
                
                
                modifyingEulerRotationPanel = false;
                
            }
        });
        
        labelEulerRotationZ.textProperty().addListener(new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                modifyingEulerRotationPanel = true;
                
                if(!modifyingTransformationMatrixPanel){
                    try{
                        double value = Double.valueOf(labelEulerRotationZ.getText());
                        matrixM00.setText(String.valueOf(Math.cos(value)));
                        matrixM01.setText(String.valueOf(-Math.sin(value)));
                        matrixM10.setText(String.valueOf(Math.sin(value)));
                        matrixM11.setText(String.valueOf(Math.cos(value)));
                    }catch(Exception e){}
                }
                
                modifyingEulerRotationPanel = false;
            }
        });
        
        ChangeListener axisRotationChangeListener = new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                modifyingAxisRotationPanel = true;
                
                if(!modifyingTransformationMatrixPanel){
                    calculateRotationMatrix();
                }
                
                modifyingAxisRotationPanel = false;
                
            }
        };
        
        
        labelAxisRotationX.textProperty().addListener(axisRotationChangeListener);
        labelAxisRotationY.textProperty().addListener(axisRotationChangeListener);
        labelAxisRotationZ.textProperty().addListener(axisRotationChangeListener);
        labelAxisRotationAngle.textProperty().addListener(axisRotationChangeListener);
        
        ChangeListener matrixElementChangeListener = new ChangeListener<String>() {

            @Override
            public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
                
                if(!modifyingAxisRotationPanel && !modifyingEulerRotationPanel){
                    modifyingTransformationMatrixPanel = true;
                    getEulerFromMatrix();
                    getRotationAxisAndAngle();
                    modifyingTransformationMatrixPanel = false;
                }
                
            }
        };
        
        matrixM00.textProperty().addListener(matrixElementChangeListener);
        matrixM01.textProperty().addListener(matrixElementChangeListener);
        matrixM02.textProperty().addListener(matrixElementChangeListener);
        matrixM10.textProperty().addListener(matrixElementChangeListener);
        matrixM11.textProperty().addListener(matrixElementChangeListener);
        matrixM12.textProperty().addListener(matrixElementChangeListener);
        matrixM20.textProperty().addListener(matrixElementChangeListener);
        matrixM21.textProperty().addListener(matrixElementChangeListener);
        matrixM22.textProperty().addListener(matrixElementChangeListener);
        
        
        fileChooserOpenMatrixFile = new FileChooser();
        fileChooserOpenMatrixFile.setTitle("Choose Matrix file");
    }
    
    private void calculateRotationMatrix(){
        
        double uX = 0, uY = 0, uZ = 0;
        
        try{
            uX = Double.valueOf(labelAxisRotationX.getText());
        }catch(Exception e){
            labelAxisRotationX.setText(String.valueOf(uX));
        }
        
        try{
            uY = Double.valueOf(labelAxisRotationY.getText());
        }catch(Exception e){
            labelAxisRotationY.setText(String.valueOf(uY));
        }
        
        try{
            uZ = Double.valueOf(labelAxisRotationZ.getText());
        }catch(Exception e){
            labelAxisRotationZ.setText(String.valueOf(uZ));
        }
        
        Vector3d vec = new Vector3d(uX, uY, uZ);
        vec.normalize();
        
        if(!Double.isNaN(vec.length())){
            uX = vec.x;
            uY = vec.y;
            uZ = vec.z;

            double angle = 0;

            try{
                angle = Double.valueOf(labelAxisRotationAngle.getText());
            }catch(Exception e){
                labelAxisRotationAngle.setText(String.valueOf(angle));
            }

            double c = Math.cos(Math.toRadians(angle));
            double s = Math.sin(Math.toRadians(angle));

            matrixM00.setText(String.valueOf((uX*uX)+(1-(uX*uX))*c));
            matrixM01.setText(String.valueOf((uX*uY)*(1-c)-(uZ*s)));
            matrixM02.setText(String.valueOf((uX*uZ)*(1-c)+(uY*s)));
            matrixM10.setText(String.valueOf((uX*uY)*(1-c)+(uZ*s)));
            matrixM11.setText(String.valueOf((uY*uY)+(1-(uY*uY))*c));
            matrixM12.setText(String.valueOf((uY*uZ)*(1-c)-(uX*s)));
            matrixM20.setText(String.valueOf((uX*uZ)*(1-c)-(uY*s)));
            matrixM21.setText(String.valueOf((uY*uZ)*(1-c)+(uX*s)));
            matrixM22.setText(String.valueOf((uZ*uZ)+(1-(uZ*uZ))*c));
        }
        
    }
    
    public void setStage(Stage stage) {
        this.stage = stage;
    }
    
    /*public void setParent(MainFrameController controller){
        this.parent = controller;
    }*/
    
    public void fillMatrix(Matrix4d matrix){
        
        matrixM00.setText(String.valueOf(matrix.m00));
        matrixM01.setText(String.valueOf(matrix.m01));
        matrixM02.setText(String.valueOf(matrix.m02));
        matrixM03.setText(String.valueOf(matrix.m03));
        matrixM10.setText(String.valueOf(matrix.m10));
        matrixM11.setText(String.valueOf(matrix.m11));
        matrixM12.setText(String.valueOf(matrix.m12));
        matrixM13.setText(String.valueOf(matrix.m13));
        matrixM20.setText(String.valueOf(matrix.m20));
        matrixM21.setText(String.valueOf(matrix.m21));
        matrixM22.setText(String.valueOf(matrix.m22));
        matrixM23.setText(String.valueOf(matrix.m23));
        matrixM30.setText(String.valueOf(matrix.m30));
        matrixM31.setText(String.valueOf(matrix.m31));
        matrixM32.setText(String.valueOf(matrix.m32));
        matrixM33.setText(String.valueOf(matrix.m33));
    }
    
    public Matrix4d getMatrix(){
        
        Matrix4d matrix;
        
        try{
            matrix = new Matrix4d(
                Double.valueOf(matrixM00.getText()),
                Double.valueOf(matrixM01.getText()), 
                Double.valueOf(matrixM02.getText()), 
                Double.valueOf(matrixM03.getText()), 
                Double.valueOf(matrixM10.getText()),
                Double.valueOf(matrixM11.getText()),
                Double.valueOf(matrixM12.getText()), 
                Double.valueOf(matrixM13.getText()),
                Double.valueOf(matrixM20.getText()),
                Double.valueOf(matrixM21.getText()), 
                Double.valueOf(matrixM22.getText()),
                Double.valueOf(matrixM23.getText()),
                Double.valueOf(matrixM30.getText()),
                Double.valueOf(matrixM31.getText()),
                Double.valueOf(matrixM32.getText()),
                Double.valueOf(matrixM33.getText()));
            
            return matrix;
            
        }catch(Exception e){}
        
        matrix = new Matrix4d();
        matrix.setIdentity();
        
        return matrix;
        
    }
    
    private void getRotationAxisAndAngle(){
        
        try{
            double m00 = Double.valueOf(matrixM00.getText());
            double m11 = Double.valueOf(matrixM11.getText());
            double m22 = Double.valueOf(matrixM22.getText());
            double m21 = Double.valueOf(matrixM21.getText());
            double m12 = Double.valueOf(matrixM12.getText());
            double m02 = Double.valueOf(matrixM02.getText());
            double m20 = Double.valueOf(matrixM20.getText());
            double m10 = Double.valueOf(matrixM10.getText());
            double m01 = Double.valueOf(matrixM01.getText());

            double trace = m00 + m11 + m22;
            double cos_t = (trace - 1)/2.0d;

            double alpha_rad;

            Vector3d axis = new Vector3d();

            if (Math.abs(cos_t) <= 1)
            {
                alpha_rad = Math.acos(cos_t); //result in [0;pi]
            }else{
                alpha_rad = 0;
            }

            axis.x = m21-m12;
            axis.y = m02-m20;
            axis.z = m10-m01;

            //normalize axis
            double n2 = axis.lengthSquared();

            if (n2 > 0)
            {
                axis.x /= Math.sqrt(n2);
                axis.y /= Math.sqrt(n2);
                axis.z /= Math.sqrt(n2);
            }
            else
            {
                //axis is too small!
                axis = new Vector3d(0, 0, 1);
            }

            labelAxisRotationAngle.setText(String.valueOf(Math.toDegrees(alpha_rad)));

            labelAxisRotationX.setText(String.valueOf(axis.x));
            labelAxisRotationY.setText(String.valueOf(axis.y));
            labelAxisRotationZ.setText(String.valueOf(axis.z));
            
        }catch(Exception e){
            
        }
        
    }
    
    private void getEulerFromMatrix(){
        
        try{
            double m20 = Double.valueOf(matrixM20.getText());
            double m21 = Double.valueOf(matrixM21.getText());
            double m22 = Double.valueOf(matrixM22.getText());
            double m00 = Double.valueOf(matrixM00.getText());
            double m10 = Double.valueOf(matrixM10.getText());
            double m01 = Double.valueOf(matrixM01.getText());
            double m02 = Double.valueOf(matrixM02.getText());


            double theta_rad;
            double cos_theta;
            double psi_rad;
            double phi_rad;

            if (Math.abs(m20) != 1){

                    theta_rad = -Math.asin(m20);
                    cos_theta = Math.cos(theta_rad);
                    psi_rad = Math.atan2(m21/cos_theta, m22/cos_theta);
                    phi_rad = Math.atan2(m10/cos_theta, m00/cos_theta);
            }else{
                    phi_rad = 0;

                    if (m20 == -1)
                    {
                        theta_rad = (Math.PI)/2.0d;
                        psi_rad = Math.atan2(m01,m02);
                    }
                    else
                    {
                        theta_rad = -(Math.PI)/2.0d;
                        psi_rad = -Math.atan2(m01,m02);
                    }
            }

            labelEulerRotationX.setText(String.valueOf(Math.toDegrees(psi_rad)));
            labelEulerRotationY.setText(String.valueOf(Math.toDegrees(theta_rad)));
            labelEulerRotationZ.setText(String.valueOf(Math.toDegrees(phi_rad)));
        }catch(Exception e){
            
        }
        
    }
    
    private void setToIdentity(){
        
        Matrix4d matrix = new Matrix4d();
        matrix.setIdentity();
        fillMatrix(matrix);
    }

    @FXML
    private void onActionButtonSetIdentity(ActionEvent event) {
        setToIdentity();
    }

    @FXML
    private void onActionButtonOpenMatrixFile(ActionEvent event) {
        
        if(lastMatrixFile != null){
            fileChooserOpenMatrixFile.setInitialDirectory(lastMatrixFile.getParentFile());
        }
        
        File selectedFile = fileChooserOpenMatrixFile.showOpenDialog(stage);
        
        if (selectedFile != null) {

            lastMatrixFile = selectedFile;
            
            try {
                
                Matrix4d mat = MatrixFileParser.getMatrixFromFile(selectedFile);
                
                if (mat != null) {

                    modifyingTransformationMatrixPanel = true;

                    fillMatrix(mat);

                    modifyingTransformationMatrixPanel = false;

                } else {
                    showMatrixFormatErrorDialog();
                }
                
            } catch (IOException ex) {
                logger.error("Cannot open matrix file : "+selectedFile.getAbsolutePath(), ex);
            }
        }
    }

    @FXML
    private void onActionButtonConfirm(ActionEvent event) {
        confirmed = true;
        stage.close();
    }
    
    public void showMatrixFormatErrorDialog() {

        Alert alert = new Alert(Alert.AlertType.ERROR);
        alert.setTitle("Error");
        alert.setHeaderText("Impossible to parse matrix file");
        alert.setContentText(MATRIX_FORMAT_ERROR_MSG);

        alert.showAndWait();
    }


    public boolean isConfirmed() {
        return confirmed;
    }
    
    public void reset(){
        
        confirmed = false;
        
        modifyingAxisRotationPanel = false;
        modifyingEulerRotationPanel = false;
        modifyingTransformationMatrixPanel = false;
        modifyingTranslationPanel = false;
    }

    @FXML
    private void onActionButtonAcceptExtremumsPoints(ActionEvent event) {
        
        try{
            Vector3d point1 = new Vector3d(Double.valueOf(textFieldPoint1X.getText()), Double.valueOf(textFieldPoint1Y.getText()), Double.valueOf(textFieldPoint1Z.getText()));
            Vector3d point2 = new Vector3d(Double.valueOf(textFieldPoint2X.getText()), Double.valueOf(textFieldPoint2Y.getText()), Double.valueOf(textFieldPoint2Z.getText()));
            Matrix4d matrix = MatrixUtility.getMatrixTransformation(point1, point2);

            if(matrix != null){
                fillMatrix(matrix);
            }
        }catch(Exception e){
            
        }
        
    }
}
