
package org.micromanager.asidispim;

import ij.IJ;
import ij.ImagePlus;
import ij.ImageStack;
import ij.WindowManager;
import ij.process.ImageProcessor;

import java.awt.Cursor;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.io.File;
import java.util.concurrent.ExecutionException;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFormattedTextField; 
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JTextField;
import javax.swing.SwingWorker;

import net.miginfocom.swing.MigLayout;
import org.micromanager.PropertyMap;

import org.micromanager.Studio;
import org.micromanager.asidispim.data.AcquisitionModes;
import org.micromanager.internal.utils.FileDialogs;

import org.micromanager.asidispim.data.Devices; 
import org.micromanager.asidispim.data.MyStrings;
import org.micromanager.asidispim.data.Prefs;
import org.micromanager.asidispim.data.Properties;
import org.micromanager.asidispim.utils.ImageJUtils.IJCommandThread;
import org.micromanager.asidispim.utils.ImageUtils;
import org.micromanager.asidispim.utils.ListeningJPanel;
import org.micromanager.asidispim.utils.MyDialogUtils;
import org.micromanager.asidispim.utils.PanelUtils;
import org.micromanager.data.Coordinates;
import org.micromanager.data.Coords;
import org.micromanager.data.Datastore;
import org.micromanager.data.SummaryMetadata;
import org.micromanager.display.DisplayWindow;
import org.micromanager.internal.utils.NumberUtils;
import org.micromanager.internal.utils.ReportingUtils;


/**
 * Panel in ASIdiSPIM plugin specifically for data analysis/processing
 * For now, we provide a way to export Micro-Manager datasets into 
 * a mipav compatible format 
 * mipav likes data in a folder as follows:
 * folder - SPIMA - name_SPIMA-0.tif, name_SPIMA-x.tif, name_SPIMA-n.tif
 *        - SPIMB - name_SPIMB-0.tif, name_SPIMB-x.tif, name_SPIMB-n.tif
 * 
 * TODO: make mipav export work for single sided acquisitions
 * TODO: make mipav export work for multi-position acquisitions
 * @author Nico
 */
@SuppressWarnings("serial")
public class DataAnalysisPanel extends ListeningJPanel {
   private final Studio gui_;
   private final Prefs prefs_;
   private final Properties props_; 
 	private final Devices devices_; 
   private final JPanel exportPanel_;
   private final JPanel deskewPanel_;
   private final JPanel imageJPanel_;
   private final JTextField saveDestinationField_;
   private final JTextField baseNameField_;
   private final JFormattedTextField deskewFactor_; 
   private final JCheckBox deskewInvert_; 
 	private final JCheckBox deskewInterpolate_; 
   private final JCheckBox deskewAutoTest_; 
   private final JButton exportButton_; 
   
   public static final String[] TRANSFORMOPTIONS = 
      {"None", "Rotate Right 90\u00B0", "Rotate Left 90\u00B0", "Rotate outward",
       "Rotate 180\u00B0"};
   public static final String[] EXPORTFORMATS = 
      {"mipav GenerateFusion", "Multiview Reconstruction (deprecated)"};
   public static FileDialogs.FileType EXPORT_DATA_SET 
           = new FileDialogs.FileType("EXPORT_DATA_SET",
                 "Export to Location",
                 System.getProperty("user.home") + "/Untitled",
                 false, (String[]) null);
   
   /**
    * 
    * @param gui
    * @param prefs - Plugin-wide preferences
    */
   public DataAnalysisPanel(Studio gui, Prefs prefs, Properties props, Devices devices) {    
      super(MyStrings.PanelNames.DATAANALYSIS.toString(),
              new MigLayout(
              "",
              "[right]",
              "[]16[]"));
      gui_ = gui;
      prefs_ = prefs;
      props_ = props; 
 	   devices_ = devices; 
 	   PanelUtils pu = new PanelUtils(prefs_, props_, devices); 
      final DataAnalysisPanel dataAnalysisPanel = this; 
      
      int textFieldWidth = 35;

      // start export sub-panel
      exportPanel_ = new JPanel(new MigLayout(
              "",
              "[right]4[center]4[left]",
              "[]8[]"));


      exportPanel_.setBorder(PanelUtils.makeTitledBorder("Export diSPIM data", 
              exportPanel_));
      
      exportPanel_.add(new JLabel("Export directory:"), "");
      
      saveDestinationField_ = new JTextField();
      saveDestinationField_.setText(prefs_.getString(panelName_,
              Properties.Keys.PLUGIN_EXPORT_DATA_DIR, ""));
      saveDestinationField_.setColumns(textFieldWidth);
      saveDestinationField_.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(final ActionEvent e) {
             prefs_.putString(panelName_, Properties.Keys.PLUGIN_EXPORT_DATA_DIR,
                    saveDestinationField_.getText());
         }
      });
      exportPanel_.add(saveDestinationField_);
      
      JButton browseToSaveDestinationButton = new JButton();
      browseToSaveDestinationButton.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(final ActionEvent e) {
            setSaveDestinationDirectory(saveDestinationField_);
            prefs_.putString(panelName_, Properties.Keys.PLUGIN_EXPORT_DATA_DIR,
                    saveDestinationField_.getText());
         }
      });
      
      browseToSaveDestinationButton.setMargin(new Insets(2, 5, 2, 5));
      browseToSaveDestinationButton.setText("...");
      exportPanel_.add(browseToSaveDestinationButton, "wrap");
      
      exportPanel_.add(new JLabel("Base Name:"), "");
      baseNameField_ = new JTextField();
      proposeBaseFieldText();
      baseNameField_.setColumns(textFieldWidth);
      exportPanel_.add(baseNameField_, "wrap");
      
      
      // row with transform options
      JLabel transformLabel = new JLabel("Transform:");
      exportPanel_.add(transformLabel);
      final JComboBox transformSelect = new JComboBox();
      for (String item : TRANSFORMOPTIONS) {
         transformSelect.addItem(item);
      }
      String transformOption = prefs_.getString(
              panelName_, Properties.Keys.PLUGIN_EXPORT_TRANSFORM_OPTION, 
              TRANSFORMOPTIONS[1]);
      transformSelect.setSelectedItem(transformOption);
      transformSelect.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            prefs_.putString(panelName_, 
                    Properties.Keys.PLUGIN_EXPORT_TRANSFORM_OPTION, 
                    (String)transformSelect.getSelectedItem());
         }
      });
      exportPanel_.add(transformSelect, "left, wrap");
      
      // row with output options
      JLabel exportFormatLabel = new JLabel("Export for:");
      exportPanel_.add(exportFormatLabel);
      final JComboBox exportFormatSelect = new JComboBox();
      for (String item : EXPORTFORMATS) {
         exportFormatSelect.addItem(item);
      }
      String exportFormatOption = prefs_.getString(
              panelName_, Properties.Keys.PLUGIN_EXPORT_FORMAT, 
              EXPORTFORMATS[1]);
      exportFormatSelect.setSelectedItem(exportFormatOption);
      exportFormatSelect.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            prefs_.putString(panelName_, 
                    Properties.Keys.PLUGIN_EXPORT_FORMAT, 
                    (String)exportFormatSelect.getSelectedItem());
         }
      });
      exportPanel_.add(exportFormatSelect, "left, wrap");
      
      
      final JProgressBar progBar = new JProgressBar();
      progBar.setStringPainted(true);
      progBar.setVisible(false);
      final JLabel infoLabel = new JLabel("");
     
      exportButton_ = new JButton("Export");
      exportButton_.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
            ExportTask task = new ExportTask(saveDestinationField_.getText(),
                    baseNameField_.getText(),
                    transformSelect.getSelectedIndex(), 
                    exportFormatSelect.getSelectedIndex() );
            task.addPropertyChangeListener(new PropertyChangeListener() {

               @Override
               public void propertyChange(PropertyChangeEvent evt) {
                  if ("progress".equals(evt.getPropertyName())) {
                     int progress = (Integer) evt.getNewValue();
                     if (!progBar.isVisible()) {
                        progBar.setVisible(true);
                        infoLabel.setText("Saving...");
                        infoLabel.setVisible(true);
                     }
                     progBar.setValue(progress);
                     if (progress == 100) {
                        progBar.setVisible(false);
                        infoLabel.setText("Done Saving...");
                     }
                  }
               }
            });
            task.execute();
         }
      });
      exportPanel_.add(exportButton_, "span 3, center, wrap");
      exportPanel_.add(infoLabel,"");
      exportPanel_.add(progBar, "span3, center, wrap");    
      
      super.add(exportPanel_);
      // end export sub-panel

      // start ImageJ sub-panel
      imageJPanel_ = new JPanel(new MigLayout(
              "",
              "[center]",
              "[]8[]"));

      imageJPanel_.setBorder(PanelUtils.makeTitledBorder("ImageJ", imageJPanel_));

      JButton adjustBC = new JButton("Brightness/Contrast");
      adjustBC.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            IJCommandThread t = new IJCommandThread("Brightness/Contrast...");
            t.start();
         }
      });
      imageJPanel_.add(adjustBC, "wrap");

      JButton splitChannels = new JButton("Split Channels");
      splitChannels.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            IJCommandThread t = new IJCommandThread("Split Channels");
            t.start();
         }
      });
      imageJPanel_.add(splitChannels, "wrap");

      JButton zProjection = new JButton("Z Projection");
      zProjection.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e) {
            IJCommandThread t = new IJCommandThread("Z Project...", "projection=[Max Intensity]");
            t.start();
         }
      });
      imageJPanel_.add(zProjection, "wrap");

      // end ImageJ sub-panel
      super.add(imageJPanel_);

      // start deskew sub-panel 
      deskewPanel_ = new JPanel(new MigLayout(
              "",
              "[right]4[center]4[left]",
              "[]8[]"));

      deskewPanel_.setBorder(PanelUtils.makeTitledBorder("Deskew stage scanning data", this));

      deskewPanel_.add(new JLabel("Deskew fudge factor:"));
      deskewFactor_ = pu.makeFloatEntryField(panelName_,
              Properties.Keys.PLUGIN_DESKEW_FACTOR.toString(), 1.0, 5);
      deskewPanel_.add(deskewFactor_, "wrap");
      
      deskewInvert_ = pu.makeCheckBox("Invert direction",
              Properties.Keys.PLUGIN_DESKEW_INVERT, panelName_, false);
      deskewPanel_.add(deskewInvert_, "left, span 2, wrap");

      deskewInterpolate_ = pu.makeCheckBox("Interpolate",
              Properties.Keys.PLUGIN_DESKEW_INTERPOLATE, panelName_, false);
      deskewPanel_.add(deskewInterpolate_, "left, span 2, wrap");

      deskewAutoTest_ = pu.makeCheckBox("Auto-deskew test acquisitions", 
 		        Properties.Keys.PLUGIN_DESKEW_AUTO_TEST, panelName_, false); 
 		deskewPanel_.add(deskewAutoTest_, "left, span 2, wrap"); 
            
      JButton deskewButton = new JButton("Deskew Open Dataset");
      deskewButton.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(final ActionEvent e) {
            runDeskew(dataAnalysisPanel);
         }
      });
      deskewPanel_.add(deskewButton, "span 3, center, wrap");

      super.add(deskewPanel_);
   }

   public void runDeskew(final ListeningJPanel caller) {
      /**
       * Worker thread to execute deskew. Patterned after Nico's ExportTask
       * SwingWorker code but updating progress bar wasn't working and task is
       * pretty quick so I removed that code.
       *
       * @author Jon
       *
       */
      class DeskewTask extends SwingWorker<Void, Void> {

         DeskewTask() {
            // empty constructor for now
         }

         @Override
         public Void doInBackground() throws Exception {
            setProgress(0);
            setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));

            long startTime = System.currentTimeMillis();
            final DisplayWindow currentWindow = gui_.displays().getCurrentWindow();
            final ImagePlus ip;
            final boolean firstSideIsA;
            final boolean twoSided;
            String windowTitle;
            final AcquisitionModes.Keys acqMode;
            double zStepPx = 0.0;
            if (currentWindow != null) {
               ip = currentWindow.getImagePlus();

               final Datastore datastore = currentWindow.getDatastore();
               final SummaryMetadata summaryMetadata = datastore.getSummaryMetadata();
               PropertyMap metadata = summaryMetadata.getUserData();

               acqMode = AcquisitionModes.getKeyFromString(metadata.getString("SPIMmode"));
               if (!(acqMode == AcquisitionModes.Keys.STAGE_SCAN
                       || acqMode == AcquisitionModes.Keys.STAGE_SCAN_INTERLEAVED
                       || acqMode == AcquisitionModes.Keys.STAGE_SCAN_UNIDIRECTIONAL)) {
                  throw new Exception("Can only deskew stage scanning data");
               }
               firstSideIsA = !metadata.getString("FirstSide").equals("B");
               twoSided = metadata.getString("NumberOfSides").equals("2"); 

               if (metadata.containsKey("AcquisitionName")) {
                  windowTitle = metadata.getString("AcquisitionName");
               } else {
                  windowTitle = ip.getTitle();
               }
               if (metadata.containsString("PixelSize_um") && metadata.containsString("z-step_um")) {
                  // with test acquisitions ip.getCalibration() isn't correct for some reason so prefer metadata 
                  double pixelSize = NumberUtils.coreStringToDouble(metadata.getString("PixelSize_um"));
                  if (pixelSize < 1e-6) {
                     throw new Exception("Cannot have pixel size of 0");
                  }
                  zStepPx = NumberUtils.coreStringToDouble(metadata.getString("z-step_um")) / pixelSize;
               }
            } else {
               ip = IJ.getImage();
               if (ip == null) {
                  throw new Exception("No display open");
               }
               // guess at settings since we can't access MM metadata 
               firstSideIsA = true;
               twoSided = true; 
               acqMode = AcquisitionModes.Keys.STAGE_SCAN;
               windowTitle = ip.getTitle(); 
 	            ReportingUtils.logDebugMessage("Deskew may be incorrect because don't have Micro-Manager dataset with metadata");
            }

            // if zStepPx wasn't set from MM metadata then get value from ImagePlus object  
            if (zStepPx < 1e-6) {
               double pixelSize = ip.getCalibration().pixelWidth;
               if (pixelSize < 1e-6) {
                  throw new Exception("Cannot have pixel size of 0");
               }
               zStepPx = ip.getCalibration().pixelDepth / pixelSize;
            }

            // for 45 degrees we shift the same amount as the interplane spacing, so factor of 1.0 
            // assume diSPIM unless marked specifically otherwise 
            // I don't understand why mathematically but it seems that for oSPIM the factor is 1.0 
            //   too instead of being tan(60 degrees) due to the rotation 
            final double dx = zStepPx * (Double) deskewFactor_.getValue();

            final int sc = ip.getNChannels();
            final int sx = ip.getWidth();
            final int sy = ip.getHeight();
            final int ss = ip.getNSlices();
            final String title = ip.getTitle() + "-deskewed";
            final int sx_new = sx + (int) Math.abs(Math.ceil(dx * ss));

            if (sc > 1) {
               IJ.run("Duplicate...", "title=" + title + " duplicate");
               IJ.run("Split Channels");
            } else {
               IJ.run("Duplicate...", "title=C1-" + title + " duplicate");  // make it named as 1st channel would be 
            }
            String mergeCmd = "";
            int dir;
            for (int c = 0; c < sc; c++) {    // loop over channels 
               IJ.selectWindow("C" + (c + 1) + "-" + title);
               switch (acqMode) {
                  case STAGE_SCAN:
                     if (twoSided) {
                        dir = (c % 2) * 2 - 1;  // -1 for path A which are odd channels, 1 for path B 
                     } else {
                        // single-sided is path A for all channels 
                        dir = -1;
                     }
                     // invert direction if we started with path B, regardless of single- or double-sided 
                     if (!firstSideIsA) {
                        dir *= -1;
                     }
                     break;
                  case STAGE_SCAN_INTERLEAVED:
                  case STAGE_SCAN_UNIDIRECTIONAL:
                     // always the same direction 
                     dir = -1;
                     break;
                  default:
                     // should never make it here 
                     throw new Exception("Can only deskew stage scanning data");
               }
               if (deskewInvert_.isSelected()) {
                  dir *= -1;
               }
               
               IJ.run("Canvas Size...", "width=" + sx_new + " height=" + sy + " position=Center-"
                       + (dir < 0 ? "Right" : "Left") + " zero");
               for (int s = 0; s < ss; s++) {  // loop over slices in stack 
                  IJ.setSlice(s + 1);
                  IJ.run("Translate...", "x=" + (dx * s * dir) + " y=0 interpolation="
                          + (deskewInterpolate_.isSelected() ? "Bilinear slice" : "None"));
               }
               mergeCmd += ("c" + (c + 1) + "=C" + (c + 1) + "-" + title + " ");
            }
            IJ.run("Merge Channels...", mergeCmd + "create");
            if (sc > 1) {
               IJ.run("Merge Channels...", mergeCmd + "create");
            } else {
               IJ.run("Rename...", "title=" + title);
            }
            long finishTime = System.currentTimeMillis();
            ReportingUtils.logDebugMessage("Deskew operation took " + (finishTime - startTime)
                    + " milliseconds with total of " + (sc * ss) + " images");

            return null;
         }

         @Override
         public void done() {
            setCursor(null);
            try {
               get();
               setProgress(100);
            } catch (ExecutionException ex) {
               Throwable cause = ex.getCause();
               if (!cause.getMessage().equals("Macro canceled")) {
                  if (cause instanceof SaveTaskException) {
                     MyDialogUtils.showError(cause, "Deskew error");
                  } else {
                     MyDialogUtils.showError(ex);
                  }
               }
            } catch (InterruptedException ex) {
               MyDialogUtils.showError(ex, "Interrupted while deskewing data");
            }
         }
      }

      // runDeskew() code goes here
      (new DeskewTask()).execute();
   }

   

@Override
        public void gotSelected() {
      proposeBaseFieldText();
   }

   private void proposeBaseFieldText() {
      ImagePlus ip = WindowManager.getCurrentImage();
      if (ip != null) {
         String baseName = ip.getShortTitle();
         baseName = baseName.replaceAll("[^a-zA-Z0-9_\\.\\-]", "_");
         baseNameField_.setText(baseName);
      

}
   }
   
   
    /**
     * Worker thread that executes file saving. Updates the progress bar using
     * the setProgress method, which results in a PropertyChangedEvent in
     * attached listeners
     */
    class ExportTask extends SwingWorker<Void, Void> {

        final String targetDirectory_;
        final String baseName_;
        final int transformIndex_;
        final int exportFormat_;

        ExportTask(String targetDirectory, String baseName,
                int transformIndex, int exportFormat) {
            targetDirectory_ = targetDirectory;
            baseName_ = baseName.replaceAll("[^a-zA-Z0-9_\\.\\-]", "_");
            transformIndex_ = transformIndex;
            exportFormat_ = exportFormat;
        }

        @Override
        protected Void doInBackground() throws Exception {
            setProgress(0);
            DisplayWindow dw = gui_.displays().getCurrentWindow();

            if (null == dw) {
                throw new SaveTaskException("Can only convert Micro-Manager data set ");
            }
            ImagePlus ip = dw.getImagePlus();
            Datastore store = dw.getDatastore();

            if (exportFormat_ == 0) { // mipav

                ImageProcessor iProc = ip.getProcessor();
                if (!store.getSummaryMetadata().getUserData().getString("NumberOfSides").equals("2")) {
                    throw new SaveTaskException("mipav export only works with two-sided data for now.");
                }
                if (store.getNextIndex(Coords.STAGE_POSITION) > 1) {
                    throw new SaveTaskException("mipav export does not yet work with multiple positions");
                }

                boolean usesChannels = store.getNextIndex(Coords.CHANNEL) > 2;  // if have channels besides two cameras
                String[] channelDirArray = new String[store.getNextIndex(Coords.CHANNEL)];
                if (usesChannels) {
                    for (int c = 0; c < store.getNextIndex(Coords.CHANNEL); c++) {
                        String chName = store.getSummaryMetadata().getUserData().
                                getStringArray("ChNames")[c];
                        String colorName = chName.substring(chName.indexOf("-") + 1);  // matches with AcquisitionPanel naming convention
                        channelDirArray[c] = targetDirectory_ + File.separator + baseName_ + File.separator
                                + (((c % 2) == 0) ? "SPIMA" : "SPIMB") + File.separator + colorName;
                    }
                } else {
                    channelDirArray[0] = targetDirectory_ + File.separator + baseName_
                            + File.separator + "SPIMA";
                    channelDirArray[1] = targetDirectory_ + File.separator + baseName_
                            + File.separator + "SPIMB";
                }

                for (String dir : channelDirArray) {
                    if (new File(dir).exists()) {
                        throw new SaveTaskException("Output directory already exists");
                    }
                }

                for (String dir : channelDirArray) {
                    new File(dir).mkdirs();
                }

                final int nrCh = store.getNextIndex(Coords.CHANNEL);
                final int nrFr = store.getNextIndex(Coords.T);
                final int nrZ = store.getNextIndex(Coords.Z);
                final int totalNr = nrCh * nrFr * nrZ;
                int counter = 0;

                for (int c = 0; c < nrCh; c++) {  // for each channel
                    for (int t = 0; t < nrFr; t++) {  // for each timepoint
                        ImageStack stack = new ImageStack(iProc.getWidth(), iProc.getHeight());
                        for (int i = 0; i < store.getNextIndex(Coords.Z); i++) {
                            Coords coords = Coordinates.builder().channel(c).t(t).z(i).build();
                            // TODO make utility that converts Image into ImageProcessor
                            ImageProcessor iProc2 = ImageUtils.getImageProcessor(store.getImage(coords));

                            // optional transformation
                            switch (transformIndex_) {
                                case 1: {
                                    iProc2.rotate(90);
                                    break;
                                }
                                case 2: {
                                    iProc2.rotate(-90);
                                    break;
                                }
                                case 3: {
                                    iProc2.rotate(((c % 2) == 1) ? 90 : -90);
                                    break;
                                }
                                case 4: { 
                                    iProc2.rotate(180); 
                                    break; 
                                }
                            }

                            stack.addSlice(iProc2);
                            counter++;
                            double rate = ((double) counter / (double) totalNr) * 100.0;
                            setProgress((int) Math.round(rate));
                        }
                        ImagePlus ipN = new ImagePlus("tmp", stack);
                        ipN.setCalibration(ip.getCalibration());
                        ij.IJ.save(ipN, channelDirArray[c] + File.separator
                                + (((c % 2) == 0) ? "SPIMA" : "SPIMB")
                                + "-" + t + ".tif");
                    }
                }

            } else if (exportFormat_
                    == 1) {  // Multiview reconstruction
                throw new SaveTaskException("Should import Micro-Manager datasets "
                        + "directly into Fiji Multiview reconstruction as of April 2015.");
            }

            return null;
        }

        @Override
        public void done() {
            setCursor(null);
            try {
                get();
                setProgress(100);
            } catch (ExecutionException ex) {
                Throwable cause = ex.getCause();
                if (!cause.getMessage().equals("Macro canceled")) {
                    if (cause instanceof SaveTaskException) {
                        MyDialogUtils.showError(cause, "Data Export Error");
                    } else {
                        MyDialogUtils.showError(ex);
                    }
                }
            } catch (InterruptedException ex) {
                MyDialogUtils.showError(ex, "Interrupted while exporting data");
            }
        }
    }
    
   /**
    * for API, perform export like clicking on the button
    */
   public void runExport() {
      exportButton_.doClick();
   }

   /**
    * for API, set the base name field for export
    */
   public void setExportBaseName(String baseName) {
      baseNameField_.setText(baseName);
   }

   /**
    * Since java 1.6 does not seem to have this functionality....
    * @param folder folder to be deleted
    */
   public static void deleteFolder(File folder) {
      File[] files = folder.listFiles();
      if (files != null) { 
         for (File f : files) {
            if (f.isDirectory()) {
               deleteFolder(f);
            } else {
               f.delete();
            }
         }
      }
      folder.delete();
   }

   private void setSaveDestinationDirectory(JTextField rootField) {
      File result = FileDialogs.openDir(null,
              "Please choose a directory root for image data",
              EXPORT_DATA_SET);
      if (result != null) {
         rootField.setText(result.getAbsolutePath());
      }
   }

   public class SaveTaskException extends Exception {

      private static final long serialVersionUID = -8472323699461107823L;
      private Throwable cause;

      public SaveTaskException(String message) {
         super(message);
      }

      public SaveTaskException(Throwable t) {
         super(t.getMessage());
         this.cause = t;
      }

      @Override
      public Throwable getCause() {
         return this.cause;
      }
   }
   
}
