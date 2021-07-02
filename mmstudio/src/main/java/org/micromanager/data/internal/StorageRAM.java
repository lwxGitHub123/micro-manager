///////////////////////////////////////////////////////////////////////////////
//PROJECT:       Micro-Manager
//SUBSYSTEM:     Data API implementation
//-----------------------------------------------------------------------------
//
// AUTHOR:       Chris Weisiger, 2015
//
// COPYRIGHT:    University of California, San Francisco, 2015
//
// LICENSE:      This file is distributed under the BSD license.
//               License text is included with the source distribution.
//
//               This file is distributed in the hope that it will be useful,
//               but WITHOUT ANY WARRANTY; without even the implied warranty
//               of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//               IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//               CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//               INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

package org.micromanager.data.internal;

import com.google.common.eventbus.Subscribe;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.micromanager.data.Coords;
import org.micromanager.data.DataProviderHasNewSummaryMetadataEvent;
import org.micromanager.data.Datastore;
import org.micromanager.data.Image;
import org.micromanager.data.RewritableStorage;
import org.micromanager.data.SummaryMetadata;


/**
 * Simple RAM-based storage for Datastores. Methods that interact with the
 * HashMap that is our image storage are synchronized.
 * 
 * TODO: coordsToImage_ can be set to null in the close function
 * if any of the member functions are called after "close", a null pointer exception
 * will follow.  We can either check for null whenever coordsToImage is used,
 * or make sure that no member is ever called after the close function 
 * (which may be very difficult to guarantee).
 */
public final class StorageRAM implements RewritableStorage {
   private HashMap<Coords, Image> coordsToImage_;
   private Coords maxIndex_;
   private SummaryMetadata summaryMetadata_;
   private final Set<String> axesInUse_;
   private Image anyImage_;

   public StorageRAM(Datastore store) {
      coordsToImage_ = new HashMap<>();
      maxIndex_ = new DefaultCoords.Builder().build();
      axesInUse_ = new TreeSet<>();
      summaryMetadata_ = (new DefaultSummaryMetadata.Builder()).build();
      // It is imperative that we be notified of new images before anyone who
      // wants to retrieve the images from the store is notified.
      ((DefaultDatastore) store).registerForEvents(this, 0);
   }

   /**
    * Add a new image to our storage, and update maxIndex_.
    */
   @Override
   public synchronized void putImage(Image image) {
      Image imageExisting = getAnyImage();
      if (imageExisting != null) {
         ImageSizeChecker.checkImageSizes(image, imageExisting);
      }
      Coords coords = image.getCoords();
      coordsToImage_.put(coords, image);
      for (String axis : coords.getAxes()) {
         axesInUse_.add(axis);
         if (maxIndex_.getIndex(axis) < coords.getIndex(axis)) {
            // Either this image is further along on this axis, or we have
            // no index for this axis yet.
            maxIndex_ = maxIndex_.copyBuilder()
                  .index(axis, coords.getIndex(axis))
                  .build();
         }
      }
   }

   @Override
   public void freeze() {
      // The Datastore handles making certain writes don't occur, and we don't
      // do anything special to "finish" storing data, so this is a no-op.
   }

   @Override
   public synchronized Image getImage(Coords coords) {
      if (coordsToImage_ != null && coordsToImage_.containsKey(coords)) {
         return coordsToImage_.get(coords);
      }
      return null;
   }

   @Override
   public Image getAnyImage() {
      if (anyImage_ != null) {
         return anyImage_;
      }
      synchronized (this) {
         if (coordsToImage_ != null && coordsToImage_.size() > 0) {
            Coords coords = new ArrayList<>(coordsToImage_.keySet()).get(0);
            anyImage_ = coordsToImage_.get(coords);
            return anyImage_;
         }
      }
      return null;
   }

   @Override
   public synchronized List<Image> getImagesMatching(Coords coords) {
      if (coordsToImage_ == null) {
         return null;
      }
      List<Image> results = new ArrayList<>();
      for (Image image : coordsToImage_.values()) {
         // TODO figure out why subSpace was used and fix problems by not doing it
         //  if (image.getCoords().isSubspaceCoordsOf(coords)) {
         if (image.getCoords().equals(coords)) {
            results.add(image);
         }
      }
      return results;
   }

   public synchronized List<Image> getImagesIgnoringAxes(Coords coords, String... ignoreTheseAxes)
           throws IOException {
      if (coordsToImage_ == null) {
         return null;
      }
      // Optimization: traversing large HashMaps is costly, so avoid that when there is no need
      // without this, there is noticeable slowdown for one axis data > ~10,000 images.
      // An alternative optimization approach is to keep collections of coords
      // for all possible ignoredAxes.  There is more upfront work involved but may be
      // needed for fast multi-camera imaging.
      List<Image> result = new ArrayList<>();
      boolean haveIgnoredAxes = false;
      for (String axis : ignoreTheseAxes) {
         if (axesInUse_.contains(axis)) {
            haveIgnoredAxes = true;
            continue;
         }
      }
      if (!haveIgnoredAxes) {
         result.add(coordsToImage_.get(coords));
      } else {  // could not optimize, traverse our HashMap
         for (Image image : coordsToImage_.values()) {
            Coords imCoord = image.getCoords().copyRemovingAxes(ignoreTheseAxes);
            if (imCoord.equals(coords)) {
               result.add(image);
            }
         }
      }
      return result;
   }

   @Override
   public synchronized Iterable<Coords> getUnorderedImageCoords() {
      return coordsToImage_.keySet();
   }

   @Override
   public boolean hasImage(Coords coords) {
      return coordsToImage_.containsKey(coords);
   }

   @Override
   public int getMaxIndex(String axis) {
      return maxIndex_.getIndex(axis);
   }

   // TODO: check that metadata axis are a reliable source of information
   @Override
   public List<String> getAxes() {
      return summaryMetadata_.getOrderedAxes();
   }

   @Override
   public Coords getMaxIndices() {
      return maxIndex_;
   }

   @Override
   public SummaryMetadata getSummaryMetadata() {
      return summaryMetadata_;
   }

   @Subscribe
   public void onNewSummary(DataProviderHasNewSummaryMetadataEvent event) {
      summaryMetadata_ = event.getSummaryMetadata();
   }

   @Override
   public int getNumImages() {
      return coordsToImage_.size();
   }

   @Override
   public synchronized void deleteImage(Coords coords) throws IllegalArgumentException {
      if (!coordsToImage_.containsKey(coords)) {
         throw new IllegalArgumentException("Storage does not contain image at " + coords);
      }
      coordsToImage_.remove(coords);
   }

   @Override
   public void close() {
      coordsToImage_ = null;
   }
}
