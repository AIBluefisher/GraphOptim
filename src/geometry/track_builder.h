#ifndef GEOMETRY_TRACK_BUILDER_H_
#define GEOMETRY_TRACK_BUILDER_H_

#include <vector>
#include <unordered_map>

#include "utils/types.h"

namespace gopt {

// Track class stores all observations of a 3D point.
struct TrackElement {
  TrackElement()
    : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) {}
  
  TrackElement(const image_t image_id, const point2D_t point2D_idx)
    : image_id(image_id), point2D_idx(point2D_idx) {}
  
  // The image in which the track element is observed.
  image_t image_id;
  
  // The point in the image that the track element is observed.
  point2D_t point2D_idx;
};

typedef std::vector<TrackElement> TrackElements;

// Build tracks from feature correspondences across multiple images. Tracks are
// created with the connected components algorithm and have a maximum allowable
// size. If there are multiple features from one image in a track, we do not do
// any intelligent selection and just arbitrarily choose a feature to drop so
// that the tracks are consistent.
class TrackBuilder {
 public:
  TrackBuilder(const size_t min_track_length, const size_t max_track_length);

  // Build tracks for a given series of track elements.
  void Build(const std::vector<TrackElement>& track_elements,
             const std::vector<std::pair<track_t, track_t>>& pair_ids);

  // Remove bad tracks that are too short or have ids collision.
  bool Filter();

  // Return the number of connected set in the Union Find structure.
  size_t NumTracks() const;

  // Extract consistent tracks.
  const std::unordered_map<track_t, TrackElements>& GetConsistentTracks() const;
  std::unordered_map<track_t, TrackElements>& GetConsistentTracks();

  double MeanTrackLength() const;

 private:
  const size_t min_track_length_;
  const size_t max_track_length_;

  std::unordered_map<track_t, TrackElements> consistent_tracks_;
};

}  // namespace gopt

#endif  // GEOMETRY_TRACK_BUILDER_H_
