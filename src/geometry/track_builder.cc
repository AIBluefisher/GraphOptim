#include "geometry/track_builder.h"

#include <fstream>
#include <glog/logging.h>

#include "graph/union_find.h"

namespace gopt {

TrackBuilder::TrackBuilder(const size_t min_track_length,
                           const size_t max_track_length)
  : min_track_length_(min_track_length),
    max_track_length_(max_track_length) {}

void TrackBuilder::Build(
    const std::vector<TrackElement>& track_elements,
    const std::vector<std::pair<track_t, track_t>>& pair_ids) {
  LOG(INFO) << "Building tracks...";
  graph::UnionFind uf(track_elements.size(), max_track_length_);

  for (size_t i = 0; i < pair_ids.size(); i++) {
    uf.Union(pair_ids[i].first, pair_ids[i].second);
  }

  for (size_t i = 0; i < track_elements.size(); i++) {
    const size_t parent_id = uf.FindRoot(i);
    consistent_tracks_[parent_id].emplace_back(track_elements[i]);
  }

  LOG(INFO) << "\tTotal tracks: " << consistent_tracks_.size();
  LOG(INFO) << "\tMean track length: " << MeanTrackLength();
}

bool TrackBuilder::Filter() {
  LOG(INFO) << "Filtering tracks...";
  size_t num_small_tracks = 0;
  size_t num_inconsistent_track_elements = 0;
  
  std::unordered_set<track_t> removed_track_id;

  for (auto track_it = consistent_tracks_.begin();
       track_it != consistent_tracks_.end(); track_it++) {
    // If track.length < min_track_length, we should discard this track.
    if (track_it->second.size() < min_track_length_) {
      removed_track_id.insert(track_it->first);
      num_small_tracks++;
    }

    const std::vector<TrackElement>& candidate_track = track_it->second;
    std::vector<TrackElement> consistent_track;
    consistent_track.reserve(track_it->second.size());

    std::unordered_set<image_t> image_ids;
    for (size_t i = 0; i < candidate_track.size(); i++) {
      const TrackElement& track_element = candidate_track[i];
      // Do not add the track_element if the track already contains a
      // track element from the same image.
      if (image_ids.count(track_element.image_id) != 0) {
        num_inconsistent_track_elements++;
        continue;
      }

      image_ids.insert(track_element.image_id);
      consistent_track.emplace_back(track_element);
    }

    if (consistent_track.size() < min_track_length_) {
      removed_track_id.insert(track_it->first);
      num_small_tracks++;
    }

    if (candidate_track.size() != consistent_track.size()) {
      consistent_tracks_[track_it->first].clear();
      consistent_tracks_[track_it->first].assign(consistent_track.begin(),
                                                 consistent_track.end());
    }
  }

  for (const track_t track_id : removed_track_id) {
    consistent_tracks_.erase(track_id);
  }

  LOG(INFO) << "\t" << num_small_tracks << " small tracks are removed";
  LOG(INFO) << "\t" << num_inconsistent_track_elements
            << " inconsistent track element are removed.";
  LOG(INFO) << "\tTotal tracks: " << consistent_tracks_.size();
  LOG(INFO) << "\tMean track length: " << MeanTrackLength();
  return true;
}

size_t TrackBuilder::NumTracks() const { return consistent_tracks_.size(); }

const std::unordered_map<track_t, TrackElements>&
TrackBuilder::GetConsistentTracks() const {
  return consistent_tracks_;
}

std::unordered_map<track_t, TrackElements>&
TrackBuilder::GetConsistentTracks() {
  return consistent_tracks_;
}

double TrackBuilder::MeanTrackLength() const {
  double mean_track_length = 0;
  for (const auto& iter : consistent_tracks_) {
    mean_track_length += static_cast<double>(iter.second.size());
  }
  mean_track_length /= static_cast<double>(consistent_tracks_.size());
  return mean_track_length;
}

bool TrackBuilder::WriteToFile(const std::string& filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    LOG(ERROR) << "file " << filename << " cannot be opened!";
    return false;
  }

  for (const auto& iter : consistent_tracks_) {
    ofs << iter.first << " " << iter.second.size() << std::endl;
    for (const auto& element : iter.second) {
      ofs << element.image_id - 1 << " " << element.point2D_idx << " ";
    }
    ofs << std::endl;
  }
  ofs.close();

  return true;
}

}  // namespace gopt
