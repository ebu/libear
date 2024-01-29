#pragma once
#include <boost/optional.hpp>
#include <map>
#include <memory>
#include <regex>
#include "../common/point_source_panner.hpp"
#include "../common/screen_edge_lock.hpp"
#include "ear/helpers/output_gains.hpp"
#include "ear/layout.hpp"
#include "ear/metadata.hpp"
#include "ear/warnings.hpp"

namespace ear {

  class GainCalculatorDirectSpeakersImpl {
   public:
    GainCalculatorDirectSpeakersImpl(
        const Layout& layout,
        std::map<std::string, std::string> additionalSubstitutions = {});

    void calculate(const DirectSpeakersTypeMetadata& metadata,
                   OutputGains& direct,
                   const WarningCB& warning_cb = default_warning_cb);

    template <typename T>
    void calculate(const DirectSpeakersTypeMetadata& metadata,
                   std::vector<T>& direct,
                   const WarningCB& warning_cb = default_warning_cb) {
      OutputGainsT<T> direct_wrapped(direct);
      calculate(metadata, direct_wrapped, warning_cb);
    }

   private:
    /** @brief Get the bs.2051 speaker label from an ADM speaker label.
     *
     * This parses URNs, and deals with alternative notations for LFE channels.
     */
    std::string _nominalSpeakerLabel(const std::string& label);
    /** @brief Get the index of the candidate speaker closest to a given
    position.
     *
     * If there are multiple speakers that are considered equally close with
     * respect to a given tolerance, no decision on the closest speaker can be
     * made.
     *
     * @param position  Target position
     * @param candidates Subset of speakers to be considered
     * @param tol  tolerance for defintion of "closest".
     *
     * @returns  Index to the speaker in speakers that is closest to the
     * target position or `None` if no such speaker can be uniquely defined.
     */
    boost::optional<int> _closestChannelIndex(const SpeakerPosition& position,
                                              std::vector<bool> candidates,
                                              double tol);

    /** @brief Determine if type_metadata is an LFE channel, issuing a warning
     * is there's a discrepancy between the speakerLabel and the frequency
     * element.
     */
    bool _isLfeChannel(const DirectSpeakersTypeMetadata& metadata,
                       const WarningCB& warning_cb);

    PolarSpeakerPosition _applyScreenEdgeLock(PolarSpeakerPosition position);
    CartesianSpeakerPosition _applyScreenEdgeLock(
        CartesianSpeakerPosition position);
    SpeakerPosition _applyScreenEdgeLock(const SpeakerPosition& position);
    boost::optional<int> _findChannelWithinBounds(
        const SpeakerPosition& position, bool isLfe, double tol);
    std::vector<std::pair<int, double>> _findCandidates(
        const PolarSpeakerPosition& position, bool isLfe, double tol);
    std::vector<std::pair<int, double>> _findCandidates(
        const CartesianSpeakerPosition& position, bool isLfe, double tol);
    std::vector<std::pair<int, double>> _findCandidates(
        const SpeakerPosition& position, bool isLfe, double tol);

    Layout _layout;
    std::shared_ptr<PointSourcePanner> _pointSourcePanner;
    ScreenEdgeLockHandler _screenEdgeLockHandler;
    int _nChannels;
    std::vector<std::string> _channelNames;
    Eigen::VectorXd _azimuths;
    Eigen::VectorXd _elevations;
    Eigen::VectorXd _distances;
    Eigen::MatrixXd _positions;
    Eigen::Array<bool, Eigen::Dynamic, 1> _isLfe;
    std::map<std::string, std::string> _substitutions;
    const std::regex SPEAKER_URN_REGEX =
        std::regex("^urn:itu:bs:2051:[0-9]+:speaker:(.*)$");
  };

}  // namespace ear
