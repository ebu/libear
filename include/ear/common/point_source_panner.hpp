#pragma once
#include <Eigen/Core>
#include <memory>
#include <numeric>
#include <string>
#include "ear/layout.hpp"
#include "ear/metadata.hpp"

namespace ear {

  /** @brief An interface for objects that can calculate gains for some
   * positions, e.g. a triangle of loudspeakers.
   */
  class RegionHandler {
   public:
    RegionHandler(Eigen::VectorXi outputChannels, Eigen::MatrixXd positions);
    virtual boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) const = 0;
    virtual ~RegionHandler() = default;

    boost::optional<Eigen::VectorXd> handleRemap(Eigen::Vector3d position,
                                                 int numberOfChannels) const;
    Eigen::VectorXi outputChannels();

   protected:
    Eigen::VectorXi _outputChannels;
    Eigen::MatrixXd _positions;
  };

  /** @brief Region handler representing a triplet of loudspeakers, implementing
   * VBAP.
   *
   * This is implemented such that if handle(pos) returns array x:
   *
   *  - dot(x, positions) is collinear with pos
   *  - x[i] >= 0 for all i
   *  - norm(x) == 1
   *
   * Note that the positions are *not* normalised, as this is not always
   * desirable.
   */
  class Triplet : public RegionHandler {
   public:
    /** @brief Ctor
     *
     * @param  output_channels The channel numbers of the values returned by
     *   handle.
     * @param  positions  Cartesian positions of the three speakers; index order
     *   is speaker, axis.
     */
    Triplet(Eigen::Vector3i outputChannels, Eigen::Matrix3d positions);

    boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) const override;

   private:
    Eigen::Matrix3d _basis;
  };

  /** @brief Region handler representing n real loudspeakers and a central
   * virtual loudspeaker, whose gain is distributed to the real loudspeakers.
   *
   * Triplet regions are formed between the virtual speaker and pairs of real
   * speakers on the edge of the ngon. Any gain sent to the virtual speaker is
   * multiplied by centre_downmix and summed into the gains for the real
   * loudspeakers,which are then normalised.
   */
  class VirtualNgon : public RegionHandler {
   public:
    /** @brief Ctor
     *
     * @param  output_channels The channel numbers of the values returned by
     *   handle.
     * @param  positions Cartesian positions of the n loudspeakers.
     * @param  centrePosition Cartesian position of the central virtual
     *   loudspeaker
     * @param  centreDownmix Downmix coefficients for distributing gains from
     * the centre virtual loudspeaker to the loudspeakers defined by
     * positions.
     */
    VirtualNgon(Eigen::VectorXi outputChannels, Eigen::MatrixXd positions,
                Eigen::Vector3d centrePosition, Eigen::VectorXd centreDownmix);

    boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) const override;

   private:
    Eigen::Vector3d _centrePosition;
    Eigen::VectorXd _centreDownmix;
    std::vector<std::unique_ptr<RegionHandler>> _regions;
  };

  class QuadRegion : public RegionHandler {
   public:
    QuadRegion(Eigen::VectorXi outputChannels, Eigen::MatrixXd positions);

    boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) const override;

   private:
    Eigen::Matrix3d _calcPolyBasis(Eigen::MatrixXd positions);

    boost::optional<double> _pan(Eigen::Vector3d position,
                                 Eigen::Matrix3d polyBasis) const;

    Eigen::VectorXi _order;
    Eigen::Matrix3d _polyBasisX;
    Eigen::Matrix3d _polyBasisY;
  };

  /** @brief Base class for all PointSourcePanner like classes
   */
  class PointSourcePanner {
   public:
    virtual boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) = 0;
    virtual int numberOfOutputChannels() const = 0;
  };

  /** @brief Wrapper around multiple regions.
   */
  class PolarPointSourcePanner : public PointSourcePanner {
   public:
    /** @brief Ctor
     *
     * @param  regions Regions used to handle a position.
     * @param  numberOfChannels Number of output channels; this is computed
     * from the output channels of the regions if not provided.
     */
    PolarPointSourcePanner(std::vector<std::unique_ptr<RegionHandler>> regions,
                           boost::optional<int> numberOfChannels = boost::none);

    boost::optional<Eigen::VectorXd> handle(Eigen::Vector3d position) override;

    int numberOfOutputChannels() const override;

   private:
    int _numberOfRequiredChannels();

    const std::vector<std::unique_ptr<RegionHandler>> _regions;
    int _numberOfOutputChannels;
  };

  /** @brief Wrapper around a point source panner with an additional downmix.
   */
  class PointSourcePannerDownmix : public PointSourcePanner {
   public:
    /** @brief Ctor
     *
     * @param  psp Inner point source panner.
     * @param  downmix Downmix matrix (mxn) from m inputs to n outputs.
     */
    PointSourcePannerDownmix(std::shared_ptr<PointSourcePanner> psp,
                             Eigen::MatrixXd downmix);
    ~PointSourcePannerDownmix() = default;

    boost::optional<Eigen::VectorXd> handle(Eigen::Vector3d position) override;

    int numberOfOutputChannels() const override;

   private:
    std::shared_ptr<PointSourcePanner> _psp;
    Eigen::MatrixXd _downmix;
  };

  /** @brief Generate extra loudspeaker positions to fill gaps in layers.
   *
   * @param  layout Original layout without the LFE channels
   *
   * @returns
   *   - list of extra channels (layout.Channel).
   *   - downmix matrix to mix the extra channel outputs to the real channels
   */
  std::pair<std::vector<Channel>, Eigen::MatrixXd> extraPosVerticalNominal(
      Layout layout);

  std::shared_ptr<PointSourcePanner> configureFullPolarPanner(
      const Layout& layout);

  class StereoPannerDownmix : public RegionHandler {
   public:
    StereoPannerDownmix(Eigen::VectorXi outputChannels,
                        Eigen::MatrixXd positions);

    boost::optional<Eigen::VectorXd> handle(
        Eigen::Vector3d position) const override;

   private:
    std::shared_ptr<PointSourcePanner> _psp;
  };

  class AllocentricPanner : public PointSourcePanner {
   public:
    AllocentricPanner() = default;
    ~AllocentricPanner() = default;

    boost::optional<Eigen::VectorXd> handle(Eigen::Vector3d position) override;
    int numberOfOutputChannels() const override;
  };

  std::shared_ptr<PointSourcePanner> configureStereoPolarPanner(
      const Layout& layout);

  std::shared_ptr<PointSourcePanner> configureFullPolarPanner(
      const Layout& layout);

  std::shared_ptr<PointSourcePanner> configureAllocentricPanner(
      const Layout& layout);

  std::shared_ptr<PointSourcePanner> configurePolarPanner(const Layout& layout);

}  // namespace ear
