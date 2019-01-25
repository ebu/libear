#include "ear/hoa/gain_calculator_hoa.hpp"
#include <map>
#include "ear/common/helpers/eigen_helpers.hpp"
#include "ear/common/point_source_panner.hpp"
#include "ear/hoa/hoa.hpp"

namespace ear {
  GainCalculatorHOAImpl::GainCalculatorHOAImpl(const Layout &layout) {
    std::shared_ptr<PointSourcePanner> panner =
        configurePolarPanner(layout.withoutLfe());
    is_lfe = copy_vector<decltype(is_lfe)>(layout.isLfe());
    points = hoa::load_points();
    G_virt = hoa::calc_G_virt(
        points, [panner](Eigen::Vector3d pos) { return panner->handle(pos); });
  }

  const std::map<std::string, hoa::norm_f_t &> ADM_norm_types{
      {"N3D", hoa::norm_N3D},
      {"SN3D", hoa::norm_SN3D},
      {"FuMa", hoa::norm_FuMa},
  };

  void GainCalculatorHOAImpl::calculate(const HOATypeMetadata &metadata,
                                        OutputGainMat &direct,
                                        const WarningCB &warning_cb) {
    if (metadata.orders.size() != metadata.degrees.size())
      throw invalid_argument("orders and degrees must be the same size");

    for (size_t i = 0; i < metadata.orders.size(); i++) {
      if (metadata.orders[i] < 0)
        throw invalid_argument("orders must not be negative");
      if (abs(metadata.degrees[i]) > metadata.orders[i])
        throw invalid_argument(
            "magnitude of degree must not be greater than order");
    }

    auto norm_it = ADM_norm_types.find(metadata.normalization);
    if (norm_it == ADM_norm_types.end())
      throw adm_error("unknown normalization type: '" + metadata.normalization +
                      "'");

    if (metadata.screenRef)
      warning_cb({Warning::Code::HOA_SCREENREF_NOT_IMPLEMENTED,
                  "screenRef for HOA is not implemented; ignoring"});

    if (metadata.nfcRefDist != 0.0)
      warning_cb({Warning::Code::HOA_NFCREFDIST_NOT_IMPLEMENTED,
                  "nfcRefDist is not implemented; ignoring"});

    hoa::norm_f_t &norm = norm_it->second;

    Eigen::Map<const Eigen::VectorXi> n(metadata.orders.data(),
                                        metadata.orders.size());
    Eigen::Map<const Eigen::VectorXi> m(metadata.degrees.data(),
                                        metadata.degrees.size());

    Eigen::MatrixXd Y_virt = hoa::calc_Y_virt(points, n, m, hoa::norm_N3D);
    Eigen::MatrixXd D_virt = Y_virt.transpose() / points.rows();

    Eigen::MatrixXd D = G_virt * D_virt;

    hoa::normalize_decode_matrix(D, Y_virt);

    D *= hoa::normalisation_conversion(n, m, hoa::norm_N3D, norm).asDiagonal();

    Eigen::MatrixXd D_full = Eigen::MatrixXd::Zero(is_lfe.size(), D.cols());
    for (Eigen::Index in_i = 0; in_i < D.cols(); in_i++) {
      mask_write(D_full.col(in_i), !is_lfe, D.col(in_i));
    }

    direct.write_mat(D_full);
  }
};  // namespace ear
