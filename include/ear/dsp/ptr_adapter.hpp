#pragma once
#include <vector>
#include "../helpers/assert.hpp"

namespace ear {
  namespace dsp {

    /// Adapter from Eigen matrix expressions (and possibly other things) to
    /// float**.
    template <typename PtrT = float *>
    class PtrAdapterT {
     public:
      PtrAdapterT(size_t nchannels) : _ptrs(nchannels) {}

      /// Point each pointer at a column of Eigen Matrix expression \p matrix,
      /// with an offset of \p offset.
      template <typename T>
      void set_eigen(T &&mat, size_t offset = 0) {
        ear_assert((size_t)mat.cols() == _ptrs.size(),
                   "wrong number of channels");
        for (size_t i = 0; i < _ptrs.size(); i++) {
          _ptrs[i] = mat.col(i).data() + offset;
        }
      }

      /// Get a pointer to each channel.
      PtrT *ptrs() { return _ptrs.data(); }

      // Make non-copyable.
      // This points to some external memory, so if this is copied as part of
      // another object it will probably go wrong.
      PtrAdapterT(const PtrAdapterT &) = delete;
      PtrAdapterT &operator=(const PtrAdapterT &) = delete;

     private:
      std::vector<PtrT> _ptrs;
    };

    using PtrAdapter = PtrAdapterT<float *>;
    using PtrAdapterConst = PtrAdapterT<const float *>;
  }  // namespace dsp
}  // namespace ear
