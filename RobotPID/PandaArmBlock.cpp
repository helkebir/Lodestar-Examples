//
// Created by Hamza El-Kebir on 2/20/22.
//

#include "PandaArmBlock.hpp"

namespace ls {
    namespace blocks {
        const ::std::array<::std::string, BlockTraits<PandaArmBlock>::kIns> BlockTraits<PandaArmBlock>::inTypes =
                {demangle(typeid(Eigen::Matrix<double, 7, 1>).name())};

        const ::std::array<::std::string, BlockTraits<PandaArmBlock>::kOuts> BlockTraits<PandaArmBlock>::outTypes =
                {demangle(typeid(PandaState).name())};

        const ::std::array<::std::string, BlockTraits<PandaArmBlock>::kPars> BlockTraits<PandaArmBlock>::parTypes =
                {demangle(typeid(::std::string).name())};

        const ::std::array<::std::string, 0> BlockTraits<PandaArmBlock>::templateTypes =
                {};
    }
}