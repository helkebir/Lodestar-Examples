//
// Created by Hamza El-Kebir on 2/26/22.
//

#include "PandaArmVelBlock.hpp"

namespace ls {
    namespace blocks {
        const ::std::array<::std::string, BlockTraits<PandaArmVelBlock>::kIns> BlockTraits<PandaArmVelBlock>::inTypes =
                {demangle(typeid(Eigen::Matrix<double, 3, 1>).name())};

        const ::std::array<::std::string, BlockTraits<PandaArmVelBlock>::kOuts> BlockTraits<PandaArmVelBlock>::outTypes =
                {demangle(typeid(PandaState).name())};

        const ::std::array<::std::string, BlockTraits<PandaArmVelBlock>::kPars> BlockTraits<PandaArmVelBlock>::parTypes =
                {demangle(typeid(::std::string).name())};

        const ::std::array<::std::string, 0> BlockTraits<PandaArmVelBlock>::templateTypes =
                {};
    }
}