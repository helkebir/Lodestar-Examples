//
// Created by Hamza El-Kebir on 2/20/22.
//

#ifndef LSEXAMPLE_PANDAARMBLOCK_HPP
#define LSEXAMPLE_PANDAARMBLOCK_HPP


#include <Lodestar/blocks/Block.hpp>
#include <Eigen/Dense>
#include <string>
#include <array>

#include "arm/Panda.hpp"
#include <thread>

namespace ls {
    namespace blocks {
        class PandaArmBlock
                : public Block<
                        ::std::tuple<Eigen::Matrix<double, 7, 1>>,
                        ::std::tuple<PandaState>,
                        ::std::tuple<::std::string>
                > {
        public:
            using Base =
            Block<
                    ::std::tuple<Eigen::Matrix<double, 7, 1>>,
                    ::std::tuple<PandaState>,
                    ::std::tuple<::std::string>
            >;

            using Matrix = Eigen::Matrix<double, 7, 1>;

            PandaArmBlock()
            {
                this->template i<0>().object.setZero();
                bindFunction();
            }

            ~PandaArmBlock()
            {
                panda_.stop();
            }

            typename ::std::tuple_element<0, typename Base::Params>::type &
            ipAddress()
            {
                return this->template p<0>();
            }

            const typename ::std::tuple_element<0, typename Base::Params>::type &
            ipAddress() const
            {
                return this->template p<0>();
            }

        protected:
            Panda panda_;
            
            void bindFunction()
            {
                this->equation = ::std::bind(
                        &PandaArmBlock::triggerFunction, this,
                        ::std::placeholders::_1);
            }

            void triggerFunction(Base &b)
            {
                static bool init = false;
                static bool ready = false;

                if (!init) {
                    panda_ = Panda{ipAddress(), false};
                    
                    init = panda_.initConnection().ok();
                    
                    if (!init)
                        return;

                    Eigen::AngleAxisd aa{M_PI / 2, Eigen::Vector3d{0, -1, 0}};
                    Eigen::AngleAxisd aa2{M_PI, Eigen::Vector3d{1, 0, 0}};
                    aa = aa * aa2;
                    Eigen::Vector3d translation = {200e-3, 0, 0};

                    panda_.setNominalToEndEffectorTransformation(aa, translation);
                    
                    panda_.setGuidingMode();

                    ready = true;

                    ::std::thread([&] () {
                        panda_.robot_->control([&] (const franka::RobotState &rs, franka::Duration duration) -> franka::Torques
                                               {
                                                   // TODO: Add mutex for thread safety

                                                   this->template o<0>().object.robot.parse(rs);
                                                   this->template o<0>().propagate();

                                                   while (!ready) {}

                                                   ::std::array<double, 7> tau_d_array{this->template i<0>().object[0],
                                                                                       this->template i<0>().object[1],
                                                                                       this->template i<0>().object[2],
                                                                                       this->template i<0>().object[3],
                                                                                       this->template i<0>().object[4],
                                                                                       this->template i<0>().object[5],
                                                                                       this->template i<0>().object[6]};
                                                   ready = false;
                                                   return {tau_d_array};
                                               });
//                        panda_.control(f);
                    }).detach();
                }

                ready = true;
                
//                b.template o<0>() = panda_.getState().value();
            }
        };

        template<>
        class BlockTraits<PandaArmBlock> {
        public:
            static constexpr const BlockType blockType = BlockType::CustomBlock;
            enum {
                directFeedthrough = false
            };

            using type = PandaArmBlock;
            using Base = typename type::Base;

            enum {
                kIns = Base::kIns,
                kOuts = Base::kOuts,
                kPars = Base::kPars
            };

            static const ::std::array<::std::string, kIns> inTypes;
            static const ::std::array<::std::string, kOuts> outTypes;
            static const ::std::array<::std::string, kPars> parTypes;

            static const ::std::array<::std::string, 0> templateTypes;
        };
    }
}


#endif //LSEXAMPLE_PANDAARMBLOCK_HPP
