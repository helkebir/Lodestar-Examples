//
// Created by advcc on 2/16/22.
//

#ifndef LSEXAMPLE_THERMOGRAPHERBLOCK_HPP
#define LSEXAMPLE_THERMOGRAPHERBLOCK_HPP

#include <Lodestar/blocks/Block.hpp>
#include <Eigen/Dense>
#include <string>
#include <array>

#include <libirimager/direct_binding.h>

namespace ls {
    namespace blocks {
        template<typename TScalar = float, int NWidth = 382, int NHeight = 288>
        class ThermographerBlock
                : public Block<
                        ::std::tuple<float>,
                        ::std::tuple<Eigen::Matrix<TScalar, NHeight, NWidth, Eigen::ColMajor>, float>,
                        ::std::tuple<::std::string>
                > {
        public:
            using Base =
            Block<
                    ::std::tuple<float>,
                    ::std::tuple<Eigen::Matrix<TScalar, NHeight, NWidth>, float>,
                    ::std::tuple<::std::string>
            >;

            using Matrix = Eigen::Matrix<TScalar, NHeight, NWidth, Eigen::ColMajor>;
            using MatrixUShort = Eigen::Matrix<unsigned short, NHeight, NWidth, Eigen::RowMajor>;

            ThermographerBlock()
            {
                this->template i<0>().object = 0;
                bindFunction();
            }

            ~ThermographerBlock()
            {
                ::evo_irimager_terminate();
            }

            typename ::std::tuple_element<0, typename Base::Params>::type &
            serial()
            {
                return this->template p<0>();
            }

            const typename ::std::tuple_element<0, typename Base::Params>::type &
            serial() const
            {
                return this->template p<0>();
            }

        protected:
            void bindFunction()
            {
                this->equation = ::std::bind(
                        &ThermographerBlock<TScalar, NWidth, NHeight>::triggerFunction, this,
                        ::std::placeholders::_1);
            }

            void triggerFunction(Base &b)
            {
                static bool init = false;
                static int err;
                static int kWidth = NWidth;
                static int kHeight = NHeight;
                static ::std::array<unsigned short, NWidth * NHeight> thermalData{};
                static MatrixUShort M;

                if (!init) {
                    err = ::evo_irimager_usb_init(b.template p<0>().c_str(), 0, 0);
                    // Add error message if err != 0
                    init = true;
                }

                evo_irimager_get_focusmotor_pos(&b.template o<1>().object);
                b.template o<1>().propagate();
                evo_irimager_set_focusmotor_pos(b.template i<0>().object);

                static Eigen::Map<MatrixUShort> map = Eigen::Map<MatrixUShort>(&thermalData[0]);

                if ((err = ::evo_irimager_get_thermal_image(&kWidth, &kHeight, &thermalData[0])) == 0) {
                    M = map;
                    b.template o<0>().object = M.template cast<TScalar>();
                    b.template o<0>().object = b.template o<0>().object.unaryExpr([](TScalar T) -> TScalar { return T/10.0 - 100.0; });
                    b.template o<0>().propagate();
                } else {
                    // err
                }
            }
        };

        template<typename TScalar, int NWidth, int NHeight>
        class BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>> {
        public:
            static constexpr const BlockType blockType = BlockType::CustomBlock;
            enum {
                directFeedthrough = false
            };

            using type = ThermographerBlock<TScalar, NWidth, NHeight>;
            using Base = typename type::Base;

            enum {
                kIns = Base::kIns,
                kOuts = Base::kOuts,
                kPars = Base::kPars
            };

            static const ::std::array<::std::string, kIns> inTypes;
            static const ::std::array<::std::string, kOuts> outTypes;
            static const ::std::array<::std::string, kPars> parTypes;

            static const ::std::array<::std::string, 3> templateTypes;
        };

        template<typename TScalar, int NWidth, int NHeight>
        const ::std::array<::std::string, BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::kIns> BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::inTypes =
                {"float"};

        template<typename TScalar, int NWidth, int NHeight>
        const ::std::array<::std::string, BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::kOuts> BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::outTypes =
                {demangle(typeid(Eigen::Matrix<TScalar, NHeight, NWidth, Eigen::ColMajor>).name()), "float"};

        template<typename TScalar, int NWidth, int NHeight>
        const ::std::array<::std::string, BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::kPars> BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::parTypes =
                {demangle(typeid(::std::string).name())};

        template<typename TScalar, int NWidth, int NHeight>
        const ::std::array<::std::string, 3> BlockTraits<ThermographerBlock<TScalar, NWidth, NHeight>>::templateTypes =
                {demangle(typeid(TScalar).name()), "int", "int"};
    }
}


#endif //LSEXAMPLE_THERMOGRAPHERBLOCK_HPP
