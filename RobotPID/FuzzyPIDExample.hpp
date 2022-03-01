//
// Created by Hamza El-Kebir on 2/25/22.
//

#ifndef LSEXAMPLE_FUZZYPIDEXAMPLE_HPP
#define LSEXAMPLE_FUZZYPIDEXAMPLE_HPP

#include <Lodestar/blocks/std/GainBlock.hpp>
#include <Lodestar/blocks/std/SumBlock.hpp>
#include <Lodestar/blocks/std/DelayBlock.hpp>
#include <Lodestar/blocks/std/FunctionBlock.hpp>
#include <Lodestar/blocks/BlockUtilities.hpp>
#include <Lodestar/blocks/aux/Executor.hpp>

#include <Eigen/Dense>
#include <chrono>

#include "fuzzyPID.h"

using namespace ls::blocks::std;
using namespace ls::blocks;

class FuzzyPIDExample {
public:
    static void example()
    {
        using TType = float;
        using TScalar = float;

        TScalar T = 1e-3;
        TScalar K, Kp, Kd, Ki, KuPD, KuI;

        GainBlock<TType, TScalar> gainK{K}, gainKp{Kp}, gainKd{Kd}, gainKi{Ki}, gainKuPD{KuPD}, gainKuI{KuI}, gainTinv1{1.f/T}, gainTinv2{1.f/T};
        DelayBlock<TType> delay1, delay2, delay3, delay4, delay5;
        SumBlock<TType, 2> sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum;

        sum1.setOperators(decltype(sum1)::Plus, decltype(sum1)::Minus);
        sum2.setOperators(decltype(sum2)::Plus, decltype(sum2)::Minus);
        sum5.setOperators(decltype(sum5)::Plus, decltype(sum5)::Minus);

        auto &sp = sum1.i<0>();
        auto &e = sum1.o<0>();
        auto &d = gainTinv2.o<0>();
        auto &r = gainTinv1.o<0>();
        auto &duPD = fuzzyPDSum.o<0>();
        auto &duI = sum4.o<0>();
        auto &uPD = sum5.o<0>();
        auto &uI = sum6.o<0>();
        auto &uPID = sum7.o<0>();

        connect(e, delay1.i<0>());
        connect(delay1.o<0>(), gainKi.i<0>());
        connect(gainKi.o<0>(), sum4.i<0>());

        connect(e, delay2.i<0>());
        connect(delay2.o<0>(), sum3.i<0>());
        connect(e, sum3.i<1>());
        connect(sum3.o<0>(), gainTinv2.i<0>());
        connect(d, gainKp.i<0>());

        connect(e, sum2.i<0>());
        connect(e, delay3.i<0>());
        connect(delay3.o<0>(), sum2.i<1>());
        connect(sum2.o<0>(), gainTinv1.i<0>());

        connect(r, gainK.i<0>());
        connect(r, gainKd.i<0>());
        connect(gainK.o<0>(), sum4.i<1>());

        connect(gainKp.o<0>(), fuzzyPDSum.i<0>());
        connect(gainKd.o<0>(), fuzzyPDSum.i<1>());
        connect(duPD, gainKuPD.i<0>());
        connect(gainKuPD.o<0>(), sum5.i<0>());
        connect(uPD, delay4.i<0>());
        connect(delay4.o<0>(), sum5.i<1>());

        connect(sum4.o<0>(), gainKuI.i<0>());
        connect(gainKuI.o<0>(), sum6.i<0>());
        connect(uI, delay5.i<0>());
        connect(delay5.o<0>(), sum6.i<1>());

        connect(uPD, sum7.i<0>());
        connect(uI, sum7.i<1>());

        BlockPack bp{gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI, gainTinv1, gainTinv2, delay1, delay2, delay3, delay4, delay5, sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum};

        aux::Executor ex{bp};
        ex.resolveExecutionOrder();



        for (int i = 0; i < 1000; i++) {
            ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
            ex.trigger();
            ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
            ::std::cout << "0 " << ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count() << ::std::endl;
        }
    }

    static void exampleSL()
    {
        fuzzyPIDModelClass fPID;
        fPID.initialize();
        fPID.fuzzyPID_U.Input = 0;
        fPID.fuzzyPID_U.Input1 = 0;

        for (int i = 0; i < 1000; i++) {
            ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
            fPID.step();
            ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
            ::std::cout << "1 " << ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count() << ::std::endl;
        }
    }

    static void example7D()
    {
        using TType = float;
        using TScalar = float;

        TScalar T = 1e-3;
        TScalar K, Kp, Kd, Ki, KuPD, KuI;

        GainBlock<TType, TScalar> gainK{K}, gainKp{Kp}, gainKd{Kd}, gainKi{Ki}, gainKuPD{KuPD}, gainKuI{KuI}, gainTinv1{1.f/T}, gainTinv2{1.f/T};
        DelayBlock<TType> delay1, delay2, delay3, delay4, delay5;
        SumBlock<TType, 2> sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum;

        sum1.setOperators(decltype(sum1)::Plus, decltype(sum1)::Minus);
        sum2.setOperators(decltype(sum2)::Plus, decltype(sum2)::Minus);
        sum5.setOperators(decltype(sum5)::Plus, decltype(sum5)::Minus);

        auto &sp = sum1.i<0>();
        auto &e = sum1.o<0>();
        auto &d = gainTinv2.o<0>();
        auto &r = gainTinv1.o<0>();
        auto &duPD = fuzzyPDSum.o<0>();
        auto &duI = sum4.o<0>();
        auto &uPD = sum5.o<0>();
        auto &uI = sum6.o<0>();
        auto &uPID = sum7.o<0>();

        connect(e, delay1.i<0>());
        connect(delay1.o<0>(), gainKi.i<0>());
        connect(gainKi.o<0>(), sum4.i<0>());

        connect(e, delay2.i<0>());
        connect(delay2.o<0>(), sum3.i<0>());
        connect(e, sum3.i<1>());
        connect(sum3.o<0>(), gainTinv2.i<0>());
        connect(d, gainKp.i<0>());

        connect(e, sum2.i<0>());
        connect(e, delay3.i<0>());
        connect(delay3.o<0>(), sum2.i<1>());
        connect(sum2.o<0>(), gainTinv1.i<0>());

        connect(r, gainK.i<0>());
        connect(r, gainKd.i<0>());
        connect(gainK.o<0>(), sum4.i<1>());

        connect(gainKp.o<0>(), fuzzyPDSum.i<0>());
        connect(gainKd.o<0>(), fuzzyPDSum.i<1>());
        connect(duPD, gainKuPD.i<0>());
        connect(gainKuPD.o<0>(), sum5.i<0>());
        connect(uPD, delay4.i<0>());
        connect(delay4.o<0>(), sum5.i<1>());

        connect(sum4.o<0>(), gainKuI.i<0>());
        connect(gainKuI.o<0>(), sum6.i<0>());
        connect(uI, delay5.i<0>());
        connect(delay5.o<0>(), sum6.i<1>());

        connect(uPD, sum7.i<0>());
        connect(uI, sum7.i<1>());
    }
};


#endif //LSEXAMPLE_FUZZYPIDEXAMPLE_HPP
