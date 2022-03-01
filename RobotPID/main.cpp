#define EIGEN_STACK_ALLOCATION_LIMIT 512000

#include <Lodestar/blocks/std/ConstantBlock.hpp>
#include <Lodestar/blocks/std/SumBlock.hpp>
#include <Lodestar/blocks/std/GainBlock.hpp>
#include <Lodestar/blocks/std/MinMaxIdxBlock.hpp>
#include <Lodestar/blocks/std/UnaryAbsBlock.hpp>
#include <Lodestar/blocks/std/UnaryMeanBlock.hpp>
#include <Lodestar/blocks/std/BufferBlock.hpp>
#include <Lodestar/blocks/std/SignumBlock.hpp>
#include <Lodestar/blocks/std/SaturationBlock.hpp>
#include <Lodestar/blocks/std/DeadzoneBlock.hpp>
#include <Lodestar/blocks/std/FunctionBlock.hpp>
#include <Lodestar/blocks/std/SwitchBlock.hpp>
#include <Lodestar/blocks/std/SimplePIDBlock.hpp>
#include <Lodestar/blocks/std/DemuxBlock.hpp>
#include <Lodestar/blocks/std/MuxBlock.hpp>
#include <Lodestar/blocks/std/DelayBlock.hpp>

#include <Lodestar/blocks/BlockUtilities.hpp>
#include <Lodestar/blocks/aux/Executor.hpp>

#include <iostream>

#include "ThermographerBlock.hpp"
#include "PandaArmBlock.hpp"
#include "PandaArmVelBlock.hpp"

#include <Eigen/Dense>

#include <matplot/matplot.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <chrono>

#include "FuzzyPIDExample.hpp"

using namespace ls::blocks;
using namespace ls::blocks::std;

void torqueControl()
{
    const int kJointNo = 2;
    PandaArmBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 7>(PandaState)> jointStateFunc = [](
            PandaState ps) -> Eigen::Vector<double, 7> {
        return ps.robot.jointPosition;
    };

    auto jointStateBlk = FunctionBlock<>::make(jointStateFunc);

    DemuxBlock<Eigen::Vector<double, 7>> demuxBlk;

    demuxBlk.o<6>();

    SimplePIDBlock<double> pidBlk;
    pidBlk.pGain() = 100000;
    pidBlk.iGain() = 10000;
    pidBlk.dGain() = 0;
    pidBlk.samplingPeriod() = 1e-3;

    SumBlock<double, 2> sumBlk;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);

    ConstantBlock<double> constBlk{0.65}, zeroBlk{0};

    SaturationBlock<double> satBlk;
    satBlk.lower() = -25;
    satBlk.upper() = 25;

    MuxBlock<Eigen::Vector<double, 7>> muxBlk;

    connect(pandaBlk.o<0>(), jointStateBlk.i<0>());
    connect(jointStateBlk.o<0>(), demuxBlk.i<0>());
    connect(constBlk.o<0>(), sumBlk.i<0>());
    connect(demuxBlk.o<kJointNo>(), sumBlk.i<1>());
    connect(sumBlk.o<0>(), pidBlk.i<0>());
    connect(pidBlk.o<0>(), satBlk.i<0>());

    connect(satBlk.o<0>(), muxBlk.i<kJointNo>());
    connect(zeroBlk.o<0>(), muxBlk.i<0>());
    connect(zeroBlk.o<0>(), muxBlk.i<1>());
//    connect(zeroBlk.o<0>(), muxBlk.i<2>());
    connect(zeroBlk.o<0>(), muxBlk.i<3>());
    connect(zeroBlk.o<0>(), muxBlk.i<4>());
    connect(zeroBlk.o<0>(), muxBlk.i<5>());
    connect(zeroBlk.o<0>(), muxBlk.i<6>());

    connect(muxBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, jointStateBlk, demuxBlk, pidBlk, constBlk, sumBlk, satBlk, zeroBlk};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    for (int i = 0; i < 10000; i++) {
//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

        begin = ::std::chrono::steady_clock::now();
        ex.trigger();

        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
        ::std::cout << "Error: " << sumBlk.o<0>() << ::std::endl;
        ::std::cout << "Control: " << satBlk.o<0>() << ::std::endl;
        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
        end = ::std::chrono::steady_clock::now();

        sleepTime = 500000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
        if (sleepTime > 0) {
            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
        }

    }
}

void torqueControlFull()
{
    PandaArmBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 7>(PandaState)> jointStateFunc = [](
            PandaState ps) -> Eigen::Vector<double, 7> {
        return ps.robot.jointPosition;
    };

    auto jointStateBlk = FunctionBlock<>::make(jointStateFunc);

    DemuxBlock<Eigen::Vector<double, 7>> demuxBlk;

    SimplePIDBlock<Eigen::Vector<double, 7>, ls::blocks::std::SimplePIDBlockOperator::Elementwise> pidBlk;
    pidBlk.pGain() << 10, 25, 10, 50, 10, 25, 10;
    pidBlk.iGain() << 0.2, 0.4, 0.4, 0.6, 0.4, 0.6, 0.4;
    pidBlk.dGain() << 0.1, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1;
    pidBlk.dGain() *= 0.1;
    pidBlk.samplingPeriod() = 1e-4;

    SumBlock<Eigen::Vector<double, 7>, 2> sumBlk, sumBlk2;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumBlk.zero().setZero();
    sumBlk2.zero().setZero();

    ConstantBlock<Eigen::Vector<double, 7>>
            constBlk{{0.2 * M_PI, 0.1 * M_PI, 0, -0.1 * M_PI, 0, M_PI / 2, -0.75 * M_PI}},
//    constBlk{{0,0,0,0,0,0,0}},
    biasBlk{{0, 0, 0, 0, 0, 0, 0}};
//    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};

    SaturationBlock<Eigen::Vector<double, 7>> satBlk;
    satBlk.lower() = -15;
    satBlk.upper() = 15;

    MuxBlock<Eigen::Vector<double, 7>> muxBlk;

    connect(pandaBlk.o<0>(), jointStateBlk.i<0>());
    connect(constBlk.o<0>(), sumBlk.i<0>());
    connect(jointStateBlk.o<0>(), sumBlk.i<1>());
    connect(sumBlk.o<0>(), pidBlk.i<0>());
//    connect(pidBlk.o<0>(), satBlk.i<0>());
    connect(biasBlk.o<0>(), sumBlk2.i<0>());
    connect(pidBlk.o<0>(), sumBlk2.i<1>());
    connect(sumBlk2.o<0>(), satBlk.i<0>());

    connect(satBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, jointStateBlk, demuxBlk, pidBlk, constBlk, sumBlk, satBlk, sumBlk2};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point begin0 = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    for (int i = 0; i < 100000000; i++) {
        if (i < 150) {
            pandaBlk.trigger();
            continue;
        } else if (i == 150) {
            begin0 = ::std::chrono::steady_clock::now();
        }
//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

        begin = ::std::chrono::steady_clock::now();
        ex.trigger();

//        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "PID: " << sumPID.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
//        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
        if (i%1000 == 0) {
            end = ::std::chrono::steady_clock::now();

            ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin0).count() << " "
                        << sumBlk.o<0>().object[0] << " "
                        << sumBlk.o<0>().object[1] << " "
                        << sumBlk.o<0>().object[2] << " "
                        << sumBlk.o<0>().object[3] << " "
                        << sumBlk.o<0>().object[4] << " "
                        << sumBlk.o<0>().object[5] << " "
                        << sumBlk.o<0>().object[6] << ::std::endl;
        }

        end = ::std::chrono::steady_clock::now();
        sleepTime = 10000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
        if (sleepTime > 0) {
            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
        }

    }
}

void torqueControlFullAntiwindup()
{
    PandaArmBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 7>(PandaState)> jointStateFunc = [](
            PandaState ps) -> Eigen::Vector<double, 7> {
        return ps.robot.jointPosition;
    };

    auto jointStateBlk = FunctionBlock<>::make(jointStateFunc);

    GainBlock<Eigen::Vector<double, 7>, Eigen::Matrix<double, 7, 7>> Kp;
    GainBlock<Eigen::Vector<double, 7>, Eigen::Matrix<double, 7, 7>> Ki;
    GainBlock<Eigen::Vector<double, 7>, Eigen::Matrix<double, 7, 7>> Kd;
    GainBlock<Eigen::Vector<double, 7>, double> periodInv{1e4};
    GainBlock<Eigen::Vector<double, 7>, double> period{1e-4};

    DelayBlock<Eigen::Vector<double, 7>, 1> delayDiff, delayInt;

    SaturationBlock<Eigen::Vector<double, 7>> satInt;

    satInt.lower() = -150;
    satInt.upper() = 150;

    Kp.gain().diagonal() << 10, 25, 10, 50, 10, 25, 10;
    Ki.gain().diagonal() << 0.2, 0.4, 0.4, 0.6, 0.4, 0.6, 0.4;
    Kd.gain().diagonal() << 0.1, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1;
    Kd.gain() *= 0.1;

    SumBlock<Eigen::Vector<double, 7>, 2> sumBlk, sumBlk2, sumDiff, sumInt;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumDiff.setOperators(decltype(sumDiff)::Plus, decltype(sumDiff)::Minus);
    sumBlk.zero().setZero();
    sumBlk2.zero().setZero();
    sumDiff.zero().setZero();
    sumInt.zero().setZero();

    SumBlock<Eigen::Vector<double, 7>, 3> sumPID;
    sumPID.zero().setZero();

    ConstantBlock<Eigen::Vector<double, 7>>
            constBlk{{0.2 * M_PI, 0.1 * M_PI, 0, -0.1 * M_PI, 0, M_PI / 2, -0.75 * M_PI}},
//    constBlk{{0,0,0,0,0,0,0}},
    biasBlk{{0, 0, 0, 0, 0, 0, 0}};
//    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};

    SaturationBlock<Eigen::Vector<double, 7>> satBlk;
    satBlk.lower() = -15;
    satBlk.upper() = 15;

    // Proportional
    connect(sumBlk.o<0>(), Kp.i<0>());
    connect(Kp.o<0>(), sumPID.i<0>());

    // Integral
    connect(sumInt.o<0>(), satInt.i<0>());
    connect(satInt.o<0>(), delayInt.i<0>());
    connect(delayInt.o<0>(), sumInt.i<0>());
    connect(sumBlk.o<0>(), period.i<0>());
    connect(period.o<0>(), sumInt.i<1>());
    connect(satInt.o<0>(), Ki.i<0>());
    connect(Ki.o<0>(), sumPID.i<1>());

    // Differential
    connect(sumBlk.o<0>(), sumDiff.i<0>());
    connect(sumBlk.o<0>(), delayDiff.i<0>());
    connect(delayDiff.o<0>(), sumDiff.i<1>());
    connect(sumDiff.o<0>(), periodInv.i<0>());
    connect(periodInv.o<0>(), Kd.i<0>());
    connect(Kd.o<0>(), sumPID.i<2>());

    connect(sumPID.o<0>(), satBlk.i<0>());

    connect(pandaBlk.o<0>(), jointStateBlk.i<0>());
    connect(constBlk.o<0>(), sumBlk.i<0>());
    connect(jointStateBlk.o<0>(), sumBlk.i<1>());

    connect(satBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, jointStateBlk, constBlk, sumBlk, satBlk, sumBlk2, sumDiff, sumInt, sumPID, satInt, Kp, Ki,
                 Kd, period, periodInv, delayInt, delayDiff};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point begin0 = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    for (int i = 0; i < 100000000; i++) {
        if (i < 150) {
            pandaBlk.trigger();
            continue;
        } else if (i == 150) {
            begin0 = ::std::chrono::steady_clock::now();
        }
//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

        begin = ::std::chrono::steady_clock::now();
        ex.trigger();

//        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "PID: " << sumPID.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
//        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
//        if (i%100 == 0) {
//            end = ::std::chrono::steady_clock::now();
//
//            ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin0).count() << " "
//                        << sumBlk.o<0>().object[0] << " "
//                        << sumBlk.o<0>().object[1] << " "
//                        << sumBlk.o<0>().object[2] << " "
//                        << sumBlk.o<0>().object[3] << " "
//                        << sumBlk.o<0>().object[4] << " "
//                        << sumBlk.o<0>().object[5] << " "
//                        << sumBlk.o<0>().object[6] << ::std::endl;
//        }

        end = ::std::chrono::steady_clock::now();
        sleepTime = 1000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
        if (sleepTime > 0) {
            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
        }

    }
}

void fuzzy()
{
    PandaArmBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 7>(PandaState)> jointStateFunc = [](
            PandaState ps) -> Eigen::Vector<double, 7> {
        return ps.robot.jointPosition;
    };

    auto jointStateBlk = FunctionBlock<>::make(jointStateFunc);

    DemuxBlock<Eigen::Vector<double, 7>> demuxBlk;

    using TScalar = double;
    using TType = Eigen::Vector<TScalar, 7>;
    using TMatrix = Eigen::Matrix<TScalar, 7, 7>;

    TScalar T = 1e-4;

    GainBlock<TType, TMatrix> gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI;
    GainBlock<TType, TScalar> gainTinv1{1.f/T}, gainTinv2{1.f/T};
    DelayBlock<TType, 1> delay1, delay2, delay3, delay4, delay5;
    delay1.clear(TType::Zero());
    delay2.clear(TType::Zero());
    delay3.clear(TType::Zero());
    delay4.clear(TType::Zero());
    delay5.clear(TType::Zero());
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

    gainKp.gain().diagonal() << 10, 25, 10, 50, 10, 25, 10;
    gainKi.gain().diagonal() << 0.2, 0.4, 0.4, 0.6, 0.4, 0.6, 0.4;
    gainKd.gain().diagonal() << 0.1, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1;
    gainKd.gain() *= 0.1;

//    gainK.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;
//    gainKuPD.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;
//    gainKuI.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;

    gainK.gain().setZero();
    gainKuPD.gain().setZero();
    gainKuI.gain().setZero();

    SumBlock<TType, 2> sumBlk;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumBlk.zero().setZero();
    sum1.zero().setZero();
    sum2.zero().setZero();
    sum3.zero().setZero();
    sum4.zero().setZero();
    sum5.zero().setZero();
    sum6.zero().setZero();
    sum7.zero().setZero();
    fuzzyPDSum.zero().setZero();

    SimplePIDBlock<Eigen::Vector<double, 7>, ls::blocks::std::SimplePIDBlockOperator::Elementwise> pidBlk;
    pidBlk.pGain() << 10, 25, 10, 50, 10, 25, 10;
    pidBlk.iGain() << 0.2, 0.4, 0.4, 0.6, 0.4, 0.6, 0.4;
    pidBlk.dGain() << 0.1, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1;
    pidBlk.dGain() *= 0.1;
    pidBlk.samplingPeriod() = 1e-4;

    SumBlock<Eigen::Vector<double, 7>, 2> sumBlk2;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumBlk.zero().setZero();
    sumBlk2.zero().setZero();

    ConstantBlock<Eigen::Vector<double, 7>>
            constBlk{{0.2 * M_PI, 0.1 * M_PI, 0, -0.1 * M_PI, 0, M_PI / 2, -0.75 * M_PI}},
//    constBlk{{0,0,0,0,0,0,0}},
    biasBlk{{0, 0, 0, 0, 0, 0, 0}};
//    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};

    SaturationBlock<Eigen::Vector<double, 7>> satBlk;
    satBlk.lower() = -15;
    satBlk.upper() = 15;

    MuxBlock<Eigen::Vector<double, 7>> muxBlk;

    connect(pandaBlk.o<0>(), jointStateBlk.i<0>());
    connect(constBlk.o<0>(), sum1.i<0>());
    connect(jointStateBlk.o<0>(), sum1.i<1>());
//    connect(pidBlk.o<0>(), satBlk.i<0>());
    connect(biasBlk.o<0>(), sumBlk2.i<0>());
    connect(uPID, sumBlk2.i<1>());
    connect(sumBlk2.o<0>(), satBlk.i<0>());

    connect(satBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, jointStateBlk, demuxBlk, pidBlk, constBlk, satBlk, sumBlk2,
                 gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI, gainTinv1, gainTinv2, delay1, delay2, delay3, delay4, delay5, sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum};
    bp.makeGraph();
    aux::Executor ex{bp};
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point begin0 = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    for (int i = 0; i < 100000000; i++) {
        if (i < 150) {
            pandaBlk.trigger();
            continue;
        } else if (i == 150) {
            begin0 = ::std::chrono::steady_clock::now();
        }
//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

//        while (!pandaBlk.done) {}
        begin = ::std::chrono::steady_clock::now();
        ex.trigger();

//        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "PID: " << sumPID.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
//        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
        if (i%1000 == 0) {
            end = ::std::chrono::steady_clock::now();

            ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin0).count() << " "
                        << sum1.o<0>().object[0] << " "
                        << sum1.o<0>().object[1] << " "
                        << sum1.o<0>().object[2] << " "
                        << sum1.o<0>().object[3] << " "
                        << sum1.o<0>().object[4] << " "
                        << sum1.o<0>().object[5] << " "
                        << sum1.o<0>().object[6] << ::std::endl;
            ::std::cout << "Control "
                        << uPID.object[0] << " "
                        << uPID.object[1] << " "
                        << uPID.object[2] << " "
                        << uPID.object[3] << " "
                        << uPID.object[4] << " "
                        << uPID.object[5] << " "
                        << uPID.object[6] << ::std::endl;
        }

//        end = ::std::chrono::steady_clock::now();
//        sleepTime = 500000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
//        if (sleepTime > 0) {
//            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
//        }

    }
}

void torqueControlFullFuzzy()
{
    PandaArmBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 7>(PandaState)> jointStateFunc = [](
            PandaState ps) -> Eigen::Vector<double, 7> {
        return ps.robot.jointPosition;
    };

    auto jointStateBlk = FunctionBlock<>::make(jointStateFunc);

    using TScalar = double;
    using TType = Eigen::Vector<TScalar, 7>;
    using TMatrix = Eigen::Matrix<TScalar, 7, 7>;

    TScalar T = 1e-4;

    GainBlock<TType, TMatrix> gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI;
    GainBlock<TType, TScalar> gainTinv1{1.f/T}, gainTinv2{1.f/T};
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

    gainKp.gain().diagonal() << 10, 25, 10, 50, 10, 25, 10;
    gainKi.gain().diagonal() << 0.2, 0.4, 0.4, 0.6, 0.4, 0.6, 0.4;
    gainKd.gain().diagonal() << 0.1, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1;
    gainKd.gain() *= 0.1;

    gainK.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;
    gainKuPD.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;
    gainKuI.gain().diagonal() << 1, 1, 1, 1, 1, 1, 1;

    SumBlock<TType, 2> sumBlk;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumBlk.zero().setZero();
    sum1.zero().setZero();
    sum2.zero().setZero();
    sum3.zero().setZero();
    sum4.zero().setZero();
    sum5.zero().setZero();
    sum6.zero().setZero();
    sum7.zero().setZero();
    fuzzyPDSum.zero().setZero();

    ConstantBlock<TType>
            constBlk{{0.2 * M_PI, 0.1 * M_PI, 0, -0.1 * M_PI, 0, M_PI / 2, -0.75 * M_PI}},
//    constBlk{{0,0,0,0,0,0,0}},
    biasBlk{{0, 0, 0, 0, 0, 0, 0}};
//    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};

    SaturationBlock<TType> satBlk;
    satBlk.lower() = -15;
    satBlk.upper() = 15;

    connect(sumBlk.o<0>(), sum1.i<0>());
    connect(sum7.o<0>(), satBlk.i<0>());

    connect(pandaBlk.o<0>(), jointStateBlk.i<0>());
    connect(constBlk.o<0>(), sumBlk.i<0>());
    connect(jointStateBlk.o<0>(), sumBlk.i<1>());

    connect(satBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, delay1, delay2, delay5, sum1, sum2, sum3, sum4, sum5, biasBlk, sum6, sum7, fuzzyPDSum, gainKp,
                 constBlk, sumBlk, delay4, satBlk, delay3, jointStateBlk, gainK, gainKi, gainKd, gainKuPD, gainKuI, gainTinv1, gainTinv2};

    aux::Executor ex{bp};
    ex.executionOrder = bp.blocks;
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point begin0 = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    for (int i = 0; i < 100000000; i++) {
        if (i < 150) {
            pandaBlk.trigger();
            continue;
        } else if (i == 150) {
            begin0 = ::std::chrono::steady_clock::now();
        }
//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

        begin = ::std::chrono::steady_clock::now();
        ex.trigger();

//        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "PID: " << sumPID.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
//        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
        if (i%100 == 0) {
            end = ::std::chrono::steady_clock::now();

            ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin0).count() << " "
                        << sumBlk.o<0>().object[0] << " "
                        << sumBlk.o<0>().object[1] << " "
                        << sumBlk.o<0>().object[2] << " "
                        << sumBlk.o<0>().object[3] << " "
                        << sumBlk.o<0>().object[4] << " "
                        << sumBlk.o<0>().object[5] << " "
                        << sumBlk.o<0>().object[6] << ::std::endl;
        }

        end = ::std::chrono::steady_clock::now();
        sleepTime = 10000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
        if (sleepTime > 0) {
            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
        }

    }
}

//void velControlFull()
//{
//    PandaArmVelBlock pandaBlk;
//    pandaBlk.ipAddress() = KF_CONTROL_IP;
//
//    ::std::function<Eigen::Vector<double, 6>(PandaState)> eePosFunc = [] (PandaState ps) -> Eigen::Vector<double, 6> {
//        Eigen::Vector<double, 6> v = Eigen::Vector<double, 6>::Zero();
//        v.block<3,1>(0,0) = ps.robot.endEffectorPose_Base.block<3,3>(0,0) * ps.robot.endEffectorPose_Base.block<3,1>(0,3);
//        return v;
//    };
//
//    auto eePosBlk = FunctionBlock<>::make(eePosFunc);
//
////    ::std::function<Eigen::Vector<double, 6>(Eigen::Vector<double, 3>, Eigen::Vector<double, 3>)> stackVectorsFunc = [] (Eigen::Vector<double, 3> a, Eigen::Vector<double, 3> b) -> Eigen::Vector<double, 6> {
////        Eigen::Vector<double, 6> v;
////        v << a, b;
////
////        return v;
////    };
////
////    auto stackVectorsBlk = FunctionBlock<>::make(stackVectorsFunc);
//
//    DemuxBlock<Eigen::Vector<double, 6>> demuxBlk;
//
//    SimplePIDBlock<Eigen::Vector<double, 6>, ls::blocks::std::SimplePIDBlockOperator::Elementwise> pidBlk;
//    pidBlk.pGain() << 0, 0, 0, 0, 0, 0;
//    pidBlk.iGain() << 1, 1, 1, 0, 0, 0;
//    pidBlk.dGain() << 0, 0, 0, 0, 0, 0;
//    pidBlk.iGain() *= 0.1;
//    pidBlk.samplingPeriod() = 1e-3;
//
//    SumBlock<Eigen::Vector<double, 6>, 2> sumBlk, sumBlk2;
//    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
//    sumBlk.zero().setZero();
//    sumBlk2.zero().setZero();
//
//    ConstantBlock<Eigen::Vector<double, 6>>
//            constBlk{{0.1, 0, 0, 0, 0, 0}},
////    constBlk{{0,0,0,0,0,0,0}},
//    biasBlk{{0,0,0,0,0,0}};
////    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};
//
//    ConstantBlock<Eigen::Vector<double, 3>> zeroBlk{{0,0,0}};
//
//    SaturationBlock<Eigen::Vector<double, 6>> satBlk;
//    satBlk.lower() = -0.05;
//    satBlk.upper() = 0.05;
//
//    MuxBlock<Eigen::Vector<double, 6>> muxBlk;
//
//    connect(pandaBlk.o<0>(), eePosBlk.i<0>());
//    connect(constBlk.o<0>(), sumBlk.i<0>());
////    connect(eePosBlk.o<0>(), stackVectorsBlk.i<0>());
////    connect(zeroBlk.o<0>(), stackVectorsBlk.i<1>());
//    connect(eePosBlk.o<0>(), sumBlk.i<1>());
//    connect(sumBlk.o<0>(), pidBlk.i<0>());
////    connect(pidBlk.o<0>(), satBlk.i<0>());
//    connect(biasBlk.o<0>(), sumBlk2.i<0>());
//    connect(pidBlk.o<0>(), sumBlk2.i<1>());
//    connect(sumBlk2.o<0>(), satBlk.i<0>());
//
//    connect(satBlk.o<0>(), pandaBlk.i<0>());
//
//    BlockPack bp{pandaBlk, eePosBlk, demuxBlk, pidBlk, constBlk, sumBlk, satBlk, sumBlk2, zeroBlk};
//
//    aux::Executor ex{bp};
//    ex.resolveExecutionOrder();
//
//    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
//    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
//    long sleepTime = 0;
//
//    static bool init  = false;
//
//
//    for (int i=0; i<1000000; i++) {
//        if (i < 500) {
//            pandaBlk.trigger();
//            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(100000));
//            continue;
//        }
//
////        if (i < 500) {
////            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
////            pandaBlk.i<0>().propagate();
////        } else {
////            pandaBlk.i<0>().object.setZero();
////            pandaBlk.i<0>().propagate();
////        }
//
//        if (!init) {
//            eePosBlk.trigger();
//            Eigen::Vector3d pos0 = eePosBlk.o<0>().object.block<3,1>(0,0);
//            auto ps = pandaBlk.o<0>().object;
//            constBlk.constant().block<3,1>(0,0) << pos0 + ps.robot.endEffectorPose_Base.block<3,3>(0,0) * Eigen::Vector3d{0.2,0,0};
////            constBlk.o<0>().object = constBlk.constant();
////            constBlk.o<0>().propagate();
//            constBlk.trigger();
//            init = true;
//        }
//
////        begin = ::std::chrono::steady_clock::now();
//        ex.trigger();
//
////        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "State: " << eePosBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Set point: " << constBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
////        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
////        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
////        end = ::std::chrono::steady_clock::now();
////
////        sleepTime = 1000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
////        if (sleepTime > 0) {
////            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
////        }
//
//    }
//}

void velControlFull2()
{
    PandaArmVelBlock pandaBlk;
    pandaBlk.ipAddress() = KF_CONTROL_IP;

    ::std::function<Eigen::Vector<double, 3>(PandaState)> eePosFunc = [](PandaState ps) -> Eigen::Vector<double, 3> {
//        return ps.robot.endEffectorPose_Base.block<3,3>(0,0) * ps.robot.endEffectorPose_Base.block<3,1>(0,3);
        return ps.robot.endEffectorPose_Base.block<3, 1>(0, 3);
    };

    auto eePosBlk = FunctionBlock<>::make(eePosFunc);

//    ::std::function<Eigen::Vector<double, 6>(Eigen::Vector<double, 3>, Eigen::Vector<double, 3>)> stackVectorsFunc = [] (Eigen::Vector<double, 3> a, Eigen::Vector<double, 3> b) -> Eigen::Vector<double, 6> {
//        Eigen::Vector<double, 6> v;
//        v << a, b;
//
//        return v;
//    };
//
//    auto stackVectorsBlk = FunctionBlock<>::make(stackVectorsFunc);


    SimplePIDBlock<Eigen::Vector<double, 3>, ls::blocks::std::SimplePIDBlockOperator::Elementwise> pidBlk;
    pidBlk.pGain() << 0, 0, 0;
    pidBlk.iGain() << 1, 1, 1;
    pidBlk.dGain() << -0.001, -0.001, -0.001;
    pidBlk.iGain() *= 0.01;
    pidBlk.samplingPeriod() = 1e-5;

    SumBlock<Eigen::Vector<double, 3>, 3> sumBlk;
    sumBlk.setOperators(decltype(sumBlk)::Plus, decltype(sumBlk)::Plus, decltype(sumBlk)::Minus);
    sumBlk.zero().setZero();

    ConstantBlock<Eigen::Vector<double, 3>>
            constBlk{{0.1, 0, 0}},
//    constBlk{{0,0,0,0,0,0,0}},
    biasBlk{{0, 0, 0}};
//    biasBlk{{-0.170114, -19.7043, -4.7674, 24.7622, -2.80476, 1.16146, 0.28083}};

    ConstantBlock<Eigen::Vector<double, 3>> setpointBlk{{0.1, 0, 0}};

    SaturationBlock<Eigen::Vector<double, 3>> satBlk;
    satBlk.lower() = -0.05;
    satBlk.upper() = 0.05;

    connect(pandaBlk.o<0>(), eePosBlk.i<0>());
    connect(constBlk.o<0>(), sumBlk.i<0>());
    connect(setpointBlk.o<0>(), sumBlk.i<1>());
    connect(eePosBlk.o<0>(), sumBlk.i<2>());
    connect(sumBlk.o<0>(), pidBlk.i<0>());
//    connect(pidBlk.o<0>(), satBlk.i<0>());
//    connect(biasBlk.o<0>(), sumBlk2.i<0>());
//    connect(pidBlk.o<0>(), sumBlk2.i<1>());
    connect(pidBlk.o<0>(), satBlk.i<0>());
    connect(satBlk.o<0>(), pandaBlk.i<0>());

    BlockPack bp{pandaBlk, eePosBlk, pidBlk, constBlk, sumBlk, satBlk, setpointBlk};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();

    ::std::chrono::steady_clock::time_point begin = ::std::chrono::steady_clock::now();
    ::std::chrono::steady_clock::time_point end = ::std::chrono::steady_clock::now();
    long sleepTime = 0;

    static bool init = false;


    for (int i = 0; i < 100000000; i++) {
        if (i < 500) {
            pandaBlk.trigger();
            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(100000));
            continue;
        }

//        if (i < 500) {
//            pandaBlk.i<0>().object << 5, 0, 0, 0, 0, 0, 0;
//            pandaBlk.i<0>().propagate();
//        } else {
//            pandaBlk.i<0>().object.setZero();
//            pandaBlk.i<0>().propagate();
//        }

        if (!init) {
            eePosBlk.trigger();
            constBlk.constant() = eePosBlk.o<0>().object;;
//            constBlk.o<0>().object = constBlk.constant();
//            constBlk.o<0>().propagate();
            constBlk.trigger();
            init = true;
        }

        begin = ::std::chrono::steady_clock::now();
        ex.trigger();
        end = ::std::chrono::steady_clock::now();

        ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin).count() << " " << sumBlk.o<0>().object.transpose() << ::std::endl;

//        ::std::cout << "[" << i << "] dt: " << pandaBlk.o<0>().object.robot.duration.toSec() << ::std::endl;
//        ::std::cout << "State: " << eePosBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Error: " << sumBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Set point: " << constBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Control: " << satBlk.o<0>().object.transpose() << ::std::endl;
//        ::std::cout << "Joints: " << pandaBlk.o<0>().object.robot.jointPosition.transpose() << ::std::endl;
//        ::std::cout << "Torques: " << pandaBlk.o<0>().object.robot.jointTorque.transpose() << ::std::endl;
//        end = ::std::chrono::steady_clock::now();
//        ::std::cout << ::std::chrono::duration_cast<::std::chrono::microseconds>(end - begin).count() << ::std::endl;
//
//        sleepTime = 1000 - ::std::chrono::duration_cast<::std::chrono::nanoseconds>(end - begin).count();
//        if (sleepTime > 0) {
//            ::std::this_thread::sleep_for(::std::chrono::nanoseconds(sleepTime));
//        }

    }
}

void autofocus()
{
    const unsigned int NBuffer = 100;

    ThermographerBlock<float> blkT;
//    left 21074216.xml
//    right 20114198.xml
    blkT.serial() = "/home/advcc/20104321.xml";
//    blkT.serial() = "/home/advcc/21074216.xml";
    blkT.i<0>() = 0;

    GainBlock<decltype(blkT)::Matrix, Eigen::Matrix<float, 5, 7>, GainBlockOperator::Convolution> blkConvX, blkConvX2;
    GainBlock<decltype(blkT)::Matrix, Eigen::Matrix<float, 7, 5>, GainBlockOperator::Convolution> blkConvY, blkConvY2;

    SumBlock<decltype(blkT)::Matrix, 2> blkSum;
    blkSum.zero().setZero();

    SumBlock<float, 2> blkSumSharpness, blkSumPID;
    blkSumSharpness.zero() = 0;
    blkSumPID.zero() = 0;
    blkSumSharpness.setOperators(decltype(blkSumSharpness)::Plus, decltype(blkSumSharpness)::Minus);

    UnaryAbsBlock<decltype(blkT)::Matrix> blkAbs;

    MinMaxIdxBlock<decltype(blkT)::Matrix, ls::blocks::std::MinMaxIdxBlockOperator::Max> blkMaxIdx;

    UnaryMeanBlock<decltype(blkT)::Matrix> blkMean;

    BufferBlock<float, NBuffer> blkBuffer;
    blkBuffer.clear(0);

    SignumBlock<float, float> blkSignum;

    DeadzoneBlock<float> blkDeadzone, blkDeadzonePID;
    blkDeadzone.lower() = -3e-2;
    blkDeadzone.upper() = 3e-2;

    SaturationBlock<float> blkSaturation;
    blkSaturation.lower() = 0;
    blkSaturation.upper() = 60;

    //------------------- ANRA

//    auto nrBwdDiffLambda = [](
//            decltype(blkT)::Matrix T0,
//            decltype(blkT)::Matrix T1,
//            decltype(blkT)::Matrix T2,
//            decltype(blkT)::Matrix T3,
//            decltype(blkT)::Matrix T4,
//            decltype(blkT)::Matrix T5,
//            decltype(blkT)::Matrix T6,
//            decltype(blkT)::Matrix T7) -> decltype(blkT)::Matrix {
//        return (1.f / (64.f / 27.f)) * (T0.array() + 5 * T1.array() + 9 * T2.array() + 5 * T3.array() - 5 * T4.array() - 9 * T5.array() - 5 * T6.array() - T7.array());
//    };
//
//    FunctionBlock<decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//    decltype(blkT)::Matrix>::Function nrBwdDiffFunc = nrBwdDiffLambda;

//    FunctionBlock<decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix,
//            decltype(blkT)::Matrix> blkNrBwdDiff;

//    auto blkNrBwdDiff = FunctionBlock<>::make(nrBwdDiffFunc);
//
//    BufferBlock<decltype(blkT)::Matrix, 8> blkTBuffer;
//    blkTBuffer.clear(decltype(blkT)::Matrix::Zero());
//
//    GainBlock<decltype(blkT)::Matrix, float> blkLaplacianScale{1.f / ((90e-6) * (90e-6))};

//    ::std::function<decltype(blkT)::Matrix(decltype(blkT)::Matrix)> rtcFunc = [](decltype(blkT)::Matrix Tt) -> decltype(blkT)::Matrix {
//        decltype(blkT)::Matrix RTC = Tt.unaryExpr([](float x) { return ::std::exp(::std::abs(x)); });
//        RTC.array() -= RTC.minCoeff();
//        RTC.array() *= (1.f / RTC.maxCoeff());
//
//        return RTC;
//    };
//
//    auto blkRTC = FunctionBlock<>::make(rtcFunc);

//    ::std::function<float(float)> focus2pxFunc = [] (float focus) -> float {
//        return 81e-6 + 18e-6 * focus / 100.f;
//    };
//
//    auto blkFocus2Px = FunctionBlock<>::make(focus2pxFunc);
//
//    ::std::function<float(float, decltype(blkT)::Matrix, decltype(blkT)::Matrix, decltype(blkT)::Matrix)> anraFunc = [](float h, decltype(blkT)::Matrix RTC, decltype(blkT)::Matrix Tt, decltype(blkT)::Matrix TxxTyy) -> float {
//        decltype(blkT)::Matrix a = h*h * Tt.array() / TxxTyy.array();
//        return (a.array() * RTC.array()).sum();
//    };
//
//    auto blkANRA = FunctionBlock<>::make(anraFunc);
//
//    connect(blkSum.o<0>(), blkANRA.i<3>());
//    connect(blkT.o<0>(), blkTBuffer.i<0>());
//    connect(blkTBuffer.o<0>(), blkNrBwdDiff.i<0>());
//    connect(blkTBuffer.o<1>(), blkNrBwdDiff.i<1>());
//    connect(blkTBuffer.o<2>(), blkNrBwdDiff.i<2>());
//    connect(blkTBuffer.o<3>(), blkNrBwdDiff.i<3>());
//    connect(blkTBuffer.o<4>(), blkNrBwdDiff.i<4>());
//    connect(blkTBuffer.o<5>(), blkNrBwdDiff.i<5>());
//    connect(blkTBuffer.o<6>(), blkNrBwdDiff.i<6>());
//    connect(blkTBuffer.o<7>(), blkNrBwdDiff.i<7>());
//    connect(blkNrBwdDiff.o<0>(), blkANRA.i<2>());
//    connect(blkNrBwdDiff.o<0>(), blkRTC.i<0>());
//    connect(blkRTC.o<0>(), blkANRA.i<1>());
//    connect(blkT.o<1>(), blkFocus2Px.i<0>());
//    connect(blkFocus2Px.o<0>(), blkANRA.i<0>());

    //------------------- ANRA

    ::std::function<float(float, float)> controlFunc = [](float sign, float focus) -> float {
//        static bool isInit = false;
//        static float f;
//        if (!isInit) {
//            f = focus;
//            isInit = true;
//        }

//        focus -= 2.5f*sign;
        focus -= 1.f * sign;

        return focus;
    };

    auto blkControlFunc = FunctionBlock<>::make(controlFunc);

    SimplePIDBlock<float, ls::blocks::std::SimplePIDBlockOperator::Scalar, ls::blocks::std::SimplePIDBlockParameter::AdditionalInput> blkPID;
    blkPID.samplingPeriod() = 5e-3;
    blkPID.pGain() = -15;
    blkPID.iGain() = 5;
    blkPID.dGain() = 0;
    blkDeadzonePID.lower() = -1e-1;
    blkDeadzonePID.upper() = 1e-1;

    SwitchBlock<float, SwitchBlockParameter::AdditionalInput> blkSwitch, blkSwitch2;

    Eigen::Matrix<float, 5, 7> kNRFx;
    Eigen::Matrix<float, 7, 5> kNRFy;

    kNRFx << -1, -4, -5, 0, 5, 4, 1,
            -4, -16, -20, 0, 20, 16, 4,
            -6, -24, -30, 0, 30, 24, 6,
            -4, -16, -20, 0, 20, 16, 4,
            -1, -4, -5, 0, 5, 4, 1;
    kNRFx *= -(1.f / 512);

    kNRFy = kNRFx.transpose();

    blkConvX.gain() = kNRFx;
    blkConvX2.gain() = kNRFx;

    blkConvY.gain() = kNRFy;
    blkConvY2.gain() = kNRFy;

    connect(blkT.o<0>(), blkConvX.i<0>());
    connect(blkT.o<0>(), blkConvY.i<0>());
    connect(blkConvX.o<0>(), blkConvX2.i<0>());
    connect(blkConvY.o<0>(), blkConvY2.i<0>());
    connect(blkConvX2.o<0>(), blkSum.i<0>());
    connect(blkConvY2.o<0>(), blkSum.i<1>());
    connect(blkSum.o<0>(), blkAbs.i<0>());
    connect(blkAbs.o<0>(), blkMaxIdx.i<0>());
    connect(blkAbs.o<0>(), blkMean.i<0>());
    connect(blkMean.o<0>(), blkSwitch.i<0>());
    connect(blkMaxIdx.o<2>(), blkSwitch.i<1>());
    connect(blkSwitch.o<0>(), blkBuffer.i<0>());
    connect(blkBuffer.o<0>(), blkSumSharpness.i<0>());
    connect(blkBuffer.o<NBuffer - 1>(), blkSumSharpness.i<1>());
    connect(blkSumSharpness.o<0>(), blkDeadzone.i<0>());
    connect(blkDeadzone.o<0>(), blkSignum.i<0>());
    connect(blkSignum.o<0>(), blkControlFunc.i<0>());
    connect(blkT.o<1>(), blkControlFunc.i<1>());
    connect(blkControlFunc.o<0>(), blkSwitch2.i<0>());
    connect(blkSumSharpness.o<0>(), blkPID.i<0>());
    connect(blkPID.o<0>(), blkDeadzonePID.i<0>());
    connect(blkDeadzonePID.o<0>(), blkSumPID.i<0>());
    connect(blkT.o<1>(), blkSumPID.i<1>());
    connect(blkSumPID.o<0>(), blkSwitch2.i<1>());
    connect(blkSwitch2.o<0>(), blkSaturation.i<0>());
    connect(blkSaturation.o<0>(), blkT.i<0>());

    BlockPack bp{blkT, blkConvX, blkConvX2, blkConvY, blkConvY2, blkSum, blkAbs, blkMaxIdx, blkBuffer, blkSumSharpness,
                 blkSignum, blkDeadzone, blkControlFunc, blkDeadzonePID, blkSaturation, blkMean, blkSwitch, blkSwitch2,
                 blkPID, blkSumPID};
//    BlockPack bp{blkT, blkConvX, blkConvX2, blkConvY, blkConvY2, blkSum, blkAbs, blkMaxIdx, blkBuffer, blkSumSharpness,
//                 blkSignum, blkDeadzone, blkControlFunc, blkDeadzonePID, blkSaturation, blkMean, blkSwitch, blkSwitch2,
//                 blkPID, blkSumPID,
//                 blkTBuffer, blkANRA, blkFocus2Px, blkNrBwdDiff, blkRTC};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();


    auto getTemperature = [&](double x, double y) -> double {
        if ((y >= 288 || y < 0) || (x >= 382 || x < 0))
            return 0;

        return blkT.o<0>().object((int) round(y), (int) round(x));
    };

    cv::Mat M(288, 382, CV_32FC1), MLaplacian(288, 382, CV_32FC1);
    Eigen::Map<Eigen::Matrix<float, 288, 382, Eigen::RowMajor>> M_Eigen(M.ptr<float>(), M.rows, M.cols);
    Eigen::Map<Eigen::Matrix<float, 288, 382, Eigen::RowMajor>> MLaplacian_Eigen(MLaplacian.ptr<float>(),
                                                                                 MLaplacian.rows, MLaplacian.cols);
    M_Eigen.setZero();
    MLaplacian_Eigen.setZero();

//    blkT.o<0>() = M_Eigen;

//    M.resize(382*288);
//    M.reshape(382, 288);

//    ::std::stringstream ss;
//    ex.makeDotFile(ss, true, true);

    // Toggle between mean and max focus.
    blkSwitch.toggle();

    // Toggle between custom control function and PID.
    blkSwitch2.toggle();

    ::std::chrono::steady_clock::time_point begin, end;

    for (int i = 0; i < 10000; i++) {
        if (i < 250) {
            blkT.i<0>() = 0;
            blkT.trigger();
            continue;
        } else if (i == 250) {
            begin = ::std::chrono::steady_clock::now();
        }

//        blkT.trigger();
        ex.trigger();

        if (i > 5) {
//            matplot::gca()->xlim({0, 382});
//            matplot::gca()->ylim({0, 288});
//            matplot::fcontour(getTemperature)->filled(true).colormap_line_when_filled(true);
//
//            matplot::show();

            M_Eigen << blkT.o<0>().object;
            M_Eigen *= (1.0 / M_Eigen.maxCoeff());

            MLaplacian_Eigen << blkAbs.o<0>().object;
            MLaplacian_Eigen.unaryExpr([&](float a) -> float { return a - MLaplacian_Eigen.minCoeff(); });
            MLaplacian_Eigen *= (1.0 / MLaplacian_Eigen.maxCoeff());
//        cv::eigen2cv((blkT.o<0>().object/blkT.o<0>().object.maxCoeff()).eval(), M);
            cv::namedWindow("Temperature field", cv::WINDOW_AUTOSIZE);
            cv::imshow("Temperature field", M);
            cv::namedWindow("Laplacian field", cv::WINDOW_AUTOSIZE);
            cv::imshow("Laplacian field", MLaplacian);
            cv::waitKey(5);
        }

//        ::std::cout << "[" << i << "] sharpness change: " << blkSignum.o<0>().object << " | "
//                    << blkSumSharpness.o<0>().object << ::std::endl;
//        ::std::cout << "[" << i << "] setpoint (" << blkSwitch2.o<0>().object << "): " << blkSaturation.o<0>().object
//                    << ::std::endl;

//        ::std::cout << "[" << i << "] mean temperature: " << blkT.o<0>().object.mean() << ", max: " << blkT.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv x: " << blkConvX.o<0>().object.mean() << ", max: " << blkConvX.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv x 2: " << blkConvX2.o<0>().object.mean() << ", max: " << blkConvX2.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv y: " << blkConvY.o<0>().object.mean() << ", max: " << blkConvY.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv y 2: " << blkConvY2.o<0>().object.mean() << ", max: " << blkConvY2.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean laplacian: " << blkSum.o<0>().object.mean() << ", max: " << blkMaxIdx.o<2>().object << " at (" << blkMaxIdx.o<0>().object << ", " << blkMaxIdx.o<1>().object << ")" << ::std::endl;
//        ::std::cout << "[" << i << "] focus: " << blkT.o<1>().object << ::std::endl;
//        ::std::cout << "[" << i << "] diffusivity: " << blkANRA.o<0>().object << ::std::endl;
        end = ::std::chrono::steady_clock::now();
        ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin).count() << ", " << blkMaxIdx.o<2>().object << ", " << blkT.o<1>().object << ::std::endl;
    }
}

void autofocusFuzzy()
{
    using TScalar = float;
    using TType = float;
    using TMatrix = float;

    TScalar T = 5e-3;

    GainBlock<TType, TMatrix> gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI;
    GainBlock<TType, TScalar> gainTinv1{1.f / T}, gainTinv2{1.f / T};
    DelayBlock<TType> delay1, delay2, delay3, delay4, delay5;
    SumBlock<TType, 2> sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum;

    sum1.setOperators(decltype(sum1)::Plus, decltype(sum1)::Minus);
    sum2.setOperators(decltype(sum2)::Plus, decltype(sum2)::Minus);
    sum5.setOperators(decltype(sum5)::Plus, decltype(sum5)::Minus);

    connect(sum1.o<0>(), delay1.i<0>());
    connect(delay1.o<0>(), gainKi.i<0>());
    connect(gainKi.o<0>(), sum4.i<0>());

    connect(sum1.o<0>(), delay2.i<0>());
    connect(delay2.o<0>(), sum3.i<0>());
    connect(sum1.o<0>(), sum3.i<1>());
    connect(sum3.o<0>(), gainTinv2.i<0>());
    connect(gainTinv2.o<0>(), gainKp.i<0>());

    connect(sum1.o<0>(), sum2.i<0>());
    connect(sum1.o<0>(), delay3.i<0>());
    connect(delay3.o<0>(), sum2.i<1>());
    connect(sum2.o<0>(), gainTinv1.i<0>());

    connect(gainTinv1.o<0>(), gainK.i<0>());
    connect(gainTinv1.o<0>(), gainKd.i<0>());
    connect(gainK.o<0>(), sum4.i<1>());

    connect(gainKp.o<0>(), fuzzyPDSum.i<0>());
    connect(gainKd.o<0>(), fuzzyPDSum.i<1>());
    connect(fuzzyPDSum.o<0>(), gainKuPD.i<0>());
    connect(gainKuPD.o<0>(), sum5.i<0>());
    connect(sum5.o<0>(), delay4.i<0>());
    connect(delay4.o<0>(), sum5.i<1>());

    connect(sum4.o<0>(), gainKuI.i<0>());
    connect(gainKuI.o<0>(), sum6.i<0>());
    connect(sum6.o<0>(), delay5.i<0>());
    connect(delay5.o<0>(), sum6.i<1>());

    connect(sum5.o<0>(), sum7.i<0>());
    connect(sum6.o<0>(), sum7.i<1>());

    gainKp.gain() = -15;
    gainKi.gain() = 5;
    gainKd.gain() = 0;

    gainK.gain() = 1;
    gainKuPD.gain() = 1;
    gainKuI.gain() = 1;

    const unsigned int NBuffer = 100;

    ThermographerBlock<float> blkT;
//    left 21074216.xml
//    right 20114198.xml
    blkT.serial() = "/home/advcc/20104321.xml";
//    blkT.serial() = "/home/advcc/21074216.xml";
    blkT.i<0>() = 0;

    GainBlock<decltype(blkT)::Matrix, Eigen::Matrix<float, 5, 7>, GainBlockOperator::Convolution> blkConvX, blkConvX2;
    GainBlock<decltype(blkT)::Matrix, Eigen::Matrix<float, 7, 5>, GainBlockOperator::Convolution> blkConvY, blkConvY2;

    SumBlock<decltype(blkT)::Matrix, 2> blkSum;
    blkSum.zero().setZero();

    SumBlock<float, 2> blkSumSharpness, blkSumPID;
    blkSumSharpness.zero() = 0;
    blkSumPID.zero() = 0;
    blkSumSharpness.setOperators(decltype(blkSumSharpness)::Plus, decltype(blkSumSharpness)::Minus);

    UnaryAbsBlock<decltype(blkT)::Matrix> blkAbs;

    MinMaxIdxBlock<decltype(blkT)::Matrix, ls::blocks::std::MinMaxIdxBlockOperator::Max> blkMaxIdx;

    UnaryMeanBlock<decltype(blkT)::Matrix> blkMean;

    BufferBlock<float, NBuffer> blkBuffer;
    blkBuffer.clear(0);

    SignumBlock<float, float> blkSignum;

    DeadzoneBlock<float> blkDeadzone, blkDeadzonePID;
    blkDeadzone.lower() = -3e-2;
    blkDeadzone.upper() = 3e-2;

    SaturationBlock<float> blkSaturation;
    blkSaturation.lower() = 0;
    blkSaturation.upper() = 60;

    ::std::function<float(float, float)> controlFunc = [](float sign, float focus) -> float {
//        static bool isInit = false;
//        static float f;
//        if (!isInit) {
//            f = focus;
//            isInit = true;
//        }

//        focus -= 2.5f*sign;
        focus -= 1.f * sign;

        return focus;
    };

    auto blkControlFunc = FunctionBlock<>::make(controlFunc);

    blkDeadzonePID.lower() = -1e-1;
    blkDeadzonePID.upper() = 1e-1;

    SwitchBlock<float, SwitchBlockParameter::AdditionalInput> blkSwitch, blkSwitch2;

    Eigen::Matrix<float, 5, 7> kNRFx;
    Eigen::Matrix<float, 7, 5> kNRFy;

    kNRFx << -1, -4, -5, 0, 5, 4, 1,
            -4, -16, -20, 0, 20, 16, 4,
            -6, -24, -30, 0, 30, 24, 6,
            -4, -16, -20, 0, 20, 16, 4,
            -1, -4, -5, 0, 5, 4, 1;
    kNRFx *= -(1.f / 512);

    kNRFy = kNRFx.transpose();

    blkConvX.gain() = kNRFx;
    blkConvX2.gain() = kNRFx;

    blkConvY.gain() = kNRFy;
    blkConvY2.gain() = kNRFy;

    connect(blkT.o<0>(), blkConvX.i<0>());
    connect(blkT.o<0>(), blkConvY.i<0>());
    connect(blkConvX.o<0>(), blkConvX2.i<0>());
    connect(blkConvY.o<0>(), blkConvY2.i<0>());
    connect(blkConvX2.o<0>(), blkSum.i<0>());
    connect(blkConvY2.o<0>(), blkSum.i<1>());
    connect(blkSum.o<0>(), blkAbs.i<0>());
    connect(blkAbs.o<0>(), blkMaxIdx.i<0>());
    connect(blkAbs.o<0>(), blkMean.i<0>());
    connect(blkMean.o<0>(), blkSwitch.i<0>());
    connect(blkMaxIdx.o<2>(), blkSwitch.i<1>());
    connect(blkSwitch.o<0>(), blkBuffer.i<0>());
    connect(blkBuffer.o<0>(), blkSumSharpness.i<0>());
    connect(blkBuffer.o<NBuffer - 1>(), blkSumSharpness.i<1>());
    connect(blkSumSharpness.o<0>(), blkDeadzone.i<0>());
    connect(blkDeadzone.o<0>(), blkSignum.i<0>());
    connect(blkSignum.o<0>(), blkControlFunc.i<0>());
    connect(blkT.o<1>(), blkControlFunc.i<1>());
    connect(blkControlFunc.o<0>(), blkSwitch2.i<0>());
    connect(blkSumSharpness.o<0>(), sum1.i<0>());
    connect(sum7.o<0>(), blkDeadzonePID.i<0>());
    connect(blkDeadzonePID.o<0>(), blkSumPID.i<0>());
    connect(blkT.o<1>(), blkSumPID.i<1>());
    connect(blkSumPID.o<0>(), blkSwitch2.i<1>());
    connect(blkSwitch2.o<0>(), blkSaturation.i<0>());
    connect(blkSaturation.o<0>(), blkT.i<0>());

    BlockPack bp{blkT, blkConvX, blkConvX2, blkConvY, blkConvY2, blkSum, blkAbs, blkMaxIdx, blkBuffer, blkSumSharpness,
                 blkSignum, blkDeadzone, blkControlFunc, blkDeadzonePID, blkSaturation, blkMean, blkSwitch, blkSwitch2,
                 blkSumPID,
                 gainK, gainKp, gainKd, gainKi, gainKuPD, gainKuI, gainTinv1, gainTinv2, delay1,
                 delay2, delay3, delay4, delay5, sum1, sum2, sum3, sum4, sum5, sum6, sum7, fuzzyPDSum};
//    BlockPack bp{blkT, blkConvX, blkConvX2, blkConvY, blkConvY2, blkSum, blkAbs, blkMaxIdx, blkBuffer, blkSumSharpness,
//                 blkSignum, blkDeadzone, blkControlFunc, blkDeadzonePID, blkSaturation, blkMean, blkSwitch, blkSwitch2,
//                 blkPID, blkSumPID,
//                 blkTBuffer, blkANRA, blkFocus2Px, blkNrBwdDiff, blkRTC};

    aux::Executor ex{bp};
    ex.resolveExecutionOrder();


    auto getTemperature = [&](double x, double y) -> double {
        if ((y >= 288 || y < 0) || (x >= 382 || x < 0))
            return 0;

        return blkT.o<0>().object((int) round(y), (int) round(x));
    };

    cv::Mat M(288, 382, CV_32FC1), MLaplacian(288, 382, CV_32FC1);
    Eigen::Map<Eigen::Matrix<float, 288, 382, Eigen::RowMajor>> M_Eigen(M.ptr<float>(), M.rows, M.cols);
    Eigen::Map<Eigen::Matrix<float, 288, 382, Eigen::RowMajor>> MLaplacian_Eigen(MLaplacian.ptr<float>(),
                                                                                 MLaplacian.rows, MLaplacian.cols);
    M_Eigen.setZero();
    MLaplacian_Eigen.setZero();

//    blkT.o<0>() = M_Eigen;

//    M.resize(382*288);
//    M.reshape(382, 288);

//    ::std::stringstream ss;
//    ex.makeDotFile(ss, true, true);

// Toggle between mean and max focus.
    blkSwitch.toggle();

// Toggle between custom control function and PID.
    blkSwitch2.toggle();

    ::std::chrono::steady_clock::time_point begin, end;

    for (int i = 0; i < 10000; i++) {
        if (i < 250) {
            blkT.i<0>() = 0;
            blkT.trigger();
            continue;
        } else if (i == 250) {
            begin = ::std::chrono::steady_clock::now();
        }

//        blkT.trigger();
        ex.trigger();

        if (i > 5) {
//            matplot::gca()->xlim({0, 382});
//            matplot::gca()->ylim({0, 288});
//            matplot::fcontour(getTemperature)->filled(true).colormap_line_when_filled(true);
//
//            matplot::show();

            M_Eigen << blkT.o<0>().object;
            M_Eigen *= (1.0 / M_Eigen.maxCoeff());

            MLaplacian_Eigen << blkAbs.o<0>().object;
            MLaplacian_Eigen.unaryExpr([&](float a) -> float { return a - MLaplacian_Eigen.minCoeff(); });
            MLaplacian_Eigen *= (1.0 / MLaplacian_Eigen.maxCoeff());
//        cv::eigen2cv((blkT.o<0>().object/blkT.o<0>().object.maxCoeff()).eval(), M);
            cv::namedWindow("Temperature field", cv::WINDOW_AUTOSIZE);
            cv::imshow("Temperature field", M);
            cv::namedWindow("Laplacian field", cv::WINDOW_AUTOSIZE);
            cv::imshow("Laplacian field", MLaplacian);
            cv::waitKey(5);
        }

//        ::std::cout << "[" << i << "] sharpness change: " << blkSignum.o<0>().object << " | "
//                    << blkSumSharpness.o<0>().object << ::std::endl;
//        ::std::cout << "[" << i << "] setpoint (" << blkSwitch2.o<0>().object << "): " << blkSaturation.o<0>().object
//                    << ::std::endl;

//        ::std::cout << "[" << i << "] mean temperature: " << blkT.o<0>().object.mean() << ", max: " << blkT.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv x: " << blkConvX.o<0>().object.mean() << ", max: " << blkConvX.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv x 2: " << blkConvX2.o<0>().object.mean() << ", max: " << blkConvX2.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv y: " << blkConvY.o<0>().object.mean() << ", max: " << blkConvY.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean deriv y 2: " << blkConvY2.o<0>().object.mean() << ", max: " << blkConvY2.o<0>().object.maxCoeff() << ::std::endl;
//        ::std::cout << "[" << i << "] mean laplacian: " << blkSum.o<0>().object.mean() << ", max: " << blkMaxIdx.o<2>().object << " at (" << blkMaxIdx.o<0>().object << ", " << blkMaxIdx.o<1>().object << ")" << ::std::endl;
//        ::std::cout << "[" << i << "] focus: " << blkT.o<1>().object << ::std::endl;
//        ::std::cout << "[" << i << "] diffusivity: " << blkANRA.o<0>().object << ::std::endl;
        end = ::std::chrono::steady_clock::now();
        ::std::cout << ::std::chrono::duration_cast<::std::chrono::milliseconds>(end - begin).count() << ", " << blkMaxIdx.o<2>().object << ", " << blkT.o<1>().object << ::std::endl;
    }
}

int main()
{
//    autofocus();
//    autofocusFuzzy();
//    torqueControlFull();
    torqueControlFullAntiwindup();
//    fuzzy();
//    torqueControlFullFuzzy();
//    velControlFull2();

//    FuzzyPIDExample::example();
//    FuzzyPIDExample::exampleSL();

    return 0;
}