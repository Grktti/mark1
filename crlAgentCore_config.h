/***************************************************************************
 * agentCore_config.hpp
 *
 * Copyright (C) 2023 - Hiroshi IGARASHI
 * Aug. 22, 2023
 *****************************************************************************/


#ifndef __AGENT_CORE_CONFIG_H__
#define __AGENT_CORE_CONFIG_H__

#include <iostream>
#include <vector>
#include <random>
#include <cmath>


std::random_device seed0;
std::random_device seed1;     // 非決定的な乱数生成器を生成


template<class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
//    os << "[";
    for (int i = 0; i < (int)v.size(); i++) {
        //os << std::defaultfloat << v[i];
        os << std::fixed << std::showpos << v[i];
        if (i != (int)v.size() - 1) os << ", ";
    }
//    os << " ]";
    os << std::defaultfloat;
    return os;
}

template<class T>
std::vector<T> operator+(const std::vector<T> &v1, const std::vector<T> &v2) {
    if (v1.size() != v2.size()) std::cerr << "#error: size() is not equal. @vector operator-" << std::endl;
    std::vector<T> ans = v1;
    for (size_t i = 0, size = ans.size(); i < size; ++i)
        ans[i] += v2[i];
    return ans;
}

template<class T>
std::vector<T> operator-(const std::vector<T> &v1, const std::vector<T> &v2) {
    if (v1.size() != v2.size()) std::cerr << "#error: size() is not equal. @vector operator-" << std::endl;
    std::vector<T> ans(v1.size());
    for (size_t i = 0, size = ans.size(); i < size; ++i)
        ans[i] = v1[i] - v2[i];
    return ans;
}

template<class T>
std::vector<T> operator/(const std::vector<T> &v1, const double v2) {
    std::vector<T> ans(v1.size());
    for (size_t i = 0, size = ans.size(); i < size; ++i)
        ans[i] = v1[i] / v2;
    return ans;
}

template<class T>
std::vector<T> operator*(const std::vector<T> &v1, const double v2) {
    std::vector<T> ans(v1.size());
    for (size_t i = 0, size = ans.size(); i < size; ++i)
        ans[i] = v1[i] * v2;
    return ans;
}

template<class T>
std::vector<T> operator*(const double v1, const std::vector<T> &v2) {
    return v2 * v1;
}


#define STAT_SIZE 8 // [x, y, dx, dy, ddx, ddy, ux, uy]
#define U_SIZE 2 // 最終的な入力ベクトルのサイズ [次元]

#define DEFAULT_U_MAX 50.0
#define DEFAULT_V_MAX 50.0
#define DEFAULT_M 1.0
#define DEFAULT_D 1.0
#define DEFAULT_G 4.0 // input gain
#define DEFAULT_COL_EPS 0.2
#define DEFAULT_PERSONAL_SPACE !.0
#define DEFAULT_SIGHT_RANGE 200
#define DEFAULT_SIGHT_ANGLE 360
#define DEFAULT_SIGHT_SIGMA 0.0
#define DEFAULT_RADIUS 2.0
//#define MAX_CHECK

#define VIEWER_DOG_NUM 20 // DO NOT CHANGE! (linking to py_viewer)
#define VIEWER_SHEEP_NUM 100 // DO NOT CHANGE! (linking to py_viewer)
#define VIEWER_AGENT_NUM 100

#define DEFAULT_FIELD_X_MAX 100
#define DEFAULT_FIELD_X_MIN -100
#define DEFAULT_FIELD_Y_MAX 100
#define DEFAULT_FIELD_Y_MIN -100

namespace agentcore {
    typedef struct {
        double SIGHT_RANGE; // 視野範囲
        double SIGHT_ANGLE; // 視野角度
        double SIGHT_SIGMA; // 視野角度のばらつき
        double RADIUS; // 半径
        double M, D, G; // mass, damping, input gain
        double U_MAX; // 入力の最大値
        double V_MAX; // 入力の最大値
    } agent_physical_t;

    typedef struct {
        double X_MAX;
        double X_MIN;
        double Y_MAX;
        double Y_MIN;
        double X_SIZE;
        double Y_SIZE;
    } field_environment_t;

    void init_physical_param(agent_physical_t &p) {
        p.SIGHT_RANGE = DEFAULT_SIGHT_RANGE;
        p.SIGHT_ANGLE = DEFAULT_SIGHT_ANGLE;
        p.SIGHT_SIGMA = DEFAULT_SIGHT_SIGMA;
        p.RADIUS = DEFAULT_RADIUS;
        p.M = DEFAULT_M;
        p.D = DEFAULT_D;
        p.G = DEFAULT_G;
        p.U_MAX = DEFAULT_U_MAX;
        p.V_MAX = DEFAULT_V_MAX;
    }

    void init(field_environment_t &env) {
        env.X_MAX = DEFAULT_FIELD_X_MAX;
        env.X_MIN = DEFAULT_FIELD_X_MIN;
        env.Y_MAX = DEFAULT_FIELD_Y_MAX;
        env.Y_MIN = DEFAULT_FIELD_Y_MIN;
        env.X_SIZE = env.X_MAX - env.X_MIN;
        env.Y_SIZE = env.Y_MAX - env.Y_MIN;
    }

    bool copy(const agent_physical_t &src, agent_physical_t &dst) {
        dst.SIGHT_RANGE = src.SIGHT_RANGE;
        dst.SIGHT_ANGLE = src.SIGHT_ANGLE;
        dst.SIGHT_SIGMA = src.SIGHT_SIGMA;
        dst.RADIUS = src.RADIUS;
        dst.M = src.M;
        dst.D = src.D;
        dst.G = src.G;
        dst.U_MAX = src.U_MAX;
        dst.V_MAX = src.V_MAX;
        return true;
    }

    bool copy(const field_environment_t &src, field_environment_t &dst) {
        dst.X_MAX = src.X_MAX;
        dst.X_MIN = src.X_MIN;
        dst.Y_MAX = src.Y_MAX;
        dst.Y_MIN = src.Y_MIN;
        dst.X_SIZE = src.X_SIZE;
        dst.Y_SIZE = src.Y_SIZE;
        return true;
    }

    bool check_isnan(const std::vector<double> &_x) {

        bool ck_flg = true;
        for (int i = 0; i < (int)_x.size(); i++) {
            if (std::isnan(_x[i]) || std::isinf(_x[i]) || !std::isfinite(_x[i])) {
                std::cerr << "#error: isnan: "<<std::isnan(_x[i])<<", isfinite: "<<std::isinf(_x[i]);
                std:: cerr<<", isfinate: ["<<std::isfinite(_x[i])<<"]:  ";
                std::cerr<<" _x["<<i<<"]: ["<<_x[i]<<"]: nan or inf is detected. " << std::endl;
                ck_flg = false;
            }
        }
        if(!ck_flg) {
            std::cerr << "#error: _x: ["<<_x<<"]: nan or inf is detected. ";
            std::cerr << " @agentcore_config.h::check_isnan(const std::vector<double> &_x)" << std::endl;
        }
        return ck_flg;
    }

    bool check_isnan(const int s, const double *_x) {
        bool ck_flg = true;
        for (int i = 0; i < s; i++) {
            if (std::isnan(_x[i]) || std::isinf(_x[i]) || !std::isfinite(_x[i])) {
                std::cerr << "#error: isnan: "<<std::isnan(_x[i])<<", isinf: "<<std::isinf(_x[i]);
                std:: cerr<<", isfinite: ["<<std::isfinite(_x[i])<<"]:  ";
                std::cerr<<" _x["<<s<<"]: ["<<_x[i]<<"]: nan or inf is detected. " << std::endl;
                ck_flg = false;
            }
        }
        if(!ck_flg) {
            std::cerr << "#error: _x: nan or inf is detected. ";
            std::cerr << " @agentcore_config.h::check_isnan(const int s, const double *_x)" << std::endl;
        }
        return ck_flg;
    }

    bool check_isnan(const double _x) {
        bool ck_flg = true;
            if (std::isnan(_x) || std::isinf(_x) || !std::isfinite(_x)) {
                std::cerr << "#error: isnan: "<<std::isnan(_x)<<", isinf: "<<std::isinf(_x);
                std:: cerr<<", isfinite: ["<<std::isfinite(_x)<<"]:  ";
                std::cerr<<" _x: ["<<_x<<"]: nan or inf is detected. " << std::endl;
                ck_flg = false;
            }
        if(!ck_flg) {
            std::cerr << "#error: _x: ["<<_x<<"]: nan or inf is detected. ";
            std::cerr << " @agentcore_config.h::check_isnan(const double _x)" << std::endl;
        }
        return ck_flg;
    }
}

//-----------------------------
// for mathematical calculation

double g_rand_gauss(const double mean, const double std) {
    if(std==0.0) return mean;

    std::mt19937 engine(seed0());            // メルセンヌ・ツイスター法
    // std::minstd_rand0 engine(seed());    // 線形合同法
    // std::ranlux24_base engine(seed());   // キャリー付き減算法
    std::normal_distribution<> dist(mean, std);
    return dist(engine);
}

double g_rand(const double min, const double max) {
    std::mt19937 engine(seed1());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    std::uniform_real_distribution<> rand_real(min, max);        // [0, 99] 範囲の一様乱数
    return rand_real(engine);
}

template<class T>
double norm(const std::vector<T> &v2) {
    double n = 0;
    for (int i = 0; i < (int)v2.size(); i++) {
        n += v2[i] * v2[i];
    }
    return sqrt(n);
}

template<class T>
double normalize(std::vector<T> &v2) {
    double n = norm(v2);
    if(fabs(n) < 0.001) {
        return 0.0;
    }
    for (int i = 0; i < (int)v2.size(); i++) {
        v2[i] /= n;
    }
    return n;
}

// ベクトルaとbの仰角(elevation angle) [rad] を求める
double elev(const std::vector<double> &a, const std::vector<double> &b) {

    double elv = 0.0;
    double carg;
    if (norm(a) == 0) return 0.0;
    if (norm(b) == 0) return 0.0;
    carg = (a[0] * b[0] + a[1] * b[1]) / (norm(a) * norm(b));
    elv = acos(carg);
    return elv;
}


std::vector<double> &get_plor_vector_on_x(double rad, double R = 1.0) {
    static std::vector<double> v(2);
    v[0] = R * cos(rad);
    v[1] = R * sin(rad);
    return v;
}

std::vector<double> &get_plor_vector_on_y(double rad, double R = 1.0) {
    static std::vector<double> v(2);
    v[0] = -R * sin(rad);
    v[1] = R * cos(rad);
    return v;
}


double sigmoid(const double x, const double a, const double min, const double max) {
    double y;
    y = (max - min) / (1.0 + exp(-a * x)) + min;
    return y;
}

std::vector<double> &get_rot2(const double rad, const std::vector<double> &v2) {
    if (v2.size() != 2) {
        std::cerr << "#error: v2.size() != 2, @get_rot2()" << std::endl;
        exit(1);
    }
    static std::vector<double> ans(2);
    ans[0] = cos(rad) * v2[0] - sin(rad) * v2[1];
    ans[1] = sin(rad) * v2[0] + cos(rad) * v2[1];
    return ans;
}

// [0, 1] との仰角を返す
double elev_on_y(const std::vector<double> &a, bool minus = false) {
    double elv = 0.0;
    std::vector<double> yv(2);
    yv[0] = 0.0;
    yv[1] = 1.0;
    if(minus)   yv[1] = -1.0;
    double carg;
    if (a[0] == 0. && a[1] == 0.) return 0.;
    if (a[1] == 0.0) {
        if (a[0] < 0) elv = M_PI / 2;
        else elv = -M_PI / 2;
    } else if (a[0] == 0.0) {
        if (a[1] >= 0) elv = 0.0;
        else elv = M_PI;
    } else {
        carg = (a[0] * yv[0] + a[1] * yv[1]) / (norm(a) * norm(yv));
        elv = acos(carg);
    }
    // 右手系 なので
    if (a[0] > 0) elv = -elv;

    if (elv > M_PI)
        elv -= 2.0 * M_PI;
    if (elv < -M_PI)
        elv += 2.0 * M_PI;
    return elv;
}

// [1, 0] との仰角を返す
double elev_on_x(const std::vector<double> &a, bool minus = false) {
    double elv = 0.0;
    std::vector<double> xv(2);
    xv[0] = 1.0;
    if(minus)   xv[0] = -1.0;
    xv[1] = 0.0;
    double carg;
    if (a[0] == 0. && a[1] == 0.) return 0.;
    if (a[0] == 0.0) {
        if (a[1] < 0) elv = -M_PI / 2;
        else elv = M_PI / 2;
    } else if (a[1] == 0.0) {
        if (a[0] > 0) elv = 0.0;
        else elv = M_PI;
    } else {
        carg = (a[0] * xv[0] + a[1] * xv[1]) / (norm(a) * norm(xv));
        elv = acos(carg);
    }
    // 右手系 なので
    if (a[1] < 0) elv = -elv;
    if (elv > M_PI)
        elv -= 2.0 * M_PI;
    if (elv < -M_PI)
        elv += 2.0 * M_PI;

    return elv;
}



#endif //__AGENT_CORE_CONFIG_H__
