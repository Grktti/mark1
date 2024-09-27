/***************************************************************************
 * agentCore.hpp
 *
 * Copyright (C) 2023 - Hiroshi IGARASHI
 * Aug. 22, 2023
 *****************************************************************************/

#ifndef AGENT_CORE_HPP
#define AGENT_CORE_HPP

#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <cmath>
#include "crlAgentCore_config.h"

namespace ac = agentcore;


class crlAgentCore {
private:
    typedef std::vector<double> vecd;


protected:
    double m_stat[STAT_SIZE]; // 現在の位置・速度・加速度・操作入力 (x, y, dx, dy, ddx, ddy, ux, uy) /in R^8
    bool m_init_flg; // 初期化したら true
    int m_id, m_type;
    std::string m_label; // type:id
    ac::agent_physical_t m_pys; // 物理パラメータ
    ac::field_environment_t m_env;

public:
    crlAgentCore() {
        m_init_flg = false;
        m_label = "NULL";
    };

    ~crlAgentCore() {
        m_label.clear();

    };

    // コピーコンストラクタ
    crlAgentCore(const crlAgentCore &ac) {
        if (!ac.check_core()) {
            std::cerr << "#error[" << ac.label() << "]: .check_core() returns false. ";
            std::cerr << "@agentCore::agentCore() --copy constructor--" << std::endl;
            exit(1);
        }
        m_id = ac.m_id;
        m_type = ac.m_type;
        m_init_flg = ac.m_init_flg;
        for (int i = 0; i < STAT_SIZE; i++)
            m_stat[i] = ac.m_stat[i];
        m_label = ac.m_label;
        copy(ac.m_pys, m_pys);
        copy(ac.m_env, m_env);
    };

    bool set(const crlAgentCore &ac) {
        if (!ac.check_core()) {
            std::cerr << "#error[" << m_label << "]: [" << ac.label() << "].check_core() returns false. ";
            std::cerr << "@agentCore::set()" << std::endl;
            return false;
        }
        m_id = ac.m_id;
        m_type = ac.m_type;
        m_init_flg = ac.m_init_flg;
        for (int i = 0; i < STAT_SIZE; i++)
            m_stat[i] = ac.m_stat[i];
        m_label = ac.m_label;
        copy(ac.m_pys, m_pys);
        copy(ac.m_env, m_env);
        return check_core();
    };

    bool init(int id, int type, double field_x_max, double field_x_min, double field_y_max, double field_y_min) {
        if (id < 0) {
            std::cerr << "#error[" << m_label << "]: id is negative! ";
            std::cerr << "@agentCore::init()" << std::endl;
            exit(1);
        }
        if (field_x_max < field_x_min || field_y_max < field_y_min) {
            std::cerr << "#error[" << m_label << "]: field_x_max < field_x_min or field_y_max < field_y_min! ";
            std::cerr << "@agentCore::init()" << std::endl;
            exit(1);
        }
        m_env.X_MAX = field_x_max;
        m_env.X_MIN = field_x_min;
        m_env.Y_MAX = field_y_max;
        m_env.Y_MIN = field_y_min;
        m_env.X_SIZE = field_x_max - field_x_min;
        m_env.Y_SIZE = field_y_max - field_y_min;
        m_id = id;
        m_type = type;
        m_pys.U_MAX = DEFAULT_U_MAX;
        m_pys.V_MAX = DEFAULT_V_MAX;
        m_stat[0] = g_rand(m_env.X_MIN * 0.85, m_env.X_MAX * 0.85);
        m_stat[1] = g_rand(m_env.Y_MIN * 0.85, m_env.Y_MAX * 0.85);
        for (int i = 2; i < STAT_SIZE; i++) {
            m_stat[i] = g_rand_gauss(0.0, 1.0);
        }

        // TODO: m_pys が初期化されてしまう
        init_physical_param(m_pys);
        m_init_flg = true;
        return true;
    };

    bool set_label(const std::string &name) {
        m_label = name;
        return true;
    }

    double get_pos_x() const {
        return m_stat[0];
    }

    double get_pos_y() const {
        return m_stat[1];
    }

    double get_veloc_x() const {
        return m_stat[2];
    }

    double get_veloc_y() const {
        return m_stat[3];
    }

    double get_accel_x() const {
        return m_stat[4];
    }

    double get_accel_y() const {
        return m_stat[5];
    }

    double get_u_x() const {
        return m_stat[6];
    }

    double get_u_y() const {
        return m_stat[7];
    }

    double get_stat(int id) const {
        if (id < 0 || id >= STAT_SIZE) {
            std::cerr << "#error[" << m_label << "]: id: " << id << " is out of range! ";
            std::cerr << "@agentCore::get_stat()" << std::endl;
            exit(1);
        }
        return m_stat[id];
    }

    const std::vector<double> &get_stat_vect() const {
        static std::vector<double> stat;
        stat.resize(STAT_SIZE);
        if (ac::check_isnan(STAT_SIZE, m_stat)) {
            for (int i = 0; i < STAT_SIZE; i++) {
                stat[i] = m_stat[i];
            }
        } else {
            std::cerr << "#error[" << m_label << "]: m_stat: [" << m_stat << "] is nan or inf! ";
            std::cerr << "@agentCore::get_stat_vect()" << std::endl;
            exit(1);
        }
        return stat;
    }

    bool get_stat(double *stat_) const {
        if (!ac::check_isnan(STAT_SIZE, m_stat)) {
            std::cerr << "#error[" << m_label << "]: m_stat: [" << m_stat << "] is nan or inf! ";
            exit(1);
        }
        for (int i = 0; i < STAT_SIZE; i++)
            stat_[i] = m_stat[i];

        return true;
    };

    bool get_stat(std::vector<double> &stv) const {
        if (stv.size() != STAT_SIZE) {
            stv.clear();
            stv.assign(STAT_SIZE, 0.0);
        }
        if (ac::check_isnan(STAT_SIZE, m_stat)) {
            for (int i = 0; i < STAT_SIZE; i++) {
                stv[i] = m_stat[i];
            }
        } else {
            std::cerr << "#error[" << label() << "]: m_stat includes nan! ";
            std::cerr << "@agentCore::get_stat()" << std::endl;
            return false;
        }
        return true;
    };

    bool set_stat(const std::vector<double> &stv) {

        if (ac::check_isnan(stv)) {
            for (int i = 0; i < STAT_SIZE; i++) {
                m_stat[i] = stv[i];
            }
        } else {
            std::cerr << "#error[" << label() << "]: stv: [" << stv << "] is nan or inf! ";
            std::cerr << "@agentCore::set_stat()" << std::endl;
            return false;
        }

        return true;
    }

    bool set_stat(const double *st) {

        if (!ac::check_isnan(STAT_SIZE, st)) {
            std::cerr << "#error[" << label() << "]: st: [" << st << "] st.size(): " << STAT_SIZE;
            std::cerr << " or check_isnan(st) error, agentCore::set_stat()" << std::endl;
            return false;
        }
        for (int i = 0; i < STAT_SIZE; i++) {
            m_stat[i] = st[i];
        }
        return true;
    }

    bool check_stat(const std::vector<double> &stat) const {

        if (stat.size() != STAT_SIZE || !ac::check_isnan(stat)) {
            std::cerr << "#error[" << m_label << "]: stat: [" << stat << "] stat.size(): " << stat.size();
            std::cerr << " or check_isnan(stat) error, agentCore::check_stat()" << std::endl;
            return false;
        }
        return true;
    }

    bool get_pos_now(vecd &pos_) const {
        pos_.resize(U_SIZE);
        pos_[0] = m_stat[0];
        pos_[1] = m_stat[1];
        return true;
    }

    const std::vector<double> &get_pos() const {

        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false. ";
            std::cerr << "@agentCore::get_pos()" << std::endl;
            exit(1);
        }
        static vecd pos(2, 0.0);
        pos[0] = m_stat[0];
        pos[1] = m_stat[1];
        return pos;
    };


    static const std::vector<double> &get_pos(const std::vector<double> &_stat) {
        static std::vector<double> pos(2, 0.0);
        pos[0] = _stat[0];
        pos[1] = _stat[1];
        return pos;
    };

    const std::vector<double> &get_pos(const double *_stat) const {
        static std::vector<double> pos(2, 0.0);
        pos[0] = _stat[0];
        pos[1] = _stat[1];
        return pos;
    };


    bool set_pos(const std::vector<double> &x) {
        if ((int) x.size() != U_SIZE) {
            std::cerr << "#warning[" << m_label << "]: x: [" << x << "] x.size(): " << x.size();
            std::cerr << " : not " << U_SIZE << " @agentCore::set_pos()" << std::endl;
        }
        if (!ac::check_isnan(x)) {
            std::cerr << "#error[" << label() << "]: check_isnan(x) error. ";
            std::cerr << "@agentCore::set_pos()" << std::endl;
            return false;
        }
        m_stat[0] = x[0];
        m_stat[1] = x[1];
        return true;
    };

    bool set_pos(const double *x) {
        m_stat[0] = x[0];
        m_stat[1] = x[1];
        return true;
    };

    const std::vector<double> &get_veloc() const {
        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false. ";
            std::cerr << "@agentCore::get_veloc()" << std::endl;
            exit(1);
        }
        static std::vector<double> accel(2);
        accel[0] = m_stat[2];
        accel[1] = m_stat[3];
        return accel;
    };

    const std::vector<double> &get_veloc(const std::vector<double> &_stat) const {
        static std::vector<double> vel(2, 0.0);
        vel[0] = _stat[2];
        vel[1] = _stat[3];
        return vel;
    };

    bool set_veloc(const std::vector<double> &v) {

        if (v.size() != U_SIZE || !ac::check_isnan(v)) {
            std::cerr << "#error[" << m_label << "]: v: [" << v << "] x.size(): " << v.size();
            std::cerr << " or check_isnan(v) error, agentCore::set_veloc()" << std::endl;
            return false;
        }
        m_stat[2] = v[0];
        m_stat[3] = v[1];
        return true;
    };

    bool set_veloc(const double *v) {
        m_stat[2] = v[0];
        m_stat[3] = v[1];
        return true;
    };

    const std::vector<double> &get_accel() const {
        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false. ";
            std::cerr << "@agentCore::get_force()" << std::endl;
            exit(1);
        }
        static std::vector<double> a(2);
        a[0] = m_stat[4];
        a[1] = m_stat[5];
        return a;
    };

    bool get_accel(double *acc) const {
        acc[0] = m_stat[4];
        acc[1] = m_stat[5];
        return true;
    };

    bool set_accel(const std::vector<double> &a) {
        if (a.size() != STAT_SIZE || !ac::check_isnan(a)) {
            std::cerr << "#error[" << m_label << "]: near: [" << a << "] near.size(): " << a.size();
            std::cerr << " or check_isnan(x) error, agentCore::set_accel()" << std::endl;
            return false;
        }
        m_stat[4] = a[0];
        m_stat[5] = a[1];
        return true;
    };

    bool set_accel(const double *a) {
        m_stat[4] = a[0];
        m_stat[5] = a[1];
        return true;
    };

    bool set_force(const std::vector<double> &u) {
        if (u.size() != 2 || !ac::check_isnan(u)) {
            std::cerr << "#error[" << m_label << "]: cog: [" << u << "] x.size(): " << u.size();
            std::cerr << " or check_isnan(cog) error, agentCore::set_force()" << std::endl;
            return false;
        }
        m_stat[6] = u[0];
        m_stat[7] = u[1];
        return true;
    };

    const std::vector<double> &get_force() const {
        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core returns false. @agentCore::get_force()" << std::endl;
            exit(1);
        }
        static std::vector<double> u(2);
        u[0] = m_stat[6];
        u[1] = m_stat[7];
        return u;
    };

    bool get_force(double *f) const {
        f[0] = m_stat[6];
        f[1] = m_stat[7];
        return true;
    };


    bool set_physical_parameters(const ac::agent_physical_t &ap) {

        if (ap.SIGHT_RANGE < 0.0 || ap.RADIUS < 0.0 || ap.M < 0.0 || ap.D < 0.0 || ap.G < 0.0 || ap.U_MAX < 0.0 ||
            ap.V_MAX < 0.0) {
            std::cerr << "#error[" << label() << "]: ap.SIGHT_RANGE < 0.0 || ap.RADIUS < 0.0 || ";
            std::cerr << "ap.M < 0.0 || ap.D < 0.0 || ap.G < 0.0 || ap.U_MAX < 0.0 || ap.V_MAX < 0.0";
            std::cerr << " @agentCore::set_physical_parameters()" << std::endl;
            return false;
        }
        m_pys.SIGHT_RANGE = ap.SIGHT_RANGE;
        m_pys.RADIUS = ap.RADIUS;
        m_pys.M = ap.M;
        m_pys.D = ap.D;
        m_pys.G = ap.G;
        m_pys.U_MAX = ap.U_MAX;
        m_pys.V_MAX = ap.V_MAX;
        return true;
    };

    bool get_physical_parameters(ac::agent_physical_t &ap_) const {
        ap_.SIGHT_RANGE = m_pys.SIGHT_RANGE;
        ap_.RADIUS = m_pys.RADIUS;
        ap_.M = m_pys.M;
        ap_.D = m_pys.D;
        ap_.G = m_pys.G;
        ap_.U_MAX = m_pys.U_MAX;
        ap_.V_MAX = m_pys.V_MAX;
        return true;
    }

    bool set_environment_parameters(const ac::field_environment_t &fe) {
        if (fe.X_MAX < fe.X_MIN || fe.Y_MAX < fe.Y_MIN) {
            std::cerr << "#error[" << label() << "]: fe.X_MAX < fe.X_MIN || fe.Y_MAX < fe.Y_MIN ";
            std::cerr << " @agentCore::set_environment_parameters()" << std::endl;
            return false;
        }
        m_env.X_MAX = fe.X_MAX;
        m_env.X_MIN = fe.X_MIN;
        m_env.Y_MAX = fe.Y_MAX;
        m_env.Y_MIN = fe.Y_MIN;
        m_env.X_SIZE = fe.X_MAX - fe.X_MIN;
        m_env.Y_SIZE = fe.Y_MAX - fe.Y_MIN;
        return true;
    };

    bool get_environment_parameters(ac::field_environment_t &fe_) const {
        fe_.X_MAX = m_env.X_MAX;
        fe_.X_MIN = m_env.X_MIN;
        fe_.Y_MAX = m_env.Y_MAX;
        fe_.Y_MIN = m_env.Y_MIN;
        fe_.X_SIZE = m_env.X_SIZE;
        fe_.Y_SIZE = m_env.Y_SIZE;
        return true;
    }

    bool set_u_max(const double u_max) {
        if (u_max < 0.0) {
            std::cerr << "#error[" << label() << "]: U_MAX < 0.0: (" << u_max << ") ";
            std::cerr << "@agentCore::set_u_max()" << std::endl;
            return false;
        }
        m_pys.U_MAX = u_max;
        return true;
    };

    double get_u_max() const {
        return m_pys.U_MAX;
    };

    bool set_v_max(const double v_max) {
        if (v_max < 0.0) {
            std::cerr << "#error[" << label() << "]: V_MAX < 0.0: (" << v_max << ")";
            std::cerr << " @agentCore::set_v_max()" << std::endl;
            return false;
        }
        m_pys.V_MAX = v_max;
        return true;
    };

    double get_v_max() const {
        return m_pys.V_MAX;
    };

    const std::string &label() const {
        return m_label;
    };

    bool name(const std::string &n) {
        m_label = n;
        return true;
    }

    bool set_sight(const double sight) {
        if (sight < 0) {
            std::cerr << "#error[" << label() << "]: SIGHT_RANGE < 0: (" << sight << ") ";
            std::cerr << "@agentCore::set_sight()" << std::endl;
            return false;
        }
        m_pys.SIGHT_RANGE = sight;
        return true;
    };

    double get_sight_range() const {
        return m_pys.SIGHT_RANGE;
    }

    double get_sight_angle() const {
        return m_pys.SIGHT_ANGLE;
    }

    double get_sight_sigma() const {
        return m_pys.SIGHT_SIGMA;
    }

    bool set_M(const double M) {
        if (M < 0) {
            std::cerr << "#error[" << label() << "]: M < 0: (" << M << ") ";
            std::cerr << "@agentCore::set_M()" << std::endl;
            return false;
        }
        m_pys.M = M;
        return true;
    }

    bool set_D(const double D) {
        if (D < 0) {
            std::cerr << "#error[" << label() << "]: D < 0: (" << D << ") ";
            std::cerr << "@agentCore::set_D()" << std::endl;
            return false;
        }
        m_pys.D = D;
        return true;
    }

    bool set_G(const double G) {
        if (G < 0) {
            std::cerr << "#error[" << label() << "]: G < 0: (" << G << ") ";
            std::cerr << "@agentCore::set_G()" << std::endl;
            return false;
        }
        m_pys.G = G;
        return true;
    }

    bool set_input_gain(const double G) {
        if (G < 0) {
            std::cerr << "#error[" << label() << "]: G < 0: (" << G << ") ";
            std::cerr << "@agentCore::set_input_gain()" << std::endl;
            return false;
        }
        m_pys.G = G;
        return true;
    }

    double get_M() const {
        return m_pys.M;
    }

    double get_D() const {
        return m_pys.D;
    }

    double get_input_gain() const {
        return m_pys.G;
    }

    double get_radius() const {
        return m_pys.RADIUS;
    }

    int get_id() const {
        return m_id;
    };

    int get_type() const {
        return m_type;
    };

    bool is_collision_occurred(const crlAgentCore &a, const double eps) const {
        if (is_same(a)) {
            //std::cerr << "#warning[" << label() << "]: same agent_const is checked! ";
            //std::cerr << "@agentCore::is_collision_occurred()" << std::endl;
            return false;
        }
        double d = get_toroidal_dist2_with_radius(a, 0.0) - eps;
        if (d < 0)
            return true;
        else
            return false;
    };

    bool is_insight(const crlAgentCore &a) const {
        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false.";
            std::cerr << " @agentCore::is_insight() " << std::endl;
            exit(1);
        }
        if (is_same(a)) {
            std::cerr << "#warning[" << label() << "]: same agent_const with [" << a.label() << "]";
            std::cerr << " @agentCore::is_insight()" << std::endl;
            return false;
        }
        double d = get_toroidal_dist2_with_radius(a, 0.0);
        if (d < m_pys.SIGHT_RANGE)
            return true;
        else
            return false;
    }

    double get_sight_angle_elev_deg(const crlAgentCore &a) const {

        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false.";
            std::cerr << " @agentCore::get_sight_angle_elev_deg() " << std::endl;
            exit(1);
        }
        if (is_same(a)) {
            std::cerr << "#warning[" << label() << "]: same agent_const with [" << a.label() << "]";
            std::cerr << " @agentCore::get_sight_angle_elev_deg()" << std::endl;
            return 0.0;
        }
        vecd dlt(2, 0.0);
        get_toroidal_vector2(dlt, get_pos(), a.get_pos(), m_env.X_MIN, m_env.X_MAX, m_env.Y_MIN, m_env.Y_MAX,
                             a.get_sight_sigma());
        double elev_deg = atan2(dlt[1], dlt[0]) * 180.0 / M_PI;
        double velc_deg = atan2(get_veloc_x(), get_veloc_y()) * 180.0 / M_PI;

        return elev_deg - velc_deg;
    }

    double get_sight_angle_elev_deg_on_map(const int map_x, const int map_y) const {

        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false.";
            std::cerr << " @agentCore::get_sight_angle_elev_deg_on_map() " << std::endl;
            exit(1);
        }
        vecd tpos(2, 0.0);
        tpos[0] = m_env.X_MIN + (double) map_x;
        tpos[1] = m_env.X_MIN + (double) map_y;
        vecd dlt;
        get_toroidal_vector2(dlt, get_pos(), tpos, m_env.X_MIN, m_env.X_MAX, m_env.Y_MIN, m_env.Y_MAX, 0.0);
        double elev_deg = atan2(dlt[1], dlt[0]) * 180.0 / M_PI;
        double velc_deg = atan2(get_veloc_x(), get_veloc_y()) * 180.0 / M_PI;
        double dlt_deg = elev_deg - velc_deg;
        //std::cout << "#debug: elev_deg: " << elev_deg << ", velc_deg: " << velc_deg << ", dlt_deg: " << dlt_deg << std::endl;
        return fabs(dlt_deg);
    }


    // トロイダルベクトル（2次元）から距離を計算
    double get_toroidal_dist2(const crlAgentCore &target, double sigma) const {
        vecd dlt;
        get_toroidal_vector2(dlt, target, sigma);
        return norm(dlt);
    };

    // トロイダルベクトル（2次元）から距離を計算
    double get_toroidal_dist2_with_radius(const crlAgentCore &target, double sigma) const {
        vecd dlt;
        get_toroidal_vector2(dlt, target, sigma);
        return norm(dlt) - get_radius() - target.get_radius();
    };

    bool get_toroidal_vector2(vecd &dlt_vect, const crlAgentCore &target, double sigma) const {

        const vecd p0(get_pos());
        const vecd p1(target.get_pos());
        bool ck = get_toroidal_vector2(dlt_vect, p0, p1, m_env.X_MIN, m_env.X_MAX, m_env.Y_MIN, m_env.Y_MAX, sigma);
        //std::cout << "#debug:p0["<<label()<<"]: [" << p0 << "], p1["<<target.label()<<"]: [" << p1 << "], dlt_vect: [" << dlt_vect << "] ";
        //std::cout << "@agentCore::get_toroidal_vector2()" << std::endl;
        if (!ck) {
            std::cerr << "#error[" << label() << "]: get_toroidal_vector2() returns false. ";
            std::cerr << "@agentCore::set_input_gain() @agentCore::get_toroidal_vector2()" << std::endl;
        }
        //std::cout << "#debug: dlt_vect: " << dlt_vect << " @agentCore::get_toroidal_vector2()" << std::endl;
        return ck;
    }

    // トロイダル距離（2次元）を計算 [x, y] を dlt_x. dlt_y だけ動かした場合（偏微分計算用）
    // x1 は x1[0]とx1[1]の2次元ベクトルのみ使用
    double get_toroidal_dist2_with_dlt(const crlAgentCore &trg, double dlt_x, double dlt_y) const {
        double d = 0.0;
        crlAgentCore trg_ = trg;
        trg_.m_stat[0] += dlt_x;
        trg_.m_stat[1] += dlt_y;
        d = get_toroidal_dist2_with_radius(trg_, get_sight_sigma());
        return d; // L1
    };

    bool is_same(const crlAgentCore &a) const {
        if (!check_core()) {
            std::cerr << "#error[" << label() << "]: check_core() returns false. @agentCore::is_same() " << std::endl;
            exit(1);
        }
        if (!a.check_core()) {
            std::cerr << "#error[" << label() << "]: [" << a.label();
            std::cerr << "].check_core() returns false. @agentCore::is_same() ";
            std::cerr << std::endl;
            exit(1);
        }
        if (m_id == a.m_id && m_type == a.m_type) {
            return true;
        } else {
            return false;
        }
    }

    void debug() const {
        //std::cout << "#debug: agentCore::debug() [id: " << m_id << ", tyoe: " << m_type << "]" << std::endl;
        std::cout << "#debug[" << m_label << "]: m_stat [";
        for (int i = 0; i < STAT_SIZE; i++) {
            std::cout.precision(3);
            std::cout << " " << m_stat[i];
        }
        std::cout << "] " << std::endl;
        std::cout << "#debug[" << label() << "]: m_u[" << get_force() << "] ";
        std::cout << "@agentCore::debug()" << std::endl;
    };

protected:

    bool check_core() const {

        bool ck_flg = true;
        if (!m_init_flg) {
            std::cerr << "#error[" << label() << "]: m_init_flg = false. ";
            std::cerr << "@agentCore::check_core()" << std::endl;
            ck_flg = false;
        }
        if (!ac::check_isnan((int) STAT_SIZE, (double *) m_stat)) {
            std::cerr << "#error[" << label() << "]: m_stat includes nan! ";
            std::cerr << "@agentCore::check_core()" << std::endl;
            ck_flg = false;
        }
        return ck_flg;
    }

    bool drive_core(std::vector<double> &stat, const std::vector<double> &u_final, const double smpl_time) {

        std::vector<double> u(U_SIZE, 0.0);
        std::vector<double> v(U_SIZE, 0.0);

        if ((int) u_final.size() != U_SIZE || !ac::check_isnan(u_final)) {
            std::cerr << "#error[" << label() << "]: u_final includes nan! ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            return false;
        }
        stat.resize(STAT_SIZE, 0.0);
        for (int i = 0; i < STAT_SIZE; i++) {
            stat[i] = m_stat[i];
        }
        u = u_final;
        double u_max = m_pys.U_MAX;
        double un = normalize(u);
        if (un > u_max) {
            un = u_max;
        }
        u[0] = un * u[0];
        u[1] = un * u[1];

        stat[6] = u[0];
        stat[7] = u[1];
        if (!std::isfinite(stat[6]) || !std::isfinite(stat[7])) {
            std::cerr << "#error[" << label() << "]: [stat[6], stat[7]]: ";
            std::cerr << "[" << stat[6] << ", " << stat[7] << "] includes nan or inf. ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            stat[6] = 0.0;
            stat[7] = 0.0;

        }
        stat[4] = (1.0 / m_pys.M) * (-m_pys.D * m_stat[2] + m_pys.G * stat[6]); // ddx
        stat[5] = (1.0 / m_pys.M) * (-m_pys.D * m_stat[3] + m_pys.G * stat[7]); // ddy
        if (!std::isfinite(stat[4]) || !std::isfinite(stat[5])) {
            std::cerr << "#error[" << label() << "]: [stat[4], stat[5]]: ";
            std::cerr << "[" << stat[4] << ", " << stat[5] << "] includes nan or inf. ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            std::cerr << "#error[" << label() << "] debug: cog: [" << stat[6] << ", " << stat[7] << "] ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            std::cerr << "#error[" << label() << "] debug: u_final: [" << u_final << "] ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            return false;
        }

        stat[2] += stat[4] * smpl_time; // dx
        stat[3] += stat[5] * smpl_time; // dy

        // 速度のMAX値セット
        v[0] = stat[2];
        v[1] = stat[3];
        double v_max = m_pys.V_MAX;
        double vn = normalize(v);
        if (vn > v_max) {
            vn = v_max;
        }
        v[0] = vn * v[0];
        v[1] = vn * v[1];
        stat[2] = v[0];
        stat[3] = v[1];


        stat[0] += stat[2] * smpl_time; // x
        stat[1] += stat[3] * smpl_time; // y
        modify_into_toroidal(stat);
        set_stat(stat);
        /*
        if(get_type()==TYPE_SHEEP && get_id() == 0) {
            std::cout << "#debug[" << label() << "]: " << std::setprecision(2) << std::fixed << std::showpos;
            std::cout << "[M, D]: [" << m_pys.M << ", " << m_pys.D << "], ";
            std::cout << "cog: [" << stat[6] << ", " << stat[7] << "], ";
            std::cout << "dx: [" << stat[2] << ", " << stat[3] << "] ";
            std::cout << "ddx: [" << stat[4] << ", " << stat[5] << "] ";
            std::cout << "@agentCore::drive_core()" << std::endl;
        }*/

        if (!ac::check_isnan(stat)) {
            std::cerr << "#error[" << m_label << "]: m_stat is nan finite. ";
            std::cerr << "@agentCore::drive_core()" << std::endl;
            return false;
        }
        return true;
    };

    bool modify_into_toroidal(std::vector<double> &x_) const {
        if (x_[0] > m_env.X_MAX)
            x_[0] -= (m_env.X_MAX - m_env.X_MIN);
        else if (x_[0] < m_env.X_MIN)
            x_[0] += (m_env.X_MAX - m_env.X_MIN);
        if (x_[1] > m_env.Y_MAX)
            x_[1] -= (m_env.Y_MAX - m_env.Y_MIN);
        else if (x_[1] < m_env.Y_MIN)
            x_[1] += (m_env.Y_MAX - m_env.Y_MIN);
        return true;
    };

    bool sat_vect2(std::vector<double> &v_, const double max) const {
        if ((int) v_.size() != 2) {
            std::cerr << "#error[" << m_label << "]: v_.size() != 2. ";
            std::cerr << "@agentCore::sat_vect2()" << std::endl;
            return false;
        }
        if (!ac::check_isnan(v_)) {
            std::cerr << "#error[" << m_label << "]: v_ includes nan. ";
            std::cerr << "@agentCore::sat_vect2()" << std::endl;
            return false;
        }
        double _nv = normalize(v_);
        if (_nv > -0.001 && _nv < 0.001) {
            v_[0] = 0.0;
            v_[1] = 0.0;
            return true;
        }
        if (_nv > max) {
            v_[0] = v_[0] * max;
            v_[1] = v_[1] * max;
        }
        if (!ac::check_isnan(v_)) {
            std::cerr << "#error[" << m_label << "]: v_ includes nan. _nv: ";
            std::cerr << _nv << ", max: " << max << " @agentCore::sat_vect2()" << std::endl;
            return false;
        }
        return true;
    };

private:

    // トロイダルベクトル（2次元）を計算 [x2 - x1] を返す
    bool get_toroidal_vector2(vecd &dlt_vect, const vecd &x1, const vecd &x2, double x_min, double x_max, double y_min,
                              double y_max, double sight_s) const {

        dlt_vect.resize(U_SIZE, 0.0);
        double x_size = x_max - x_min;
        double y_size = y_max - y_min;
        vecd x1_(2, 0.0);
        vecd x2_(2, 0.0);

        //std::cout << "#debug: x1: [" << x1 << "], x2: [" << x2 << "] @get_toroidal_vector2()" << std::endl;
        for (int i = 0; i < U_SIZE; i++) {
            x1_[i] = x1[i];
            x2_[i] = g_rand_gauss(x2[i], sight_s);
        }
        for (int i = 0; i < U_SIZE; i++) {
            dlt_vect[i] = x2_[i] - x1_[i];
        }

        //std::cout << "#debug: x1_: [" << x1_ << "], x2_: [" << x2_ << "], dlt_vect: [" << dlt_vect << "] @get_toroidal_vector2()" << std::endl;
        double d0 = norm(dlt_vect);
        if (d0 < 0.5 * sqrt(x_size * x_size + y_size * y_size)) {
            return ac::check_isnan(dlt_vect);
        }


        double _d;
        x2_ = x2;
        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1];
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0];
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0];
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1];
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        return ac::check_isnan(dlt_vect);
    }

    // トロイダルベクトル（2次元）を計算 [x2 - x1] を返す
    const vecd &
    toroidal_vector2(const vecd &x1, const vecd &x2, const double x_min, const double x_max, const double y_min,
                     const double y_max, double sight_s) const {

        static vecd dlt_vect(2, 0.0);
        double x_size = x_max - x_min;
        double y_size = y_max - y_min;

        for (int i = 0; i < U_SIZE; i++) {
            dlt_vect[i] = g_rand_gauss(x2[i], sight_s) - x1[i];
        }
        //std::cout << "#debug[" << label() << "]: x1: ["<<x1<<"], x2: ["<<x2<<"], dlt_vect: [" << dlt_vect << "] ";
        //std::cout << "@toroidal_vector2()" << std::endl;
        double d0 = norm(dlt_vect);
        if (d0 < 0.5 * sqrt(x_size * x_size + y_size * y_size)) return dlt_vect;

        vecd x1_(2, 0.0);
        vecd x2_(2, 0.0);
        double _d;
        x2_ = x2;

        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1];
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] + x_size;
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0];
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0];
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1] + y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1];
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        x1_[0] = x1[0] - x_size;
        x1_[1] = x1[1] - y_size;
        if ((_d = norm(x2_ - x1_)) < d0) {
            d0 = _d;
            dlt_vect = x2_ - x1_;
        }
        return dlt_vect;
    }
};

#endif // AGENT_CORE_HPP