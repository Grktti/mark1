/***************************************************************************
 * crlAgent.hpp
 *
 * Copyright (C) 2023 - Hiroshi IGARASHI
 * Aug. 22, 2023
 *****************************************************************************/

#ifndef CRL_AGENT_HPP
#define CRL_AGENT_HPP

#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <cmath>
#include "crlAgentCore.hpp"

class crlAgent : public crlAgentCore {
    double m_field_max;
public:
    crlAgent() : crlAgentCore() {
        //    std::cout << "crlAgent constructor" << std::endl;
    }

    bool init(int id, int type, double field_max) {
        m_field_max = field_max;
        //std::cout << "#debug: id: " << id << ", type: " << type;
        //std::cout << ", field_max: " << field_max;
        //std::cout << " @crlAgent::init()" << std::endl;
        return crlAgentCore::init(id, type, field_max, -field_max, field_max, -field_max);
    }
    // -range から +range の範囲にランダムにエージェントを配置
    bool set_pos_random(double range) {
        m_stat[0] = g_rand( -range, range);
        m_stat[1] = g_rand( -range, range);
        return true;
    }

    // エージェントのランダムウォーク
    const std::vector<double> & get_random_walk_gauss(double ave, double sigma) {
        static std::vector<double> u(U_SIZE);
        u[0] = g_rand_gauss(ave, sigma);
        u[1] = g_rand_gauss(ave, sigma);
        return u;
    }
    // エージェントのランダムウォーク
    const std::vector<double> & get_random_walk(double range) {
        static std::vector<double> u(U_SIZE);
        u[0] = g_rand(-range, range);
        u[1] = g_rand(-range, range);
        return u;
    }

    //ボイドモデルの作成
    const std::vector<double> & get_boid_model(const std::vector<crlAgent> &others, double range) {
        static std::vector<double> u(U_SIZE, 0.0);
        std::vector<double> separation(U_SIZE, 0.0);
        std::vector<double> alignment(U_SIZE, 0.0);
        std::vector<double> cohesion(U_SIZE, 0.0);
        std::vector<double> repulsion(U_SIZE, 0.0);
        int count = 0;
        double k1 = 1.3;
        double k2 = 1.0;
        double k3 = 1.8;
        double k4 = 4.0;

        for (const auto& other : others) {
            if (is_same(other)) continue;  // 自分自身の場合はスキップ
            double dist = get_dist(other); // 他のエージェントまでの距離を計算
            if (dist < range) {
                // 分離、整列、凝集の計算
                //分離
                std::vector<double> diff = get_vect(other);  // 他のエージェントへの位置ベクトルを取得
                for (auto& d : diff) d *= -1.0;  // 反転して分離の方向に設定
                for (int i = 0; i < U_SIZE; ++i) separation[i] += diff[i] / (dist * dist);  // 距離の二乗で割って影響を調整
                // 整列 (Alignment)
                std::vector<double> other_vel = other.get_velocity();  // 他のエージェントの速度を取得
                for (int i = 0; i < U_SIZE; ++i) alignment[i] += other_vel[i];
                // 凝集 (Cohesion)
                std::vector<double> other_pos = other.get_position();  // 他のエージェントの位置を取得
                for (int i = 0; i < U_SIZE; ++i) cohesion[i] += other_pos[i];




            }
        }

    }

    int get_nearest_agent_id(const std::vector<crlAgent> &others) {
        double dist;
        double min_dist = 100000.0;
        int nearest_agent_id = -1;
        for(int n=0; n<others.size(); n++) {
            if(is_same(others[n])) continue;
            dist = get_dist(others[n]);
            if(dist < min_dist) {
                min_dist = dist;
                nearest_agent_id = others[n].get_id();
            }
        }
        return nearest_agent_id;
    }

    bool drive(const std::vector<double> &u, const std::vector<crlAgent> &others, double smp) {
        std::vector<double> stat;
        get_stat(stat);
        crlAgentCore::drive_core(stat, u, smp);
        if(!is_collision(others)) {
            set_stat(stat);
            return true;
        }else{
            //std::cerr << "#warning["<<label()<<"]: collision detected. @crlAgent::drive()" << std::endl;
            return false;
        }
    }

    // エージェント間の距離を計算
    double get_dist(const crlAgent &other) {
        double dist;
        dist = get_toroidal_dist2_with_radius(other, 0.0);
        return dist;
    }
    // エージェントの速度ベクトルを取得
    std::vector<double> get_velocity() const {
        std::vector<double> velocity(U_SIZE);
        get_veloc(velocity);
        return velocity;
    }
    // エージェントの位置ベクトルを取得
    std::vector<double> get_position() const {
        std::vector<double> position(U_SIZE);
        get_pos(position);
        return position;
    }

    //トロイダルベクトルを取得
    std::vector<double> &get_vect(const crlAgent &other) {
        static std::vector<double> vect(U_SIZE);
        get_toroidal_vector2(vect, other, 0.0);
        return vect;
    }

    // 衝突チェック
    bool is_collision(const std::vector<crlAgent> &others, double min_dist=0.1)  {

        for(int n=0; n<others.size(); n++) {
            if(is_same(others[n])) continue;
            if(get_dist(others[n]) < min_dist) {
                // 衝突回避方向へのベクトル
                std::vector<double> repulsive_vect(U_SIZE);
                repulsive_vect = -1.0 * get_vect(others[n]);
                // repulsive_vect を正規化
                normalize(repulsive_vect);
                // 衝突回避方向への速度をセット
                set_veloc(repulsive_vect);
                add_pos(repulsive_vect);
                return true;
            }
        }
        return false;
    }

    bool add_pos(const std::vector<double> &dlt) {
        std::vector<double> stat;
        get_stat(stat);
        stat[0] += dlt[0];
        stat[1] += dlt[1];
        set_stat(stat);
        return true;
    }


};

#endif // CRL_AGENT_HPP