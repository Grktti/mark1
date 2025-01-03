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

    std::vector<double> &get_vect(const crlAgent &other) {
        static std::vector<double> vect(U_SIZE);
        get_toroidal_vector2(vect, other, 0.0);
        return vect;
    }

    // エージェントの位置をコンソールに出力する関数
    void print_position(int agent_id) const {
        std::vector<double> position = get_pos();
        std::cout << "Agent " << agent_id << " Position: (";
        for (size_t i = 0; i < position.size(); ++i) {
            std::cout << position[i];
            if (i < position.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ")" << std::endl;
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