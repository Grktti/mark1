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
#include "crlAgentMap.hpp"

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

    //エージェントのボイドモデル
    const std::vector<double> & get_boid_model(const std::vector<crlAgent> &others, double range, agentCoreMap& agentMap){
        static std::vector<double> u(U_SIZE, 0.0);
        std::vector<double> separation(U_SIZE, 0.0);
        std::vector<double> alignment(U_SIZE, 0.0);
        std::vector<double> cohesion(U_SIZE, 0.0);
        std::vector<double> repulsion(U_SIZE, 0.0);
        int count = 0;
        double k1 = 1.2;
        double k2 = 1.0;
        double k3 = 2;
        double k4 = 3.0;  // 斥力の重み

        for (const auto& other : others) {
            if (is_same(other)) continue;
            double dist = get_dist(other);
            if (dist < range) {
                // Separation
                std::vector<double> diff = get_vect(other);
                for (auto& d : diff) d *= -1.0;
                for (int i = 0; i < U_SIZE; ++i) separation[i] += diff[i] / (dist*dist);

                // Alignment
                std::vector<double> other_vel = other.get_velocity();
                for (int i = 0; i < U_SIZE; ++i) alignment[i] += other_vel[i];

                // Cohesion
                std::vector<double> other_pos = other.get_position();
                for (int i = 0; i < U_SIZE; ++i) cohesion[i] += other_pos[i];

                ++count;
            }
        }

        if (count > 0) {
            for (int i = 0; i < U_SIZE; ++i) {
                alignment[i] /= count;
                cohesion[i] /= count;
                cohesion[i] -= get_position()[i]; // Move towards the center of mass
            }
            normalize(separation);
            normalize(alignment);
            normalize(cohesion);

            // マップ上の塗られたセルからの斥力を計算
            auto idx = agentMap.get_index(get_position());
            int viewCells = static_cast<int>(range / agentMap.get_scale());

            for (int i = -viewCells; i <= viewCells; ++i) {
                for (int j = -viewCells; j <= viewCells; ++j) {
                    int nx = idx.first + i;
                    int ny = idx.second + j;

                    if (nx >= 0 && nx < agentMap.get_size()[0] && ny >= 0 && ny < agentMap.get_size()[1]) {
                        if (agentMap.is_arleady_exist({static_cast<double>(nx), static_cast<double>(ny)})) {
                            double dx = get_position()[0] - (nx * agentMap.get_scale() - FIELD_MAX);
                            double dy = get_position()[1] - (ny * agentMap.get_scale() - FIELD_MAX);
                            double distance = std::sqrt(dx * dx + dy * dy);
                            if (distance > 0) {
                                double force = 1.0 / (distance * distance);
                                repulsion[0] += force * (dx / distance);
                                repulsion[1] += force * (dy / distance);
                            }
                        }
                    }
                }
            }

            // 最終的な力の計算
            for (int i = 0; i < U_SIZE; ++i) {
                u[i] = k1 * separation[i] + k2 * alignment[i] + k3 * cohesion[i] + k4 * repulsion[i];
            }

            /*ボイドモデルのみの計算*/
            // for (int i = 0; i < U_SIZE; ++i) {
            //     u[i] = k1*separation[i] + k2*alignment[i] + k3*cohesion[i];
            // }
            normalize(u);
        }

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

    bool trail_map(double x, double y, double FIELD_MAX) {
        std::vector<std::vector<int>> mark;
        int m_map_size[2] = {100, 100};
        mark.resize(m_map_size[0]);
        for (int i = 0; i < m_map_size[0]; i++) {
            mark[i].resize(m_map_size[1], 0);
        }
        int i = (int)(-y+FIELD_MAX);
        int j = (int)(x+FIELD_MAX);
        mark[i][j]=1;
        return true;
    }

    std::vector<double> &get_vect(const crlAgent &other) {
        static std::vector<double> vect(U_SIZE);
        get_toroidal_vector2(vect, other, 0.0);
        return vect;
    }

    // 速度を取得するメソッドを追加
    std::vector<double> get_velocity() const {
        std::vector<double> velocity(U_SIZE);
        // 速度の計算ロジックをここに追加
        // 例: m_stat の一部を速度として返す
        velocity[0] = m_stat[2]; // x方向の速度
        velocity[1] = m_stat[3]; // y方向の速度
        return velocity;
    }

    //位置を取得するメソッドを追加
    std::vector<double> get_position() const {
        std::vector<double> position(U_SIZE);
        position[0] = m_stat[0]; // x方向の位置
        position[1] = m_stat[1]; // y方向の位置
        return position;
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