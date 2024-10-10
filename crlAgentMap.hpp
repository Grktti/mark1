/**
    @file agentCoreMap.hpp
    @brief  Global map of Scramble Crossing mode (Dynamic Pedestrian Collision Avoidance)
    @author Hiroshi Igarashi
    @author Tokyo Denki Univ.
    @license  MIT License
    @date 2023/09/10
    @version 1.0
 */

#ifndef AGENT_CORE_MAP_HPP
#define AGENT_CORE_MAP_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>
#include <cmath>
#include "include/crlGLFW.hpp"
#include "crlAgentCore.hpp"
#include "crlAgentGLFW.hpp"
#define FIELD_MAX 100.0 // フィールドの大きさ



namespace ac = agentcore;

class agentCoreMap {
protected:
    const int task_dim = 2;  // 2次元の場合
    double m_scale = 1.0;
    bool m_init_flg;
//    std::vector<std::vector<int>> m_map; // m_map[i][j] /2次元平面のマップを作成

    std::vector<std::vector<std::vector<double>>> m_map; // m_map[i][j][idx] *: exist, id, typeの3次元ベクトル
    enum idx {// existで通ったかどうかのフラグ、
        EXIST = 0,
        ID,
        TYPE
    };

    ac::field_environment_t m_env;
    std::vector<int> m_map_size;

public:
    agentCoreMap() {
        m_map_size.resize(task_dim, 0);
        m_init_flg = false;
    };

    bool init(const ac::field_environment_t& env, const double scale) {
        if (scale < 0.0) {
            std::cerr << "#error: scale must be positive! ";
            std::cerr << "@agentCoreMap::init() : " << __FILE__ << __LINE__ << std::endl;
        }
        m_scale = scale;
        m_env = env;
        init_map(m_env, m_scale);
        return true;
    };

    bool init_map(const ac::field_environment_t& env, const double scale) {
        m_map_size.resize(task_dim, 0);//m_map_sizeをtask_dimの次元で初期化

        for (int i = 0; i < task_dim; i++) {
            m_map_size[i] = (int)(2*FIELD_MAX / scale);//scaleによってm_map_sizeを設定(今は200×200)
        }

        m_map.resize(m_map_size[0]);//m_map_size[0] にリサイズ(各列は EXIST、ID、TYPE の3つの要素を持つベクター)
        for (int i = 0; i < m_map_size[0]; i++) {
            m_map[i].resize(m_map_size[1]);
            for (int j = 0; j < m_map_size[1]; j++) {
                m_map[i][j].resize(3, 0); // m_map_size[0] x m_map_size[1] x 3 のサイズを持つ3次元ベクターで各要素は０
            }
        }
        m_init_flg = true;
        return true;
    }

    double map_force(const std::vector<double>& pos, const double radius) {
        return 0.0;
    }

    int get_index(double pos, int dim) {
        return (int)((pos + (0.5 * m_env.field_size[dim])) / m_scale);

    };



    [[nodiscard]] bool is_arleady_exist(const std::vector<double>& pos) const {
        if (!m_init_flg) {
            std::cerr << "#error: not initialized! ";
            std::cerr << "@agentCoreMap::is_arleady_exist()" << std::endl;
            return false;
        }

        std::vector<int> idx;
        if (!get_index(idx, pos)) {
            std::cerr << "#error: invalid position! ";
            std::cerr << "@agentCoreMap::is_arleady_exist()" << std::endl;
            return false;
        }
        if (m_map[idx[0]][idx[1]][EXIST] == 1) {
            return true;
        }
        return false;
    }


//    // エントロピー計算関数
//    double get_entropy() {
//        const int width = 100;
//        const int height = 100;
//        // エージェントが存在するセルのカウント
//        std::unordered_map<std::pair<int, int>, int, pair_hash> cell_counts;
//
//        // エージェントの位置を確認し、カウント
//        for (int x = 0; x < width; ++x) {
//            for (int y = 0; y < height; ++y) {
//                if (get_exist(x, y)) {
//                    cell_counts[{x, y}]++;
//                }
//            }
//        }
//
//        // エージェントの総数をカウント
//        int total_agents = 0;
//        for (const auto& entry : cell_counts) {
//            total_agents += entry.second;
//        }
//
//        // 確率分布を計算
//        double entropy = 0.0;
//        for (const auto& entry : cell_counts) {
//            double prob = static_cast<double>(entry.second) / total_agents;
//            if (prob > 0) {
//                entropy -= prob * std::log2(prob);
//            }
//        }
//
//        return entropy;
//    }

    bool get_exist(int x, int y) {
        return m_map[x][y][EXIST];
    }

//    bool assign(const agentCore& a) {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::assign()" << std::endl;
//            return false;
//        }
//
//        std::vector<int> idx;
//        if (!get_index(idx, a)) {
//            std::cerr << "#error[" << a.label() << "]: get_index() failed! ";
//            std::cerr << "@agentCoreMap::assign()" << std::endl;
//            return false;
//        }
//        if (m_map[idx[0]][idx[1]][EXIST] == 1) {
//            std::cerr << "#error[" << a.label() << "]: already exist! ";
//            std::cerr << "@agentCoreMap::assign()" << std::endl;
//            return false;
//        }
//        m_map[idx[0]][idx[1]][ID] = a.get_id();
//        m_map[idx[0]][idx[1]][TYPE] = a.get_type();
//        m_map[idx[0]][idx[1]][EXIST] = 1;
//        return true;
//    }
//
//    bool remove(const agentCore& a) {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::remove()" << std::endl;
//            return false;
//        }
//
//        std::vector<int> idx;
//        if (!get_index(idx, a)) {
//            std::cerr << "#error[" << a.label() << "]: get_index() failed! ";
//            std::cerr << "@agentCoreMap::remove()" << std::endl;
//            return false;
//        }
//        if (m_map[idx[0]][idx[1]][EXIST] == 0) {
//            std::cerr << "#error[" << a.label() << "]: not exist! ";
//            std::cerr << "@agentCoreMap::remove()" << std::endl;
//            return false;
//        }
//        m_map[idx[0]][idx[1]][ID] = 0;
//        m_map[idx[0]][idx[1]][TYPE] = -1;
//        m_map[idx[0]][idx[1]][EXIST] = -1;
//        return true;
//    }
//
//    [[nodiscard]] bool removable_check(const agentCore& a) const {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::removable_check()" << std::endl;
//            return false;
//        }
//
//        std::vector<int> idx;
//        if (!get_index(idx, a)) {
//            std::cerr << "#error[" << a.label() << "]: get_index() failed! ";
//            std::cerr << "@agentCoreMap::removable_check()" << std::endl;
//            return false;
//        }
//
//        if (m_map[idx[0]][idx[1]][EXIST] == 0) {
//            std::cerr << "#error[" << a.label() << "]: not exist! ";
//            std::cerr << "@agentCoreMap::removable_check()" << std::endl;
//            return false;
//        }
//        return true;
//    }
//
//    [[nodiscard]] int count_exist() const {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::count_exist()" << std::endl;
//            return -1;
//        }
//        int cnt = 0;
//
//         for (int i = 0; i < m_map_size[0]; i++) {
//         	for (int j = 0; j < m_map_size[1]; j++) {
//         		if (m_map[i][j][EXIST] == 1) cnt++;
//         	}
//         }
//
//        for (auto &mx: m_map) {
//            for (auto &mxy: mx) {
//                if (mxy[EXIST] == 1) cnt++;
//            }
//        }
//        return cnt;
//    }
//
//    [[nodiscard]] bool check_map() const {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::check_map()" << std::endl;
//            return false;
//        }
//        // for (int i = 0; i < m_map_size[0]; i++) {
//        // 	for (int j = 0; j < m_map_size[1]; j++) {
//        for (auto &mx: m_map) {
//            for (auto &mxy : mx) {
//                if (mxy[EXIST] == 1) {
//                    if (mxy[ID] < 0 || mxy[TYPE] < 0) {
//                        std::cerr << "#error: invalid data! ";
//                        std::cerr << " inspite of EXIST == 1, ID or TYPE is 0.";
//                        std::cerr << "@agentCoreMap::check_map()" << std::endl;
//                        return false;
//                    }
//                } else if (mxy[EXIST] == 0) {
//                    if (mxy[ID] != -1 || mxy[TYPE] != -1) {
//                        std::cerr << "#error: invalid data! ";
//                        std::cerr << " inspite of EXIST == 0, ID or TYPE is not 0.";
//                        std::cerr << "@agentCoreMap::check_map()" << std::endl;
//                        return false;
//                    }
//                } else {
//                    std::cerr << "#error: invalid data!  ";
//                    std::cerr << " EXIST is not 0 or 1.";
//                    std::cerr << "@agentCoreMap::check_map()" << std::endl;
//                    return false;
//                }
//            }
//        }
//        return true;
//    }

//
//protected:
//    // カスタムハッシュ関数 （for get_entropy）
//    struct pair_hash {
//        template <class T1, class T2>
//        std::size_t operator ()(const std::pair<T1, T2>& p) const {
//            auto h1 = std::hash<T1>{}(p.first);
//            auto h2 = std::hash<T2>{}(p.second);
//            return h1 ^ (h2 << 1); // Combine the hash values
//        }
//    };
//
//    bool clear_map() {
//        if (!m_init_flg) {
//            std::cerr << "#error: not initialized! ";
//            std::cerr << "@agentCoreMap::clear_map()" << std::endl;
//            return false;
//        }
//        for (auto & mx : m_map) {
//            for (auto &mxy : mx) {
//                mxy[EXIST] = 0;
//                mxy[ID] = -1;
//                mxy[TYPE] = -1;
//            }
//        }
//        return true;
//    }
//
//    bool get_index(std::vector<int>& idx, const agentCore& a) const {
//        return get_index(idx, a.get_pos());
//    }
//
//    bool get_index(std::vector<int>& idx, const std::vector<double>& pos) const {
//        idx.resize(task_dim, 0);
//        for (int i = 0; i < task_dim; i++) {
//            idx[i] = (int)((pos[i] + (0.5 * m_env.field_size[i])) / m_scale);
//
//            if (idx[i] >= m_map_size[i]) {
//                idx[i] -= m_map_size[i];
//            }
//            if (idx[i] < 0) {
//                idx[i] += m_map_size[i];
//            }
//        }
//        return check_index(idx);
//    }
//
//    [[nodiscard]] bool check_index(const std::vector<int>& idx) const {
//        for (int i = 0; i < task_dim; i++) {
//            if (idx[i] < 0 || idx[i] >= m_map_size[i]) {
//                std::cerr << "#error: out of field! ";
//                std::cerr << "idx[" << i << "]: " << idx[i] << ", m_map_size: " << m_map_size[i] << " ";
//                std::cerr << "@agentCoreMap::check_index() : ";
//                std::cerr << __FILE_NAME__ << ":" << __LINE__ << std::endl;
//                return false;
//            }
//        }
//        return true;
//    }
};

#endif //AGENT_CORE_MAP_HPP
