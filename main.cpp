#include <iostream>
#include <vector>
#include <cmath>
#include "crlAgent.hpp"
#include "crlAgentGLFW.hpp"
#include "crljoystick.hpp"
#include <thread>

crlAgentGLFW g_wnd; // GLFW ウィンドウ用クラス
#define SAMPLING_TIME 0.033 // サンプリング時間 [sec]
#define FIELD_MAX 100.0 // フィールドの大きさ
#define AGENT_NUM 12

// メインループ（この関数内のwhile内を繰り返し実行）
void main_loop(int speedx) {

    const int agent_num = AGENT_NUM; // エージェントの数
    const double field_max = FIELD_MAX; //(xの範囲: -field_max ~ field_max，yの範囲: -field_max ~ field_max)

    // agent_num 台のエージェントを定義
    // agent[0]: ID 0 のエージェント
    // agent[0].get_pos(): ID 0 のエージェントの位置を取得
    // agent[0].get_dist(agent[1]): ID 0 のエージェントと ID 1 のエージェントの距離を取得
    // agent[0].get_vect(agent[1]): ID 0 のエージェントから ID 1 のエージェントへのベクトルを取得
    // agent[0].drive(u, agent, SAMPLING_TIME): ID 0 のエージェントに入力 u を与えて駆動
    //          ※ agent は他のエージェントを含めた配列（衝突判定のため）
    std::vector<crlAgent> agent(agent_num);

    // エージェントの初期化
    for (int i = 0; i < agent_num; i++) {
        // エージェントの初期化
        agent[i].init(i, 0, field_max);
        // 初期位置をランダムに設定 (範囲: -field_max ~ field_max の75%)
        agent[i].set_pos_random(field_max * 0.75);
    }

    std::vector<double> u(2); // エージェントへの入力司令ベクトル 2次元 u[0], u[1]
    double sec = 0.0; // 現在時刻
    int nearest_agent_id = 0; // 最も近くのエージェント番号

    // メインループ ここを主に編集
    while (true) {
        for (int i = 0; i < agent_num; i++) {
            // 一番近くのエージェント ID を取得 (int nearest_agent_id に代入)
            nearest_agent_id = agent[i].get_nearest_agent_id(agent);

            if (i < 5) {
                // エージェントのランダムウォーク入力を獲得 (u[0] = -5〜5, u[1] = -5〜5)
                u = agent[i].get_random_walk(5.0);
                // エージェントの駆動(入力は u[0], u[1])
                agent[i].drive(u, agent, SAMPLING_TIME);
                // 描画用にエージェントをセット [編集不要]
                g_wnd.set_obj(i, agent[i].get_pos(), _blue(), agent[i].get_radius(), false);
            } else if (i < 8) {
                // エージェントの入力
                u[0] = sin(sec);
                u[1] = cos(sec);
                // エージェントの駆動(入力は u[0], u[1])
                agent[i].drive(u, agent, SAMPLING_TIME);
                // 描画用にエージェントをセット [編集不要]
                g_wnd.set_obj(i, agent[i].get_pos(), _red(), agent[i].get_radius(), true);
            } else {
                // nearest_agent_id 方向へのベクトルを取得 u に代入
                u = agent[i].get_vect(agent[nearest_agent_id]);
                // u を正規化 （大きさを1に）
                normalize(u);
                // エージェントの駆動(入力は u[0], u[1])
                agent[i].drive(u, agent, SAMPLING_TIME);
                // 描画用にエージェントをセット [編集不要]
                g_wnd.set_obj(i, agent[i].get_pos(), _green(), agent[i].get_radius(), true);
            }

            // エージェントの現在地をコンソールに出力
            agent[i].print_position(i);
        }
        // sleep [描画のために必要] 数値計算のみでは不要
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int) (SAMPLING_TIME * 1000.0 / speedx) - 12));
        // 時刻を 33ms 進める
        sec += SAMPLING_TIME; // SAMPLING_TIME: xuHuman.hpp で定義
    }
}

int main() {

    g_wnd.init(AGENT_NUM, FIELD_MAX);
    g_wnd.set_shakedown(false); // 慣らし運転モードを終了
    // メインループをスレッドで呼び出し
    // 2つめの引数（int型）は再生倍率
    std::thread th1(main_loop, 1);

    // GLFWの設定（画面サイズの設定可能・正方形がおすすめ）
    g_wnd.execute("multi agent sim", 640, 640);
    // wnd.execute で止まるので，ここまでは来ない...
    return 0;
}
