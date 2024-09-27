# crlMAS
マルチエージェントシステムのミニマムプログラム

## 概要
マルチエージェントシステムのミニマムプログラムです。
主な機能は以下の通りです。

- エージェントの生成/動作記述
- エージェントの描画(GLFW)

## 主な構成ファイル
- "main.cpp"  : メインプログラム
- "crlAgent.hpp" : エージェントクラス （crlAgentCoreを継承）
- "crlAgentGLFW.hpp" : GLFWによるエージェントの描画クラス（編集不要）
- "crlAgentCore.hpp" : エージェントクラスのベースクラス（編集不要）
- "crlAgentCore_config.h" : crlAgentCore用設定ファイル（編集不要）

## main.cpp
すべての起点となるメインプログラム。

    void main_loop()
にエージェントの動作を記述する。

### エージェントの生成
    std::vector<crlAgent> agent(10)
エージェントを10体生成する。

    agent[0]
ID 0 のエージェントにアクセスできる

## crlAgent.hpp
エージェントの基本クラス

### エージェントの動作記述

- エージェントの位置を取得する。 

> std::vector<double> & crlAgent::get_pos()


    使い方の例:
        // agent[0]の位置座標（2次元ベクトル p[0], p[1])を取得する。
        std::vector<double> p = agent[0].get_pos()



- エージェントを駆動する。

> void crlAgent::drive(std::vector<double> & u, std::vector<crlAgent> &others, double smp_time)
 
    使い方の例:

       std::vector<double> u = {1.0, 0.0}; // 入力ベクトル
       //agent[0]に入力ベクトル（2次元ベクトル u[0], u[1])を与える。
       agent[0].drive(u, agent, SMP_TIME);
 
- 他のエージェントとの距離・ベクトルを獲得

> double get_dist(crlAgent &other)
> std::vector<double> get_vec(crlAgent &other)

    使い方の例:

        // agent[0]とagent[1]の距離を取得
        double dist = agent[0].get_dist(agent[1]);
        // agent[0]とagent[1]のベクトルを取得
        std::vector<double> vec = agent[0].get_vec(agent[1]);

