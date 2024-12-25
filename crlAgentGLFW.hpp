/**
    @file expGLFW.hpp
    @brief  demonstration for GLFW sample program (rapper class for crlGLFW.hpp)
    @author Hiroshi Igarashi
    @author Tokyo Denki Univ.
    @license  MIT License
    @date 2023/09/28
    @version 1.0
 */

#ifndef __EXP_GLFW_HPP__
#define __EXP_GLFW_HPP__


#include "include/crlGLFW.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include "crlAgentCoreMap.hpp"

#define EXP_DIM 2 // 実験環境次元

// 色の定義
const std::vector<double> &_red() {
    static std::vector<double> color(4, 0.0);
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 0.9;
    return color;
}

const std::vector<double> &_blue() {
    static std::vector<double> color(4, 0.0);
    color[0] = 0.0;
    color[1] = 0.0;
    color[2] = 1.0;
    color[3] = 0.1;
    return color;
}

const std::vector<double> &_green() {
    static std::vector<double> color(4, 0.0);
    color[0] = 0.0;
    color[1] = 1.0;
    color[2] = 0.0;
    color[3] = 1.0;
    return color;
}

const std::vector<double> &_magenta() {
    static std::vector<double> color(4, 0.0);
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 1.0;
    color[3] = 1.0;
    return color;
}


class crlAgentGLFW : public crlGLFW {
    agentCoreMap *m_agentMap; // エージェントマップ用クラス

    bool m_init_flg;
    int m_object_num;
    // 10x2 の配列を作成し、全要素を 0.0 で初期化
    std::vector<std::vector<double>> m_x_pos;
    std::vector<std::vector<double>> m_x_color;
    std::vector<double> m_x_radius;
    std::vector<bool> m_x_fill;

    //std::vector<double> m_x;
    //std::vector<double> m_r;

    std::vector<double> m_scroll;
    double m_s;
    double m_g_s; // scale x, y
    std::vector<double> m_ua; // input assist vector for showing
    std::vector<double> m_ub; // input base vector for showing
    bool m_shakedown;
    double m_lp[2]; // last mouse position
    std::vector<double> m_mv; // mouse input vector
    bool m_act; // mouse action

public:
    crlAgentGLFW() : crlGLFW() {
        m_init_flg = false;
        m_g_s = 0.95;
    }

    bool init(int object_num, double field_size) {

        if (m_init_flg) {
            std::cout << "#debug: already initialized." << std::endl;
            return false;
        }

        m_s = 1 / (field_size);
        m_init_flg = true;
        m_object_num = object_num;
        m_x_pos.assign(object_num, std::vector<double>(EXP_DIM, 0.0));
        m_x_color.assign(object_num, std::vector<double>(4, 0.0));
        m_x_radius.assign(object_num, 1.0);
        m_x_fill.assign(object_num, false);
        //for(int i=0; i<object_num; i++) {
        //    std::cout << "#debug: m_x_pow[" << i << "]: [" << m_x_pos[i][0] << ", " << m_x_pos[i][1] << "]" << std::endl;
        //}
        m_scroll.resize(EXP_DIM);
        m_mv.resize(EXP_DIM);
        m_ua.resize(EXP_DIM);
        m_ub.resize(EXP_DIM);
        m_act = false;
        m_shakedown = true;
        return true;
    }

    /*マップクラスのインスタンスをGLFWに設定するための関数*/
    bool set_agent_map(agentCoreMap *agentMap) {
        m_agentMap = agentMap;
        return true;
    }

    void put_object(const std::vector<double> &center_pos, const double radius, std::vector<double> colors, double scale,
               bool fill) const {

        glColor4d(colors[0], colors[1], colors[2], colors[3]);
        if (fill) glBegin(GL_POLYGON);
        else glBegin(GL_LINE_LOOP);
        for (double d = 0.0; d < 2.0 * M_PI; d += 0.1) {
            glVertex2d((center_pos[0] + radius * cos(d)) * scale, (center_pos[1] + radius * sin(d)) * scale);
        }
        glEnd();
    }

    void draw_object_vector(const std::vector<double> &x, const std::vector<double> &v, double scale, const double r,
                            const double g,
                            const double b) const {

        double gain = 0.5;
        glColor3d(r, g, b);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex2d(x[0] * scale, x[1] * scale);
        glVertex2d((x[0] + v[0] * gain) * scale, (x[1] + v[1] * gain) * scale);
        glEnd();
    }

    void show_background() {

        if (m_shakedown)
            glColor3d(1.0, 1.0, 1.0);
        else
            glColor3d(0.975, 0.975, 1.0);

        glBegin(GL_POLYGON);
        glVertex2d(-1, -1);
        glVertex2d(-1, 1);
        glVertex2d(1, 1);
        glVertex2d(1, -1);
        glEnd();

        glLineWidth(2.0);
        glColor3d(0.0, 0.0, 0.0);
        glBegin(GL_LINE_LOOP);
        glVertex2d(-m_g_s, -m_g_s);
        glVertex2d(-m_g_s, m_g_s);
        glVertex2d(m_g_s, m_g_s);
        glVertex2d(m_g_s, -m_g_s);
        glEnd();

        glLineWidth(0.5);
        glBegin(GL_LINES);
        glVertex2d(-m_g_s, 0.0);
        glVertex2d(m_g_s, 0.0);
        glVertex2d(0.0, -m_g_s);
        glVertex2d(0.0, m_g_s);
        glEnd();
    }

    bool set_shakedown(bool flg) {
        m_shakedown = flg;
        return true;
    }

    bool get_shakedown() {
        return m_shakedown;
    }

    bool set_u_assist(const std::vector<double> &u) {
        if (u.size() != 2) return false;
        m_ua = u;
        return true;
    }

    bool set_u_base(const std::vector<double> &u) {
        if (u.size() != 2) return false;
        m_ub = u;
        return true;
    }

    void display() {

        show_background();

        glLineWidth(2.0);
        for (int i = 0; i < m_object_num; i++) {
            put_object(m_x_pos[i], m_x_radius[i], m_x_color[i], m_s*m_g_s, m_x_fill[i]);
            //std::cout <<"#debug["<<i<<"]: rad: "<<m_x_radius[i]<< ", pos: (" << m_x_pos[i][0] << ", " << m_x_pos[i][1] << ")" << std::endl;
        }


        //std::cout <<"#debug["<<0<<"]: rad: "<<m_x_radius[0]<< ", pos: (" << m_x_pos[0][0] << ", " << m_x_pos[0][1] << ")" << std::endl;
        //std::cout << "#debug: display" << std::endl;
    }

    bool set_obj(int id, const std::vector<double> &pos, const std::vector<double> &color, double radius, bool fill) {
        if (id >= m_object_num) {
            std::cerr << "#error: id: " << id << " is out of range." << std::endl;
            return false;
        }
        if (pos.size() != EXP_DIM) {
            std::cerr << "#error: pos size is not EXP_DIM: " << EXP_DIM << ". @set_obj()" << std::endl;
            return false;
        }
        m_x_pos[id] = pos;
        m_x_color[id] = color;
        m_x_radius[id] = radius;
        m_x_fill[id] = fill;
        return true;
    }


    bool set_ope(const std::vector<double> &pos) {
        if (pos.size() != EXP_DIM) {
            std::cerr << "#error: pos size is not EXP_DIM: " << EXP_DIM << ". @set_ope()" << std::endl;
            return false;
        }
        int x_id = 0;
        m_x_pos[x_id] = pos;
        m_x_color[x_id][0] = 0.0;
        m_x_color[x_id][1] = 0.0;
        m_x_color[x_id][2] = 1.0;
        m_x_color[x_id][3] = 0.0;
        m_x_radius[x_id] = 3.0;
        m_x_fill[x_id] = true;
        return true;
    }

    bool set_ref(const std::vector<double> &pos) {
        if (pos.size() != EXP_DIM) {
            std::cerr << "#error: pos size is not EXP_DIM: " << EXP_DIM << ". @set_ref()" << std::endl;
            return false;
        }
        int x_id = 1;
        m_x_pos[x_id] = pos;
        m_x_color[x_id][0] = 1.0;
        m_x_color[x_id][1] = 0.0;
        m_x_color[x_id][2] = 0.0;
        m_x_color[x_id][3] = 0.0;
        m_x_radius[x_id] = 3.0;
        m_x_fill[x_id] = false;
        return true;
    }


    const std::vector<double> &get_mv() const {
        return m_mv;
    }

    bool set_mv0() {
        m_mv[0] = 0.0;
        m_mv[1] = 0.0;
        return true;
    }

    void execute(const char *name, const int width, const int height) {

        if (!glfwInit()) return;

        GLFWwindow *window = glfwCreateWindow(width, height, name, NULL, NULL);
        if (!window) {
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(window);
        // 作成したウィンドウにコールバック関数を設定する
        setCallback(window);
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT);
            glClearColor(0.95, 0.95, 0.95, .5);
            display();
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
        glfwTerminate();
    }

    void keyFun(int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)) {
            glfwTerminate();
            exit(0);
        }
    }

    void windowSize(int width, int height) {

    }

    void mousePos(double x, double y) {

        static bool first = true;
        if (first) {
            m_lp[0] = x;
            m_lp[1] = y;
            first = false;
        }
/*
        if (m_act) {
            m_lp[0] = x;
            m_lp[1] = y;
            std::cout << "#debug: set_lp: ["<< m_lp[0] << ", " << m_lp[1] << "]" << std::endl;
            m_act = false;
        }
        m_mv[0] =  (x - m_lp[0])/100.0;
        m_mv[1] = - (y - m_lp[1])/100.0;
        std::cout << "#debug: mv: ["<< m_mv[0] << ", " << m_mv[1] << "]" << std::endl;
*/
        if (m_act) {
            m_mv[0] = (x - m_lp[0]) / 2.0;
            m_mv[1] = -(y - m_lp[1]) / 2.0;
            //std::cout << "#debug: mv: ["<< m_mv[0] << ", " << m_mv[1] << "]" << std::endl;
        } else {
            m_mv[0] = 0.0;
            m_mv[1] = 0.0;
        }
        m_lp[0] = x;
        m_lp[1] = y;

        if (m_mv[0] > 1.0) m_mv[0] = 1.0;
        if (m_mv[0] < -1.0) m_mv[0] = -1.0;
        if (m_mv[1] > 1.0) m_mv[1] = 1.0;
        if (m_mv[1] < -1.0) m_mv[1] = -1.0;
    }

    void mouseButton(int button, int action, int mods) {
        //printf("mouseButton %d %d %d\n", button, action, mods);
        if (action == GLFW_PRESS) {
            m_act = true;
        }
    }

    void mouseScroll(double x, double y) {
        printf("mouseScroll %.1lf %.1lf\n", x, y);
        /*
        m_mv[0] = x;
        m_mv[1] = -y;
        if(m_mv[0] > 1.0) m_mv[0] = 1.0;
        if(m_mv[0] < -1.0) m_mv[0] = -1.0;
        if(m_mv[1] > 1.0) m_mv[1] = 1.0;
        if(m_mv[1] < -1.0) m_mv[1] = -1.0;
        std::cout << "#debug: mv: ["<< m_mv[0] << ", " << m_mv[1] << "]" << std::endl;
*/
    }
};

#endif //__EXP_GLFW_HPP__