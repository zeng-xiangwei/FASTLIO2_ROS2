#include "ieskf.h"
#include <glog/logging.h>

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V24D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
    g += delta.segment<3>(21);
}

V24D State::operator-(const State &other) const
{
    V24D delta = V24D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    delta.segment<3>(21) = g - other.g;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V24D delta = V24D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    m_F.block<3, 3>(0, 15) = -Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;
    m_F.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity() * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    shared_data.converge = true;
    V24D delta = V24D::Zero();
    M24D H = M24D::Identity();
    V24D b;
    int converge_count = 0;
    const float epsi = 0.001;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M24D J = M24D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        H += J.transpose() * m_P.inverse() * J;
        b += J.transpose() * m_P.inverse() * delta;

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        delta = -H.inverse() * b;

        m_x += delta;
        shared_data.iter_num += 1;

        shared_data.converge = true;
        for (int j = 0; j < 24; j++)
        {
            if (std::fabs(delta(j)) > epsi) //如果dx>epsi 认为没有收敛
            {
                shared_data.converge = false;
                break;
            }
        }

        if (shared_data.converge)
			converge_count++;

        if (converge_count != 0 && i == m_max_iter - 2) //如果迭代了3次还没收敛 强制令成true，h_share_model函数中会重新寻找近邻点
        {
            shared_data.converge = true;
        }

        if (m_stop_func(delta))
            break;
    }

    M24D L = M24D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    m_P = L * H.inverse() * L.transpose();
}