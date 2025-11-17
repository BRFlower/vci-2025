#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            // rotation(auto calculation) :   glm::quat * glm::vec3   
            ik.JointGlobalPosition[i] = ik.JointGlobalRotation[i-1] * ik.JointLocalOffset[i] + ik.JointGlobalPosition[i - 1];
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            for (int i = ik.JointLocalOffset.size()-2; i >= 0; i --){
                auto rt0 = (i == 0) ? glm::quat(1,0,0,0) : ik.JointGlobalRotation[i-1];
                auto rt0_ = glm::inverse(rt0);
                auto vec1 = glm::normalize(rt0_ * (ik.JointGlobalPosition[ik.JointLocalOffset.size()-1] - ik.JointGlobalPosition[i]));
                auto vec0 = glm::normalize(rt0_ * (EndPosition - ik.JointGlobalPosition[i]));
                // r * vec1 * r.inv  parrallels to vec0

                // auto vm = normalize(glm::cross(vec1, vec0));
                // float dot = glm::dot(vec1, vec0);
                // if (dot > 0.9999f)
                //     continue;
                // auto theta = float(acos(dot));
                // auto r = glm::angleAxis(theta, vm);
                auto r = glm::rotation(vec1, vec0);
                ik.JointLocalRotation[i] = glm::normalize(r * ik.JointLocalRotation[i]);
                // ForwardKinematics(ik, i);
                ik.JointGlobalPosition[ik.JointLocalOffset.size()-1] = ik.JointGlobalPosition[i] + rt0 * r * rt0_ * (ik.JointGlobalPosition[ik.JointLocalOffset.size()-1] - ik.JointGlobalPosition[i]);
            }
            ForwardKinematics(ik, 0);
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                auto dir = glm::normalize(next_position - ik.JointGlobalPosition[i]);
                backward_positions[i] = next_position - dir * ik.JointOffsetLength[i+1];
                next_position = backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                auto dir = glm::normalize(backward_positions[i + 1] - now_position);
                forward_positions[i + 1] = forward_positions[i] + dir * ik.JointOffsetLength[i+1];
                now_position = forward_positions[i + 1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }
    
    glm::vec3 L(float t, glm::vec3 p1, glm::vec3 p2) {
        return p1 * (1-t) + p2 * t;
    }
    glm::vec3 circ(float t, glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, float theta1, float theta2, float scale = 1){ // theta1 * scale
        float q = theta1 * scale * (1-t) + theta2 * scale * t;
        return p0 + p1 * cos(q) + p2 * sin(q);
    }
    std::shared_ptr<std::vector<glm::vec3>> character(int nums){
        using Vec3Arr = std::vector<glm::vec3>;
        nums /= 32;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums * 32));
        int index = 0;
        for (int i = 0; i < 10 * nums; i ++)
            (*custom)[index++] = L(i / (10.0 * nums), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f));
        
        for (int i = 0; i < 2 * nums; i ++)
            (*custom)[index++] = L(i / (2.0 * nums), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.2f));
        
        for (int i = 0; i < 7 * nums; i ++)
            (*custom)[index++] = circ(i / (7.0 * nums), glm::vec3(-0.25f, 0.0f, 0.2f), glm::vec3(0.25f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.25f), 0, 1, 3.14159);

        for (int i = 0; i < 2 * nums; i ++)
            (*custom)[index++] = L(i / (2.0 * nums), glm::vec3(-0.5f, 0.0f, 0.2f), glm::vec3(-0.5f, 0.0f, 0.0f));
        for (int i = 0; i < 2 * nums; i ++)
            (*custom)[index++] = L(i / (2.0 * nums), glm::vec3(-0.5f, 0.0f, 0.0f), glm::vec3(-0.5f, 0.0f, 0.2f));
        
        for (int i = 0; i < 7 * nums; i ++)
            (*custom)[index++] = circ(i / (7.0 * nums), glm::vec3(-0.75f, 0.0f, 0.2f), glm::vec3(0.25f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.25f), 0, 1, 3.14159);
        
        for (int i = 0; i < 2 * nums; i ++)
            (*custom)[index++] = L(i / (2.0 * nums), glm::vec3(-1.0f, 0.0f, 0.2f), glm::vec3(-1.0f, 0.0f, 0.0f));
        return custom;
    }
    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        // std::shared_ptr<Vec3Arr> custom = character(32 * 50);
        // int index = 32 * 50;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(int(nums*1.01)));
        int index = 0;
        // std::shared_ptr<Vec3Arr> custom = character(320);
        // int index = 320;
        // check step nums/5;

        ///*
        float check[nums / 5];
        auto p0 = glm::vec3(glm::vec3(custom_x(0), 0.0, custom_y(0)));
        float avg = 0;
        for (int i = 1; i < nums / 5; i ++){
            auto p1 = glm::vec3(custom_x(92 * glm::pi<float>() * i*5 / nums), 0.0, custom_y(92 * glm::pi<float>() * i*5 / nums));
            check[i] =  glm::length(p1 - p0);
            p0 = p1;
            avg += check[i];
        }
        check[0] = check[1];
        avg /= nums / 5;
        //normalize
        for (int i = 0; i < nums / 5; i++){
            check[i] = glm::clamp(avg / check[i], 0.1f, 10.0f);
        }
        //clamp too much
        avg = 0;
        for (int i = nums / 5 - 1; i >= 0; i--){
            if (check[i] == 10)
                check[i] = (i + 1 == nums / 5)? 1 : check[i + 1];
        }
        for (float i = 0;i < nums;) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            i += check[int(i / 5)];
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        //*/

        // for (int i = 0; i < nums; i++) {
        //     float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
        //     float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
        //     if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
        //     (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        // }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        int const steps = 1000;
        float const ddt = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }
    }
}
