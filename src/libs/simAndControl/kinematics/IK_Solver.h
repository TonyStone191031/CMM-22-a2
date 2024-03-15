#pragma once

#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/Robot.h>

namespace crl {

struct IK_EndEffectorTargets {
    RB *rb = nullptr;
    P3D p;       // local coordinates of end effector in rb's frame
    P3D target;  // target position in world frame
};

class IK_Solver {
public:
    IK_Solver(Robot *robot) : robot(robot) {}

    ~IK_Solver(void) {}

    /**
     * add IK end effector target to solver. Specify the end effector point p, which 
     * is specified in the local coordinates of rb and its target expressed in world frame.
     */
    void addEndEffectorTarget(RB *rb, P3D p, P3D target) {
        endEffectorTargets.push_back(IK_EndEffectorTargets());
        endEffectorTargets.back().rb = rb;
        endEffectorTargets.back().p = p;
        endEffectorTargets.back().target = target;
    }

    void solve(int nSteps = 10) {
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);

        for (uint i = 0; i < nSteps; i++) {
            dVector q;
            gcrr.getQ(q);

            // get current generalized coordinates of the robots

            // TODO: Ex.2-2 Inverse Kinematics
            //
            // update generalized coordinates of the robot by solving IK.

            // remember, we don't update base pose since we assume it's already at
            // the target position and orientation
            dVector deltaq(q.size() - 6);
            deltaq.setZero();

            // TODO: here, compute deltaq using GD, Newton, or Gauss-Newton.
            // end effector targets are stored in endEffectorTargets vector.
            //
            // Hint:
            // - use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
            // - if you already implemented analytic Jacobian, you can use gcrr.compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq)
            // - don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
            // - when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
            //   see https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

            // TODO: your implementation should be here.

            // Since we have already provided with q.tail(q.size() - 6) += deltaq; (see below)
            // then we just accumulate deltaq over the four end effectors of the quadruped robot
            for (int j = 0; j < 4; j++) {
                // initialization of the jth end effector (note that end effector targets are stored in endEffectorTargets vector)
                P3D EE_j_position_in_local_frame = endEffectorTargets[j].p;
                RB* rb_of_EE_j = endEffectorTargets[j].rb;
                Matrix dpdq;
                
                // Hint: use gcrr.estimate_linear_jacobian(p, rb, dpdq) function for Jacobian matrix.
                // gcrr.estimate_linear_jacobian(EE_j_position_in_local_frame, rb_of_EE_j, dpdq);

                // - if you already implemented analytic Jacobian, you can use gcrr.compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq)
                gcrr.compute_dpdq(EE_j_position_in_local_frame, rb_of_EE_j, dpdq);
                
                // Hint: don't forget we use only last q.size() - 6 columns (use block(0,6,3,q.size() - 6) function)
                // Details: When you overwrite eigen matrix variable values in a single statement, you should use eval() function to prevent corruption.
                dpdq = dpdq.block(0, 6, 3, q.size()-6).eval();

                // Gauss-Newton method: q_j(i+1) = q_j(i) + inv(J_transpose * J) * J_transpose * (target_j - FK_j(q))
                // where i is the iterative number (note the outer loop i < nSteps), and j is the jth end effector
                P3D target_j = endEffectorTargets[j].target;
                // obtain FK_j(q) via getWorldCoordinates(), and do not forget "gcrr." (already used for gcrr.estimate_linear_jacobian())
                P3D EE_j_position_in_world_frame = gcrr.getWorldCoordinates(EE_j_position_in_local_frame, rb_of_EE_j);
                // we just accumulate deltaq, since we have already provided with q.tail(q.size() - 6) += deltaq; (see below)
                // therefore, deltaq = inv(J_transpose * J) * J_transpose * (target_j - FK_j(q))
                // Hint: when you compute inverse of the matrix, use ldlt().solve() instead of inverse() function. this is numerically more stable.
                // Grammar: Ax = b can be solved by x = A.ldlt().solve(b)
                // note that deltaq has already be set to zero: deltaq.setZero();
                deltaq += (dpdq.transpose() * dpdq).ldlt().solve(dpdq.transpose() * V3D(target_j - EE_j_position_in_world_frame));
            }

            // solve each IK problem one by one (four end effectors in all) and sum up the solutions
            q.tail(q.size() - 6) += deltaq;

            // now update gcrr with q
            gcrr.setQ(q);
        }

        gcrr.syncRobotStateWithGeneralizedCoordinates();

        // clear end effector targets
        // we will add targets in the next step again.
        endEffectorTargets.clear();
    }

private:
    Robot *robot;
    std::vector<IK_EndEffectorTargets> endEffectorTargets;
};

}  // namespace crl