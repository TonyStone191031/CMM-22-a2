#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <utils/utils.h>

namespace crl {

GeneralizedCoordinatesRobotRepresentation::
    GeneralizedCoordinatesRobotRepresentation(Robot *a) {
    robot = a;
    resize(q, getDOFCount());
    syncGeneralizedCoordinatesWithRobotState();
}

//returns the total number of degrees of freedom in the reduced representation
int GeneralizedCoordinatesRobotRepresentation::getDOFCount() const {
    //we will have 3 for root position, 3 for root orientation, and then 1 for each joint,
    //assuming they're all hinge joints, which is what this data structure is designed for
    return 6 + (int)robot->jointList.size();
}

//returns the index of the generalized coordinate corresponding to this joint
int GeneralizedCoordinatesRobotRepresentation::getQIdxForJoint(
    RBJoint *joint) const {
    //the root of the robot has a nullptr joint, and as such this should correspond to the first qIdx of the root DOFs
    if (joint == NULL) return 5;
    return 6 + joint->jIndex;
}

//returns the index of the generalized coordinate corresponding to the index of this joint
int GeneralizedCoordinatesRobotRepresentation::getQIdxForJointIdx(
    int jIdx) const {
    return 6 + jIdx;
}

//returns a pointer to the joint corresponding to this generalized coordinate index.
//If the index corresponds to a root DOF, this method will return NULL.
RBJoint *GeneralizedCoordinatesRobotRepresentation::getJointForQIdx(
    int qIdx) const {
    if (qIdx < 6) return NULL;
    return robot->jointList[qIdx - 6];
}

/**
* In the tree-like hierarchy of joints/dofs, this method returns the parent index of the
* dof corresponding to qIdx
*/
int GeneralizedCoordinatesRobotRepresentation::getParentQIdxOf(int qIdx) {
    if (qIdx < 6) return qIdx - 1;
    return getQIdxForJoint(getJointForQIdx(qIdx)->parent->pJoint);
}

// updates q and qDot given current state of robot
void GeneralizedCoordinatesRobotRepresentation::
    syncGeneralizedCoordinatesWithRobotState() {
    // write out the position of the root...
    RobotState state(robot);
    P3D pos = state.getPosition();
    Quaternion orientation = state.getOrientation();

    q[0] = pos.x;
    q[1] = pos.y;
    q[2] = pos.z;

    // Root
    computeEulerAnglesFromQuaternion(orientation, getQAxis(5), getQAxis(4),
                                     getQAxis(3), q[5], q[4], q[3]);

    // Now go through each joint, and decompose it as appropriate...
    for (uint i = 0; i < robot->jointList.size(); i++) {
        int qIdx = getQIdxForJointIdx(i);
        // Only 1-dof hinge joints
        computeRotationAngleFromQuaternion(state.getJointRelativeOrientation(i),
                                           getQAxis(qIdx), q[qIdx]);
    }
}

// returns the axis corresponding to the indexed generalized coordinate,
// expressed in local coordinates
V3D GeneralizedCoordinatesRobotRepresentation::getQAxis(int qIndex) const {
    if (qIndex >= 0 || qIndex < 6) {
        // the first three are the translational dofs of the body
        if (qIndex == 0) return V3D(1, 0, 0);
        if (qIndex == 1) return V3D(0, 1, 0);
        if (qIndex == 2) return V3D(0, 0, 1);
        if (qIndex == 3) return RBGlobals::worldUp;  // y - yaw
        if (qIndex == 4)
            return RBGlobals::worldUp.cross(robot->forward);  // x - pitch
        if (qIndex == 5) return robot->forward;               // z - roll
    }

    return getJointForQIdx(qIndex)->rotationAxis;
}

void GeneralizedCoordinatesRobotRepresentation::
    syncRobotStateWithGeneralizedCoordinates() {
    RobotState rs(robot);
    getReducedRobotState(rs);
    robot->setState(&rs);
}

// given the current state of the generalized representation, output the reduced
// state of the robot
void GeneralizedCoordinatesRobotRepresentation::getReducedRobotState(
    RobotState &state) {
    // set the position, velocity, rotation and angular velocity for the root
    state.setPosition(P3D(0, 0, 0) + getQAxis(0) * q[0] + getQAxis(1) * q[1] +
                      getQAxis(2) * q[2]);
    state.setOrientation(getOrientationFor(robot->root));

    for (uint i = 0; i < robot->jointList.size(); i++) {
        int qIdx = getQIdxForJointIdx(i);
        Quaternion jointOrientation =
            getRotationQuaternion(q[qIdx], getQAxis(qIdx));

        state.setJointRelativeOrientation(jointOrientation, i);
    }
    // and done...
}

// sets the current q values
void GeneralizedCoordinatesRobotRepresentation::setQ(const dVector &qNew) {
    assert(q.size() == qNew.size());
    // NOTE: we don't update the angular velocities. The assumption is that the
    // correct behavior is that the joint relative angular velocities don't
    // change, although the world relative values of the rotations do
    q = qNew;
}

// gets the current q values
void GeneralizedCoordinatesRobotRepresentation::getQ(dVector &q_copy) {
    q_copy = q;
}

void GeneralizedCoordinatesRobotRepresentation::getQFromReducedState(
    const RobotState &rs, dVector &q_copy) {
    dVector q_old = q;

    RobotState oldState(robot);
    robot->setState((RobotState *)&rs);
    syncGeneralizedCoordinatesWithRobotState();
    getQ(q_copy);
    robot->setState(&oldState);

    setQ(q_old);
}

/**
    pLocal is expressed in the coordinate frame of the link that pivots about DOF qIdx.
    This method returns the point in the coordinate frame of the parent of qIdx after
    the DOF rotation has been applied.
*/
P3D GeneralizedCoordinatesRobotRepresentation::
    getCoordsInParentQIdxFrameAfterRotation(int qIndex, const P3D &pLocal) {
    // if qIndex <= 2, this q is a component of position of the base. 
    if (qIndex <= 2) return pLocal;

    // TODO: Ex.1 Forward Kinematics
    // this is a subfunction for getWorldCoordinates() and compute_dpdq()
    // return the point in the coordinate frame of the parent of qIdx after
    // the DOF rotation has been applied.
    // 
    // Hint:
    // - use rotateVec(const V3D &v, double alpha, const V3D &axis) to get a vector 
    // rotated around axis by angle alpha.
    
    // TODO: implement your logic here.

    // rotation + translation
    // obtain the vector that points from joint to pLocal
    RBJoint* joint_pointer = getJointForQIdx(qIndex); // see class RBJoint in RBJoint.h for more details
    P3D joint_vector_in_child_frame = joint_pointer->cJPos;
    V3D vector_from_joint_to_pLocal = V3D(joint_vector_in_child_frame, pLocal); // see mathDefs.h to construct a vector from p1 to p2
    
    // rotation: rotate the joint-to-pLocal vector via rotateVec
    V3D vector_from_joint_to_pLocal_after_rotation = rotateVec(vector_from_joint_to_pLocal, q[qIndex], getQAxis(qIndex));

    // translation
    P3D joint_vector_in_parent_frame = joint_pointer->pJPos;
    P3D pLocal_in_parent_frame = joint_vector_in_parent_frame + vector_from_joint_to_pLocal_after_rotation;

    return pLocal_in_parent_frame;
}

// returns the world coordinates for point p, which is specified in the local
// coordinates of rb (relative to its COM): p(q)
P3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinates(const P3D &p,
                                                                   RB *rb) {
    // TODO: Ex.1 Forward Kinematics
    // implement subfunction getCoordsInParentQIdxFrameAfterRotation() first.
    //
    // Hint: you may want to use the following functions
    // - getQIdxForJoint()
    // - getParentQIdxOf()
    // - getCoordsInParentQIdxFrameAfterRotation() 

    P3D pInWorld;

    // TODO: implement your logic here.

    // Repeat recursively until reaching the base, which corresponds to a qIndex <= 5 in q
    // Therefore, recursion is implemented while qIndex >= 6 using getCoordsInParentQIdxFrameAfterRotation()
    RBJoint* joint = rb->pJoint;    // see class RB in RB.h for more details
    int qIndex = getQIdxForJoint(joint);    // Hint
    P3D pLocal_in_parent_frame = p;

    while(qIndex >= 6) {
        pLocal_in_parent_frame = getCoordsInParentQIdxFrameAfterRotation(qIndex, pLocal_in_parent_frame);   // Recursion
        qIndex = getParentQIdxOf(qIndex);   // Hint
    }

    // apply the final rotation + translation
    // final rotation
    V3D vector_from_joint_to_pLocal_after_rotation = getWorldRotationForQ(qIndex) * V3D(pLocal_in_parent_frame);
    // final translation
    P3D joint_vector_in_base_frame = P3D(q[0], q[1], q[2]);
    pInWorld = joint_vector_in_base_frame + vector_from_joint_to_pLocal_after_rotation;

    return pInWorld;
}

// returns the global orientation associated with a specific dof q...
Quaternion GeneralizedCoordinatesRobotRepresentation::getWorldRotationForQ(
    int qIndex) {
    Quaternion qRes = Quaternion::Identity();
    // 2 here is the index of the first translational DOF of the root -- these
    // dofs do not contribute to the orientation of the rigid bodies...
    while (qIndex > 2) {
        qRes = getRelOrientationForQ(qIndex) * qRes;
        qIndex = getParentQIdxOf(qIndex);
    }
    return qRes;
}

Quaternion GeneralizedCoordinatesRobotRepresentation::getRelOrientationForQ(
    int qIndex) {
    if (qIndex < 3) return Quaternion::Identity();
    return getRotationQuaternion(q[qIndex], getQAxis(qIndex));
}

// this is a somewhat slow function to use if we must iterate through multiple
// rigid bodies...
V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordsAxisForQ(
    int qIndex) {
    if (qIndex < 3) return getQAxis(qIndex);
    return getWorldRotationForQ(qIndex) * getQAxis(qIndex);
}

// returns the world-relative orientation for rb
Quaternion GeneralizedCoordinatesRobotRepresentation::getOrientationFor(
    RB *rb) {
    int qIndex = getQIdxForJoint(rb->pJoint);
    return getWorldRotationForQ(qIndex);
}

// computes the jacobian dp/dq that tells you how the world coordinates of p
// change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq(const P3D &p,
                                                             RB *rb,
                                                             Matrix &dpdq) {
    resize(dpdq, 3, (int)q.size());

    // TODO: Ex.4 Analytic Jacobian
    //
    // Hint: you may want to use the following functions
    // - getQIdxForJoint()
    // - getCoordsInParentQIdxFrameAfterRotation()
    // - getQAxis()
    // - rotateVec()
    // - getParentQIdxOf()

    // TODO: your implementation should be here

    // q = [p_base, Theta_base, theta_1, theta_2, ..., theta_nj]^T, where p_base and Theta_base are both 3-dim vector
    // J, a.k.a. dpdq should be composed of three components, with first 3×3 matrix dp/d(p_base) being 3×3 identity matrix
    // second 3×3 matrix dp/d(Theta_base), and third 3×(q.size()-6) matrix
    
    // first 3×3 matrix dp/d(p_base) being 3×3 identity matrix
    dpdq.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
    
    // second 3×3 matrix dp/d(Theta_base)
    // where p = p_in_world_frame = p_base + R_base * p_in_base_frame
    // obtain p_in_base_frame through the reverse order in getWorldCoordinates()
    P3D pInWorld = getWorldCoordinates(p, rb);
    V3D p_in_base_frame = V3D(pInWorld - P3D(q[0], q[1], q[2]));

    // loop over each orientation of the base, i.e. q[3], q[4], q[5]
    // cross product
    for (int qIndex = 3; qIndex < 6; qIndex++) {
        V3D base_ort_axis_in_world_frame = getWorldCoordsAxisForQ(qIndex);
        V3D qIndex_col_of_dpdq = base_ort_axis_in_world_frame.cross(p_in_base_frame);
        dpdq.block(0, qIndex, 3, 1) = qIndex_col_of_dpdq;
    }

    // third 3×(q.size()-6) matrix
    // determine through which joints we can get from the base to p
    // if a joint theta_i is not along the path from the base to p, then dp/d(theta_i) will be a zero vector
    std::list<int> joints_list; // a list to store the joints along the path
    std::list<int>::iterator it;    // the list iterator
    
    // loop in the same approach as in getWorldCoordinates()
    RBJoint* joint_of_p = rb->pJoint;
    int qIndex = getQIdxForJoint(joint_of_p);
    joints_list.push_back(qIndex);
    while(qIndex > 5) {
        qIndex = getParentQIdxOf(qIndex);
        joints_list.push_back(qIndex);
    }

    // loop over each joint of the quadruped robot
    for (int qIndex = 6; qIndex < q.size(); qIndex++) {
        // check if the joint is along the path from the base to p
        // use std::find(), if iterator points to end, then the value we look for is not in the list
        it = std::find(joints_list.begin(), joints_list.end(), qIndex);
        
        // q[qIndex] is not a joint along the path
        if (it == joints_list.end()) {
            dpdq.block(0, qIndex, 3, 1) = V3D(0, 0, 0);
        }
        // q[qIndex] is a joint along the path
        // similar to the second step
        else {
            // obtain p_in_qIndex_frame
            RBJoint* joint_qIndex = getJointForQIdx(qIndex);
            P3D pos_of_qIndex_joint_in_local_frame = joint_qIndex->cJPos;
            RB* rb_of_qIndex_joint = joint_qIndex->child;
            P3D pos_of_qIndex_joint_in_world_frame = 
                getWorldCoordinates(pos_of_qIndex_joint_in_local_frame, rb_of_qIndex_joint);
            V3D p_in_qIndex_frame = V3D(pos_of_qIndex_joint_in_world_frame, pInWorld);
            
            // cross product
            V3D joint_axis_in_world_frame = getWorldCoordsAxisForQ(qIndex);
            V3D qIndex_col_of_dpdq = joint_axis_in_world_frame.cross(p_in_qIndex_frame);
            dpdq.block(0, qIndex, 3, 1) = qIndex_col_of_dpdq;
        }
    }

}

// estimates the linear jacobian dp/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(
    const P3D &p, RB *rb, Matrix &dpdq) {
    resize(dpdq, 3, (int)q.size());

    for (int i = 0; i < q.size(); i++) {
        double val = q[i];
        double h = 0.0001;

        // TODO: Ex. 2-1 Inverse Kinematics - Jacobian by Finite Difference
        // compute Jacobian matrix dpdq_i by FD and fill dpdq
        q[i] = val + h;
        // P3D p_p;  // TODO: fix this: p(qi + h);
        // obtain FK_i(q + h_j), where i is based on the for loop, and q + h_j is already implemented above
        P3D p_p = getWorldCoordinates(p, rb);

        q[i] = val - h;
        // P3D p_m;  // TODO: fix this: p(qi - h)
        // obtain FK_i(q - h_j), where i is based on the for loop, and q - h_j is already implemented above
        P3D p_m = getWorldCoordinates(p, rb);

        // V3D dpdq_i(0, 0, 0);  // TODO: fix this: compute derivative dp(q)/dqi
        // J_p,i,j = (FK_i(q + h_j) - FK_i(q - h_j)) / (2*h), and note that this is a P3D value, thus need to be transformed into V3D
        V3D dpdq_i = V3D((p_p - p_m) / (2*h));

        // set Jacobian matrix components
        dpdq(0, i) = dpdq_i[0];
        dpdq(1, i) = dpdq_i[1];
        dpdq(2, i) = dpdq_i[2];

        // finally, we don't want to change q[i] value. back to original value.
        q[i] = val;
    }
}

}  // namespace crl
