#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;


// Boost headers
#include <boost/algorithm/string.hpp>

#include "pydart2_manager.h"
#include "pydart2_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_draw.h"

#include "dart/collision/detail/UnorderedPairs.hpp"
detail::UnorderedPairs<dynamics::BodyNode> mBodyNodeBlackList2;
using namespace pydart;


////////////////////////////////////////////////////////////////////////////////
// Skeleton
void SKEL(render)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawSkeleton(ri, skel.get());
}

void SKEL(renderWithColor)(int wid, int skid, double inv4[4]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    Eigen::Vector4d color(inv4);
    // MSG << "color = " << color.transpose() << "\n";
    drawSkeleton(ri, skel.get(), color, false);
}

const char* SKEL(getName)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getName().c_str();
}

double SKEL(getMass)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getMass();
}

void SKEL(resetMomentum)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->resetVelocities();
    skel->resetAccelerations();
}

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->isMobile();
}

void SKEL(setMobile)(int wid, int skid, bool mobile) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setMobile(mobile);
}

bool SKEL(getSelfCollisionCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getSelfCollisionCheck();
}

void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setSelfCollisionCheck(enable);
}

void SKEL(testFilter)()
{
    using namespace dart::simulation;
    using namespace dart::dynamics;
    using namespace dart::collision;
    // Create two bodies skeleton. The two bodies are placed at the same position
    // with the same size shape so that they collide by default.
    auto skel = Skeleton::create();
    auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));
    auto pair0 = skel->createJointAndBodyNodePair<RevoluteJoint>(nullptr);
    auto* body0 = pair0.second;
    body0->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);
    auto pair1 = body0->createChildJointAndBodyNodePair<RevoluteJoint>();
    auto* body1 = pair1.second;
    body1->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

    // Create a world and add the created skeleton
    auto world = std::make_shared<dart::simulation::World>();
    auto constraintSolver = world->getConstraintSolver();
    constraintSolver->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    world->addSkeleton(skel);

    // Get the collision group from the constraint solver
    auto group = constraintSolver->getCollisionGroup();
    cout << group->getNumShapeFrames() << " " << 2u << endl;;

    // Default collision filter for Skeleton
    auto& option = constraintSolver->getCollisionOption();
    auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
    option.collisionFilter = bodyNodeFilter;

    // Test blacklist
    skel->enableSelfCollisionCheck();
    skel->enableAdjacentBodyCheck();
    bodyNodeFilter->addBodyNodePairToBlackList(body0, body1);
    cout << group->collide(option) << endl;
    bodyNodeFilter->removeBodyNodePairFromBlackList(body0, body1);
    cout << group->collide(option) << endl;

}

void SKEL(setCollisionFilter)(int wid, int skid, int bid1, int bid2, int bEnable) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    // Get collision detector
    WorldPtr world = Manager::world(wid);
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    solver->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    //dart::collision::CollisionOption* option = solver->getCollisionOption();
    //dart::collision::BodyNodeCollisionFilter* filter = solver->getCollisionFilter();
    auto& collisionOption = solver->getCollisionOption();
    auto group = world->getConstraintSolver()->getCollisionGroup();

    //auto bodyNodeFilter = collisionOption.collisionFilter;
    auto bodyNodeFilter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
    collisionOption.collisionFilter = bodyNodeFilter;
    cout << "wid: " << wid << "  skid: " << skid << " num bodynodes: " << skel->getNumBodyNodes() << endl;
    //cout <<  mBodyNodeBlackList << endl;
    BodyNode* body1 = skel->getBodyNode(bid1);//GET_BODY(wid, skid, bid1);
    BodyNode* body2 = skel->getBodyNode(bid2);//GET_BODY(wid, skid, bid2);
    bool enable = (bEnable != 0);
    if (enable) {
        bodyNodeFilter->addBodyNodePairToBlackList(body1, body2);
        cout << "blacklisted " << bid1 << " and " << bid2 << " " << group->collide(collisionOption) << endl;
        //bodyNodeFilter->needCollision(auto o1, auto o2);
    } else {
        bodyNodeFilter->removeBodyNodePairFromBlackList(body1, body2);
        cout << "rm from blacklist " << bid1 << " and " << bid2 << endl;
    }
}


bool SKEL(getAdjacentBodyCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getAdjacentBodyCheck();
}

void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setAdjacentBodyCheck(enable);
}

void SKEL(setRootJointToTransAndEuler)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::EulerJoint>();
    oldRoot->getParentJoint()->setName("root_r");
    // create a new root
    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
        skel->createJointAndBodyNodePair
        <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    dart::dynamics::Joint* newJoint = ret.first;
    newJoint->setName("root_t");
    dart::dynamics::BodyNode* newBody = ret.second;
    newBody->setMass(0.0);
    // rearrange the root joints
    oldRoot->moveTo(newBody);
}


void SKEL(setRootJointToWeld)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::WeldJoint>();

    // oldRoot->getParentJoint()->setName("root_r");
    // // create a new root
    // std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
    //     skel->createJointAndBodyNodePair
    //     <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    // dart::dynamics::Joint* newJoint = ret.first;
    // newJoint->setName("root_t");
    // dart::dynamics::BodyNode* newBody = ret.second;
    // newBody->setMass(0.0);
    // // rearrange the root joints
    // oldRoot->moveTo(newBody);
}


////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumBodyNodes();
}

int SKEL(getNumJoints)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumJoints();
}

int SKEL(getNumDofs)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumDofs();
}

int SKEL(getNumMarkers)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumMarkers();
}

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getPositions(), outv);
}

void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setPositions(read(inv, ndofs));
}

void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getVelocities(), outv);
}

void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setVelocities(read(inv, ndofs));
}

void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getAccelerations(), outv);
}

void SKEL(setForces)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setForces(read(inv, ndofs));
}

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq1 = read(inv1, indofs1);
    Eigen::VectorXd dq2 = read(inv2, indofs2);
    Eigen::VectorXd dq_diff = skel->getVelocityDifferences(dq1, dq2);
    write(dq_diff, outv);
}

void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd q1 = read(inv1, indofs1);
    Eigen::VectorXd q2 = read(inv2, indofs2);
    Eigen::VectorXd q_diff = skel->getPositionDifferences(q1, q2);
    write(q_diff, outv);
}

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionLowerLimits(), outv);
}

void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionUpperLimits(), outv);
}

void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceLowerLimits(), outv);
}

void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceUpperLimits(), outv);
}

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOM(), outv3);
}

void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearVelocity(), outv3);
}

void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearAcceleration(), outv3);
}

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write_matrix(skel->getMassMatrix(), outm);
}

void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getCoriolisAndGravityForces(), outv);
}

void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getConstraintForces(), outv);
}
