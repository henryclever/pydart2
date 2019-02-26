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
#include "pydart2_world_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_draw.h"

using namespace pydart;
using namespace dart::simulation;
using namespace dart::dynamics;
SkeletonPtr skel;
BodyNodePtr bn;
std::vector<BodyNodePtr> mBodyNodePtrs;
int global_number = 5;



////////////////////////////////////////////////////////////////////////////////
// World
int createWorld(double timestep) {
    return Manager::createWorld(timestep);
}

int createWorldFromSkel(const char* const path) {
    int wid = Manager::createWorldFromSkel(path);
    // MSG << " [pydart2_api] # Skeletons in " << path << " = " << numSkeletons(wid) << "\n";
    return wid;
}

void destroyWorld(int wid) {
    mBodyNodePtrs.clear();
    Manager::destroyWorld(wid);
}



int WORLD(addSkeleton)(int wid, const char* const path) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    std::string strpath(path);
    std::string ext = strpath.substr(strpath.length() - 4);
    boost::algorithm::to_lower(ext);
    MSG << "[pydart_api] " << path << endl;
    SkeletonPtr skel = NULL;
    if (ext == ".sdf") {
        MSG << " [pydart_api] parse as SDF (ext: " << ext << ")" << endl;
        skel = dart::utils::SdfParser::readSkeleton(path);
    } else if (ext == "urdf") {
        MSG << " [pydart_api] parse as URDF (ext: " << ext << ")" << endl;
        dart::utils::DartLoader urdfLoader;
        skel = urdfLoader.parseSkeleton(path);
    } else if (ext == ".vsk") {
        MSG << " [pydart_api] parse as VSK (ext: " << ext << ")" << endl;
        skel = dart::utils::VskParser::readSkeleton(path);
    } else {
        ERR << " [pydart_api] bad extension (ext: " << ext << ")" << endl;
        return -1;
    }

    MSG << " [pydart_api] skel [" << path << "]" << endl;

    dart::simulation::WorldPtr world = GET_WORLD(wid);
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);
    return id;


}


int WORLD(addPySkeleton)(int wid){
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);
    return id;
}


void WORLD(addEmptySkeleton)(const char* const name) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    std::string strname(name);
    skel = Skeleton::create(strname);
    MSG << " [pydart_api] Added empty skel "  << endl;
}


void WORLD(testFilter)(int wid)
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
    auto pair2 = body1->createChildJointAndBodyNodePair<RevoluteJoint>();
    auto* body2 = pair2.second;
    body2->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

    // Create a world and add the created skeleton
    //auto world = std::make_shared<dart::simulation::World>();
    auto world = GET_WORLD(wid);
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
    bodyNodeFilter->addBodyNodePairToBlackList(body1, body2);
    cout << group->collide(option) << endl;
    bodyNodeFilter->addBodyNodePairToBlackList(body0, body2);
    cout << group->collide(option) << endl;
    //bodyNodeFilter->removeBodyNodePairFromBlackList(body0, body1);
    //cout << group->collide(option) << endl;
}




void WORLD(addCapsule)(int parent, float capsule_radius, float capsule_length, float cap_rot1, float cap_rot2, float cap_rot3, float cap_offsetX, float cap_offsetY, float cap_offsetZ, float joint_locX, float joint_locY, float joint_locZ, const char* const joint_type, const char* const joint_name){

    BodyNodePtr parentNode = nullptr;
    if (parent >= 0){
        parentNode = mBodyNodePtrs[parent];
    }

    std::string str_joint_type(joint_type);
    std::string str_joint_name(joint_name);

    if (str_joint_type == "BALL"){
        BallJoint::Properties properties;
        properties.mName = str_joint_name;
        properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(joint_locX, joint_locY, joint_locZ); //joint location
        properties.mRestPositions = Eigen::Vector3d::Constant(0.0f);
        properties.mSpringStiffnesses = Eigen::Vector3d::Constant(0.0f);
        bn = skel->createJointAndBodyNodePair<BallJoint>(parentNode, properties, BodyNode::AspectProperties(str_joint_name)).second;
        // Make a shape for the Joint
        const double& R = capsule_radius * 2 + 0.01; //m
        std::shared_ptr<EllipsoidShape> ball(new EllipsoidShape(Eigen::Vector3d(R, R, R)));
        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    }
    else if (str_joint_type == "REVOLUTE_X" or str_joint_type == "REVOLUTE_Y" or str_joint_type == "REVOLUTE_Z"){
        RevoluteJoint::Properties properties;
        properties.mName = str_joint_name;
        if (str_joint_type == "REVOLUTE_X"){
            properties.mAxis = Eigen::Vector3d::UnitX();}
        else if (str_joint_type == "REVOLUTE_Y"){
            properties.mAxis = Eigen::Vector3d::UnitY();}
        else if (str_joint_type == "REVOLUTE_Z"){
            properties.mAxis = Eigen::Vector3d::UnitZ();}
        properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(joint_locX, joint_locY, joint_locZ); //joint location
        properties.mRestPositions[0] = 0.0f;
        properties.mSpringStiffnesses[0] = 0.0f;
        bn = skel->createJointAndBodyNodePair<RevoluteJoint>(parentNode, properties, BodyNode::AspectProperties(str_joint_name)).second;
        // Make a shape for the Joint
        const double R = capsule_radius + 0.01;
        const double h = capsule_radius * 2;
        std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));
        // Line up the cylinder with the Joint axis
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        if (str_joint_type == "REVOLUTE_X"){
            tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0, 90.0 * 3.14 / 180.0, 0));}
        else if (str_joint_type == "REVOLUTE_Y"){
            tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * 3.14 / 180.0, 0, 0));}
        else if (str_joint_type == "REVOLUTE_Z"){
            tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0, 0, 90.0 * 3.14 / 180.0));}
        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());
        shapeNode->setRelativeTransform(tf);
    }
    else {
        FreeJoint::Properties properties;
        properties.mName = str_joint_name;
        properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(joint_locX, joint_locY, joint_locZ); //joint location
        properties.mRestPositions[0] = 0.0f;
        //properties.mRestPositions = Eigen::Vector3d::Constant(0.0f);
        //properties.mSpringStiffnesses = Eigen::Vector3d::Constant(0.0f);
        bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr,  properties, BodyNode::AspectProperties(str_joint_name)).second;
        // Make a shape for the Joint
        const double& R = capsule_radius * 2 + 0.01; //m
        std::shared_ptr<EllipsoidShape> ball(new EllipsoidShape(Eigen::Vector3d(R, R, R)));
        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    }


    //Now make the body

    // Set the geometry of the Body  // Create a BoxShape to be used for both visualization and collision checking
    std::shared_ptr<CapsuleShape> capsule(new CapsuleShape(capsule_radius, capsule_length));

    // Create a shape node for visualization and collision checking
    auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(capsule);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Set the location of the shape node
    Eigen::Isometry3d capsule_tf(Eigen::Isometry3d::Identity());
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(cap_rot1, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(cap_rot2, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(cap_rot3, Eigen::Vector3d::UnitY());


    Eigen::Vector3d center = Eigen::Vector3d(cap_offsetX, cap_offsetY, cap_offsetZ);
    capsule_tf.translation() = center;
    capsule_tf.rotate(m);
    //cout << m << endl;

    shapeNode->setRelativeTransform(capsule_tf);


    //MSG << node_transform << endl;
    // Move the center of mass to the center of the object
    bn->setLocalCOM(center);
    mBodyNodePtrs.push_back(bn);
    MSG << "Adding Capsule " << global_number << endl;
}


void WORLD(addWeldBox)(float width, float length, float height, float joint_locX, float joint_locY, float joint_locZ, float box_rot1, float box_rot2, float box_rot3, const char* const joint_name){

    BodyNodePtr parentNode = nullptr;

    std::string str_joint_name(joint_name);


    WeldJoint::Properties properties;
    properties.mName = str_joint_name;
    properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(joint_locX, joint_locY, joint_locZ); //joint location
    bn = skel->createJointAndBodyNodePair<WeldJoint>(nullptr,  properties, BodyNode::AspectProperties(str_joint_name)).second;

    //Now make the body

    // Set the geometry of the Body  // Create a BoxShape to be used for both visualization and collision checking
    std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(width, length, height)));

    // Create a shape node for visualization and collision checking
    auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector4d(0.95, 0.95, 0.95, 1.0));

    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(box_rot1, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(box_rot2, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(box_rot3, Eigen::Vector3d::UnitY());

    Eigen::Vector3d center = Eigen::Vector3d(joint_locX, joint_locY, joint_locZ);
    box_tf.translation() = center;
    box_tf.rotate(m);

    shapeNode->setRelativeTransform(box_tf);

    bn->setLocalCOM(center);
    mBodyNodePtrs.push_back(bn);
    MSG << "Adding Box " << global_number << endl;
}




int WORLD(getNumSkeletons)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getNumSkeletons();
}

void WORLD(reset)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->reset();
    for (size_t skid = 0; skid < world->getNumSkeletons(); skid++) {
        dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
        skel->resetCommands();
        skel->resetPositions();
        skel->resetVelocities();
        skel->resetAccelerations();
        skel->resetGeneralizedForces();
        skel->clearExternalForces();
        skel->clearConstraintImpulses();
    }
}

void WORLD(step)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->step();
}

void WORLD(checkCollision)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    // world->checkCollision();
    world->getConstraintSolver()->solve();
}

void WORLD(render)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawWorld(ri, world);
}

////////////////////////////////////////
// World::Time Functions

void WORLD(setTimeStep)(int wid, double _timeStep) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->setTimeStep(_timeStep);
}

double WORLD(getTimeStep)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getTimeStep();
}

void WORLD(setTime)(int wid, double _time) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->setTime(_time);
}

double WORLD(getTime)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getTime();
}

int WORLD(getSimFrames)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getSimFrames();
}

int WORLD(getIndex)(int wid, int _index) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getIndex(_index);
}

////////////////////////////////////////
// World::Property Functions
void WORLD(setGravity)(int wid, double inv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD    (wid);
    world->setGravity(read(inv3, 3));
}

void WORLD(getGravity)(int wid, double outv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    write(world->getGravity(), outv3);
}

////////////////////////////////////////
// World::CollisionDetector Functions
void WORLD(setCollisionDetector)(int wid, int detector_type) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    if (detector_type == 0) {
        solver->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
    } else if (detector_type == 1) {
      solver->setCollisionDetector(dart::collision::FCLCollisionDetector::create());
    }
#ifdef PYDART2_BULLET_FOUND
    else if (detector_type == 2) {
      solver->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    }
#endif
#ifdef PYDART2_ODE_FOUND
    else if (detector_type == 3) {
        solver->setCollisionDetector(dart::collision::OdeCollisionDetector::create());
    }
#endif
    else {
        ERR << " [pydart_api] unknown detector_type" << std::endl;
    }
}

int WORLD(getCollisionDetector)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    dart::collision::CollisionDetector* detector = solver->getCollisionDetector().get();
    if (dynamic_cast<dart::collision::DARTCollisionDetector*>(detector)) {
        return 0;
    } else if (dynamic_cast<dart::collision::FCLCollisionDetector*>(detector)) {
        return 1;
    }
#ifdef PYDART2_BULLET_FOUND
    else if (dynamic_cast<dart::collision::BulletCollisionDetector*>(detector)) {
        return 2;
    }
#endif
#ifdef PYDART2_ODE_FOUND
    else if (dynamic_cast<dart::collision::OdeCollisionDetector*>(detector)) {
        return 3;
    }
#endif
    return -1;
}

////////////////////////////////////////
// World::Constraint Functions
void WORLD(removeAllConstraints)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->getConstraintSolver()->removeAllConstraints();
}
