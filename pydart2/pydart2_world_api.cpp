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
BodyNodePtr bn_ch;
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
    skel->getDof(1)->setPosition(120 * 3.14 / 180.0);
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
    global_number = 88;
    MSG << " [pydart_api] Added empty skel " << global_number << endl;
}


void WORLD(addRootCapsule)(){
    BallJoint::Properties properties;
    properties.mName = "root joint";
    properties.mRestPositions = Eigen::Vector3d::Constant(0.0f);
    properties.mSpringStiffnesses = Eigen::Vector3d::Constant(0.0f);
    properties.mDampingCoefficients = Eigen::Vector3d::Constant(5.0f);


    bn = skel->createJointAndBodyNodePair<BallJoint>(nullptr, properties, BodyNode::AspectProperties("root joint")).second;
    //bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

    // Make a shape for the Joint
    const double& R = 0.2; //m
    std::shared_ptr<EllipsoidShape> ball(new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());




    //Now make the body

    // Create a BoxShape to be used for both visualization and collision checking
    std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(0.2f, 0.2f, 1.0f)));

    // Create a shape node for visualization and collision checking
    shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 1.0 / 2.0);
    box_tf.translation() = center;
    shapeNode->setRelativeTransform(box_tf);

    // Move the center of mass to the center of the object
    bn->setLocalCOM(center);

    mBodyNodePtrs.push_back(bn);

    MSG << "Adding Root Capsule " << global_number << endl;
}




void WORLD(addCapsule)(int parent, float capsule_radius, float capsule_length, const char* const joint_type){

    std::string str_joint_type(joint_type);
    if (str_joint_type == "BALL"){
        MSG << str_joint_type << endl;
        BallJoint::Properties properties;
        properties.mName = "root joint";
        properties.mRestPositions = Eigen::Vector3d::Constant(0.0f);
        properties.mSpringStiffnesses = Eigen::Vector3d::Constant(0.0f);
        properties.mDampingCoefficients = Eigen::Vector3d::Constant(5.0f);
        bn = skel->createJointAndBodyNodePair<BallJoint>(nullptr, properties, BodyNode::AspectProperties("root joint")).second;
        // Make a shape for the Joint
        const double& R = 0.2; //m
        std::shared_ptr<EllipsoidShape> ball(new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
        shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    }
    else if (str_joint_type == "REVOLUTE"){
        MSG << str_joint_type << endl;
        // Set up the properties for the Joint
        RevoluteJoint::Properties properties;
        properties.mName = "next joint";
        properties.mAxis = Eigen::Vector3d::UnitY();
        properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 1.0f);
        properties.mRestPositions[0] = 0.0f;
        properties.mSpringStiffnesses[0] = 0.0f;
        properties.mDampingCoefficients[0] = 5.0f;
        // Create a new BodyNode, attached to its parent by a RevoluteJoint
        bn = skel->createJointAndBodyNodePair<RevoluteJoint>(mBodyNodePtrs[parent], properties, BodyNode::AspectProperties("next joint")).second;
        // Make a shape for the Joint
        const double R = 0.2 / 2.0;
        const double h = 0.2;
        std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));
        // Line up the cylinder with the Joint axis
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * 3.14 / 180.0, 0, 0));
        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
        shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
        shapeNode->setRelativeTransform(tf);
    }
    else {
        bn = skel->createJointAndBodyNodePair<FreeJoint>(nullptr).second;
    }

    MSG << "making body" << endl;
    //Now make the body

    // Set the geometry of the Body  // Create a BoxShape to be used for both visualization and collision checking
    std::shared_ptr<CapsuleShape> box(new CapsuleShape(capsule_radius, capsule_length));

    // Create a shape node for visualization and collision checking
    auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 1.0 / 2.0);
    box_tf.translation() = center;
    shapeNode->setRelativeTransform(box_tf);

    // Move the center of mass to the center of the object
    bn->setLocalCOM(center);
    mBodyNodePtrs.push_back(bn);
    MSG << "Adding Capsule " << global_number << endl;
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
    dart::simulation::WorldPtr world = GET_WORLD(wid);
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
