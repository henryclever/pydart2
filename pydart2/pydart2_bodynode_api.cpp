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
#include "pydart2_bodynode_api.h"
#include "pydart2_draw.h"


#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>


using namespace pydart;
using namespace dart::dynamics;



/// An arrow shape that we will use to visualize applied forces
//std::shared_ptr<ArrowShape> mArrow;


const double default_height = 0.1; // m
const double default_width = 0.05;  // m
const double default_depth = 0.05; // m

//ArrowShape::Properties properties;
//properties.mRadius = 0.05;

std::shared_ptr<ArrowShape> mArrow(new ArrowShape(
              Eigen::Vector3d(0.0, 0.0, 0.0), //tail
              Eigen::Vector3d(0.0, 0.0, 0.0), //head
              ArrowShape::Properties(0.01, 1.8, 0.5), //radius, head rad scale, scale of head length w.r.t total length
              dart::Color::Orange(1.0))); //color



std::vector<std::shared_ptr<ArrowShape>> mArrows;

//std::shared_ptr<ArrowShape> mArrow;
//mArrow->setProperties( ArrowShape::Properties(0.01, 1.8, 0.5) ) ;

////////////////////////////////////////////////////////////////////////////////
// BodyNode
const char* BODY(getName)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getName().c_str();
}

////////////////////////////////////////
// BodyNode::Structure Functions
int BODY(getParentBodyNode)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    dart::dynamics::BodyNode* parent = body->getParentBodyNode();
    if (parent == NULL) {
        return -1;
    } else {
        return parent->getIndexInSkeleton();
    }
}

int BODY(getNumChildBodyNodes)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumChildBodyNodes();
}


int BODY(getChildBodyNode)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    dart::dynamics::BodyNode* child = body->getChildBodyNode(_index);
    if (child == NULL) {
        return -1;
    } else {
        return child->getIndexInSkeleton();
    }
}

////////////////////////////////////////
// BodyNode::Joint and Dof Functions
int BODY(getParentJoint)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    if (body->getParentJoint()) {
        return body->getParentJoint()->getJointIndexInSkeleton();
    } else {
        return -1;
    }
}


int BODY(getNumChildJoints)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumChildJoints();
}


int BODY(getChildJoint)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getChildJoint(_index)->getJointIndexInSkeleton();
}


int BODY(getNumDependentDofs)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumDependentDofs();
}


int BODY(getDependentDof)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getDependentDof(_index)->getIndexInSkeleton();
}

////////////////////////////////////////
// BodyNode::Shape
int BODY(getNumShapeNodes)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    // dart::dynamics::ShapePtr shape = body->getVisualizationShape(0);
    return body->getShapeNodes().size();
}

////////////////////////////////////////
// BodyNode::Index Functions
int BODY(getIndexInSkeleton)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getIndexInSkeleton();
}


int BODY(getIndexInTree)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getIndexInTree();
}


int BODY(getTreeIndex)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getTreeIndex();
}



////////////////////////////////////////
// BodyNode::Property Functions
void BODY(setGravityMode)(int wid, int skid, int bid, bool _gravityMode) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setGravityMode(_gravityMode);
}


bool BODY(getGravityMode)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getGravityMode();
}


bool BODY(isCollidable)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->isCollidable();
}


void BODY(setCollidable)(int wid, int skid, int bid, bool _isCollidable) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setCollidable(_isCollidable);
}


////////////////////////////////////////
// BodyNode::Inertia Functions
double BODY(getMass)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getMass();
}


void BODY(setMass)(int wid, int skid, int bid, double mass) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setMass(mass);
}


void BODY(getInertia)(int wid, int skid, int bid, double outv33[3][3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    body->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    outv33[0][0] = Ixx;    outv33[1][1] = Iyy;    outv33[2][2] = Izz;
    outv33[0][1] = Ixy;    outv33[1][0] = Ixy;
    outv33[0][2] = Ixz;    outv33[2][0] = Ixz;
    outv33[1][2] = Iyz;    outv33[2][1] = Iyz;
}


void BODY(setInertia)(int wid, int skid, int bid, double inv33[3][3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    Ixx = inv33[0][0]; Iyy = inv33[1][1]; Izz = inv33[2][2];
    Ixy = inv33[0][1]; Ixz = inv33[0][2];
    Iyz = inv33[1][2];

    body->setMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
}


////////////////////////////////////////
// BodyNode::Momentum Functions
void BODY(getLocalCOM)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getLocalCOM(), outv3);
}


void BODY(getCOM)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOM(), outv3);
}


void BODY(getCOMLinearVelocity)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMLinearVelocity(), outv3);
}


void BODY(getCOMSpatialVelocity)(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMSpatialVelocity(), outv6);
}


void BODY(getCOMLinearAcceleration)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMLinearAcceleration(), outv3);
}


void BODY(getCOMSpatialAcceleration)(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMSpatialAcceleration(), outv6);
}

////////////////////////////////////////
// BodyNode::Friction and Restitution Functions
void BODY(setFrictionCoeff)(int wid, int skid, int bid, double _coeff) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setFrictionCoeff(_coeff);
}


double BODY(getFrictionCoeff)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getFrictionCoeff();
}


void BODY(setRestitutionCoeff)(int wid, int skid, int bid, double _coeff) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setRestitutionCoeff(_coeff);
}


double BODY(getRestitutionCoeff)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getRestitutionCoeff();
}


////////////////////////////////////////
// BodyNode::Transforms
void BODY(getTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getTransform(), outv44);
}


void BODY(getWorldTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getWorldTransform(), outv44);
}


void BODY(getRelativeTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getRelativeTransform(), outv44);
}

////////////////////////////////////////
// BodyNode::Ext Force and Torque
void BODY(addExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->addExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);
}

void BODY(setExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);
}

void BODY(addExtForceWithArrow)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], double ATX, double ATY, double ATZ, double AHX, double AHY, double AHZ, int arrow_index, bool _isForceLocal, bool _isOffsetLocal) {
    BodyNode* body = GET_BODY(wid, skid, bid);
    body->addExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);



    mArrows.push_back(mArrow);

    mArrows[arrow_index]->setPositions(
            Eigen::Vector3d(ATX, ATY, ATZ),
            Eigen::Vector3d(AHX, AHY, AHZ));

    //auto shapeNodes = body->getShapeNodesWith<VisualAspect>();
    //shapeNodes[1]->getVisualAspect()->setColor(dart::Color::Green());
    body->createShapeNodeWith<VisualAspect>(mArrows[arrow_index]);

}

void BODY(setExtForceWithArrow)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], double ATX, double ATY, double ATZ, double AHX, double AHY, double AHZ, int arrow_index, bool _isForceLocal, bool _isOffsetLocal) {
    BodyNode* body = GET_BODY(wid, skid, bid);
    body->setExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);
    auto visualShapeNodes = body->getShapeNodesWith<VisualAspect>();
    // If we have three visualization shapes, that means the arrow is
    // attached. We should remove it in case this body is no longer
    // experiencing a force
    if(visualShapeNodes.size() == 3u)
    {
        visualShapeNodes[2]->remove();
    }

    mArrows[arrow_index]->setPositions(
            Eigen::Vector3d(ATX, ATY, ATZ),
            Eigen::Vector3d(AHX, AHY, AHZ));

    body->createShapeNodeWith<VisualAspect>(mArrows[arrow_index]);


}


void BODY(addExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->addExtTorque(read(inv3, 3), _isLocal);
}


void BODY(setExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setExtTorque(read(inv3, 3), _isLocal);
}

////////////////////////////////////////
// BodyNode::Jacobian Functions
void BODY(getJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getJacobian(read(inv3, 3)), outm);
}

void BODY(getLinearJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getLinearJacobian(read(inv3, 3)), outm);
}


void BODY(getAngularJacobian)(int wid, int skid, int bid, double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getAngularJacobian(), outm);
}


void BODY(getWorldJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getWorldJacobian(read(inv3, 3)), outm);
}


void BODY(getLinearJacobianDeriv)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getLinearJacobianDeriv(read(inv3, 3)), outm);
}


void BODY(getAngularJacobianDeriv)(int wid, int skid, int bid, double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getAngularJacobianDeriv(), outm);
}
