#include <algorithm>
#include <string>

#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "prop_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PropPlugin)


/////////////////////////////////////////////////
void PropPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "PropPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "PropPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;
  
  // SDF file must have elements: [ct, clockwise, link_name]
  if (_sdf->HasElement("ct"))
    this->ct = _sdf->Get<double>("ct");
  
  if (_sdf->HasElement("clockwise"))
  	this->clockwise = _sdf->Get<bool>("clockwise");
	  
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The PropPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PropPlugin::OnUpdate, this));
    }
  }
}

/////////////////////////////////////////////////
void PropPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get angular velocity in body-fixed frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->RelativeAngularVel();
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetRelativeAngularVel();
#endif
  double velmag = vel.Length();
  ignition::math::Vector3d velI = vel.Normalize();

  if (velmag <= 0.01)
    return;

  // compute moment (torque)
  // if (clockwise) ignition::math::Vector3d moment = -1 * cm * velmag * velmag * velI;
	// else ignition::math::Vector3d moment = cm * velmag * velmag * velI;

  // force about cg in inertial frame
  ignition::math::Vector3d thrust;
  if (this->clockwise == 1) {
		thrust = -1 * ct * velmag * velmag * velI;
	}
	else {
		thrust = ct * velmag * velmag * velI;
	}
	
  // Correct for nan or inf
  thrust.Correct();
  //moment.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddRelativeForce(thrust);
  //this->link->AddTorque(moment);
}
