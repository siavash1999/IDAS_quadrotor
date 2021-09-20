#include <algorithm>
#include <string>

#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "uav_dynamics_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(UAVDynamicsPlugin)


/////////////////////////////////////////////////
void UAVDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "UAVDynamicsPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "UAVDynamicsPlugin _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;

    // SDF file must have element type:
    if (_sdf->HasElement("type"))
  	    this->type = _sdf->Get<std::string>("type");

    if (this->type == "thrust")
    {
        if (_sdf->HasElement("ct"))
            this->ct = _sdf->Get<double>("ct");

        if (_sdf->HasElement("clockwise"))
      	    this->clockwise = _sdf->Get<bool>("clockwise");

        if (_sdf->HasElement("propeller_link"))
        {
            sdf::ElementPtr elem = _sdf->GetElement("propeller_link");
            // GZ_ASSERT(elem, "Element link_name doesn't exist!");
            std::string linkName = elem->Get<std::string>();
            this->propeller_link = this->model->GetLink(linkName);
            // GZ_ASSERT(this->link, "Link was NULL");

            if (!this->propeller_link)
            {
                gzerr << "Link with name[" << linkName << "] not found. "
                      << "The UAVDynamicsPlugin will not generate forces\n";
            }
            else
            {
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&UAVDynamicsPlugin::OnUpdate, this));
            }
        }
    }
    else if(this->type == "drag")
    {
        if (_sdf->HasElement("cd"))
            this->cd = _sdf->Get<double>("cd");

        if (_sdf->HasElement("frame_link"))
        {
            sdf::ElementPtr elem = _sdf->GetElement("frame_link");
            // GZ_ASSERT(elem, "Element link_name doesn't exist!");
            std::string linkName = elem->Get<std::string>();
            this->frame_link = this->model->GetLink(linkName);
            // GZ_ASSERT(this->link, "Link was NULL");

            if (!this->frame_link)
            {
                gzerr << "Link with name[" << linkName << "] not found. "
                      << "The UAVDynamicsPlugin will not generate forces\n";
            }
            else
            {
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&UAVDynamicsPlugin::OnUpdate, this));
            }
        }
    }
}

/////////////////////////////////////////////////
void UAVDynamicsPlugin::OnUpdate()
{
    if (this->type == "thrust")
    {
        GZ_ASSERT(this->propeller_link, "Link was NULL");
        // get angular velocity in body-fixed frame
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d vel = this->propeller_link->WorldAngularVel();
#else
        ignition::math::Vector3d vel = ignitionFromGazeboMath(
        this->propeller_link->GetWorldAngularVel();
#endif
        double velmag = vel.Length();
        ignition::math::Vector3d velI = vel.Normalize();

        if (velmag <= 0.01)
            return;

        // compute moment (torque)
        // if (clockwise) ignition::math::Vector3d moment = -1 * cm * velmag * velmag * velI;
	    // else ignition::math::Vector3d moment = cm * velmag * velmag * velI;

        // force about cg in body-fixed frame
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

        // apply thrust
        this->propeller_link->SetForce(thrust);
        //this->link->AddTorque(moment);
    }
    else if(this->type == "drag")
    {
        GZ_ASSERT(this->frame_link, "Link was NULL");
        // get linear velocity in body-fixed frame
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d vel = this->frame_link->RelativeLinearVel();
#else
        ignition::math::Vector3d vel = ignitionFromGazeboMath(
        this->frame_link->GetRelativeLinearVel();
#endif
        double velmag = vel.Length();
        ignition::math::Vector3d velI = vel.Normalize();

        if (velmag <= 0.01)
            return;

        // force about cg in body-fixed frame
        ignition::math::Vector3d drag;
        drag = -1 * cd * velmag * velmag * velI;

        // Correct for nan or inf
        drag.Correct();
        //moment.Correct();

        // apply drag
        this->frame_link->AddRelativeForce(drag);
    }
}
