#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE PropPlugin : public ModelPlugin
  {
    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double ct;
    
    protected: bool clockwise;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;
  };
}
#endif
