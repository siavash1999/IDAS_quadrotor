#ifndef _GAZEBO_UAV_DYNAMICS_PLUGIN_HH_
#define _GAZEBO_UAV_DYNAMICS_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>

namespace gazebo
{
  /// \brief A plugin that simulates simplified and linearized thrust and drag forces for UAVs
    class GAZEBO_VISIBLE UAVDynamicsPlugin : public ModelPlugin
    {
        // Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

    	/// \brief type of force that plugin is supposed to simulate. (either "thrust" or "drag")
    	protected: std::string type;

        /// \brief Thrust Coefficient:
        /// Thrust = Ct*w^2
        protected: double ct;

        /// \brief Drag Coefficient
        /// Drag = Cd*V^2
        protected: double cd;

        /// \brief whether propeller rotates clockwise or not.
        protected: bool clockwise;

        /// \brief Pointer to propeller link.
        protected: physics::LinkPtr propeller_link;

        /// \brief Pointer to frame link.
        protected: physics::LinkPtr frame_link;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;
    };
}
#endif
