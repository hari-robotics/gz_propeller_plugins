#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "ignition/math/Vector3.hh"
#include "sdf/Element.hh"

namespace gz_propeller_plugins {

// Using types from namespaces
// clang-format off
using Str              = std::string;
using Thread           = std::thread;
using Vector3d         = ignition::math::Vector3d;
using ElementPtr       = sdf::ElementPtr;
using ConnectionPtr    = gazebo::event::ConnectionPtr;
using ModelPtr         = gazebo::physics::ModelPtr;
using JointPtr         = gazebo::physics::JointPtr;
// clang-format on

// Typedefs
typedef struct {
    Str direction;
    double_t force_constant;
    double_t torque_constant;
    bool invert_z_axis;
} JointValue;

typedef std::map<JointPtr, JointValue> JointMap;

// Plugin Private Class implementation
class PropellerPrivate {
public:
    PropellerPrivate() {}
    virtual ~PropellerPrivate() {}

    ModelPtr parent_model_;
    ElementPtr current_element_;
    ConnectionPtr update_connection_;
    JointMap joints_;
    int8_t torque_sign_;

    void OnUpdate() {
        for (std::pair<JointPtr, JointValue> joint : joints_) {
            if (joint.second.direction == "CCW" || joint.second.direction == "ccw") {
                torque_sign_ = 1;
            } else if (joint.second.direction == "CW" || joint.second.direction == "cw") {
                torque_sign_ = -1;
            } else {
                std::cout << "Invalid direction. Plugin won't work." << std::endl;
                return;
            }

            // Alias name
            auto tmp_angular_vel = joint.first->GetVelocity(3);
            auto torque_constant = joint.second.torque_constant;
            auto force_constant = joint.second.force_constant;
            auto tmp_child_pose = joint.first->GetChild()->WorldPose().Rot();

            // Temp variables
            auto tmp_force = Vector3d(0, 0, force_constant * tmp_angular_vel * abs(tmp_angular_vel) * torque_sign_);
            auto tmp_inverse_torque = Vector3d(0, 0, -torque_constant * tmp_angular_vel * abs(tmp_angular_vel));

            // Apply force and torque to the propeller
            joint.first->GetParent()->AddForce(tmp_child_pose.RotateVector(tmp_force));

            // Apply inverse torque to the parent link
            joint.first->GetParent()->AddTorque(tmp_child_pose.RotateVector(tmp_inverse_torque));
        }
    }
};

// Plugin Implementation
class PropellerPlugin : public gazebo::ModelPlugin {
public:
    PropellerPlugin() : impl_(std::make_unique<PropellerPrivate>()) {}

    ~PropellerPlugin() {
        impl_->joints_.clear();
    }

    void Load(ModelPtr _model, ElementPtr _sdf) {
        if (!_model) {
            std::cout << "Missing model. Plugin won't work." << std::endl;
            return;
        }
        impl_->parent_model_ = _model;
        std::cout << "get model:" << _model->GetName() << std::endl;

        if (!_sdf->HasElement("joint")) {
            std::cout << "Missing joint. Plugin won't work." << std::endl;
            return;
        }
        impl_->current_element_ = _sdf->GetElement("joint");

        while (impl_->current_element_) {
            JointPtr pJoint;
            JointValue joints_value;

            if (!impl_->current_element_->HasAttribute("name")) {
                std::cout << "Missing joint name. Plugin won't work." << std::endl;
                return;
            }
            pJoint = _model->GetJoint(impl_->current_element_->Get<Str>("name"));

            if (!impl_->current_element_->HasAttribute("direction")) {
                std::cout << "Missing joint direction. Plugin won't work." << std::endl;
                return;
            }
            joints_value.direction = impl_->current_element_->Get<Str>("direction");

            if (!impl_->current_element_->HasAttribute("torque_constant")) {
                std::cout << "Missing joint torque constant. Default set to 0" << std::endl;
                joints_value.torque_constant = 0;
            }
            joints_value.torque_constant = impl_->current_element_->Get<double>("torque_constant");

            if (!impl_->current_element_->HasAttribute("force_constant")) {
                std::cout << "Missing joint force constant. Default set to 0" << std::endl;
                joints_value.force_constant = 0;
            }
            joints_value.force_constant = impl_->current_element_->Get<double>("force_constant");

            // Add joint config to the map
            impl_->joints_.insert(JointMap::value_type(pJoint, joints_value));
            impl_->current_element_ = impl_->current_element_->GetNextElement("joint");

            std::cout << "get segment: " << impl_->joints_.find(pJoint)->first->GetName() << std::endl;
        }
    }

    void Init() {
        impl_->update_connection_ =
            gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&PropellerPrivate::OnUpdate, impl_.get()));
    }

private:
    std::unique_ptr<PropellerPrivate> impl_;
};

GZ_REGISTER_MODEL_PLUGIN(PropellerPlugin)
}  // namespace gz_propeller_plugins
