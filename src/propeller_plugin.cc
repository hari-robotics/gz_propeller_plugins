#include <cmath>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/ModelState.hh"
#include "gazebo/physics/physics.hh"
#include "ignition/math/Vector3.hh"
#include "sdf/Element.hh"

namespace gz_propeller_plugins {
// clang-format off
using Str              = std::string;
using Thread           = std::thread;
using Vector3d         = ignition::math::Vector3d;
using ElementPtr       = sdf::ElementPtr;
using ConnectionPtr    = gazebo::event::ConnectionPtr;
using ModelPtr         = gazebo::physics::ModelPtr;
using JointPtr         = gazebo::physics::JointPtr;
// clang-format on


typedef struct {
    JointPtr ptr;
    int8_t dir_sign_;
    double_t force_constant;
    double_t torque_constant;
    bool invert_z_axis;
} JointValue;


class PropellerPrivate {
public:
    PropellerPrivate() {}
    virtual ~PropellerPrivate() {}

    ModelPtr parent_model_;
    ConnectionPtr update_connection_;
    std::vector<JointValue> joints_;

    void OnUpdate() {
        for (std::vector<JointValue>::iterator joint = joints_.begin(); joint != joints_.end(); joint++) {
            // Get propeller properties
            auto w_prop = joint->ptr->GetVelocity(3);
            auto C_t = joint->torque_constant;
            auto C_f = joint->force_constant;

            // Get transform matrix for propeller frame to body frame
            auto trans_world2prop = joint->ptr->GetChild()->WorldPose();
            auto trans_world2body = joint->ptr->GetParent()->WorldPose();
            auto trans_prop2body = trans_world2prop * trans_world2body.Inverse();

            // Force and torque relative to the propeller link
            auto f_prop = Vector3d(0, 0, C_f * w_prop * abs(w_prop) * joint->dir_sign_);
            auto t_prop = Vector3d(0, 0, -C_t * w_prop * abs(w_prop));

            // Force and torque relative to the body link
            auto f_body = trans_prop2body.Rot().RotateVector(f_prop);
            auto t_body = trans_prop2body.Rot().RotateVector(t_prop);

            // Apply force and torque to the connected parent link
            joint->ptr->GetParent()->AddRelativeForce(f_body);
            joint->ptr->GetParent()->AddRelativeTorque(t_body);
            // std::cout << "Prop Force: " << f_body << std::endl;
        }
        std::cout << "Body Force: " << joints_.begin()->ptr->GetParent()->RelativeForce() << std::endl;
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
        // >>> load model >>>
        if (!_model) {
            std::cout << "Missing model. Plugin won't work." << std::endl;
            return;
        }
        impl_->parent_model_ = _model;
        std::cout << "get model:" << _model->GetName() << std::endl;
        // <<< load model <<<

        // >>> load joint >>>
        if (!_sdf->HasElement("joint")) {
            std::cout << "Missing joint. Plugin won't work." << std::endl;
            return;
        }
        auto current_element = _sdf->GetElement("joint");
        // <<< load joint <<<

        // >>> load propeller parameters >>>
        while (current_element) {
            JointValue joints_value;

            // >>> Propeller joint >>>
            if (!current_element->HasAttribute("name")) {
                std::cout << "Missing joint name. Plugin won't work." << std::endl;
                return;
            }
            joints_value.ptr = _model->GetJoint(current_element->Get<Str>("name"));
            // <<< Propeller joint <<<

            // >>> Propeller direction >>>
            if (!current_element->HasAttribute("direction")) {
                std::cout << "Missing joint direction. Plugin won't work." << std::endl;
                return;
            }

            auto direction = current_element->Get<Str>("direction");

            if (direction == "CCW" || direction == "ccw") { // 3,4,5,6
                joints_value.dir_sign_ = 1; // vel + -> force + / vel - -> force -
            } else if (direction == "CW" || direction == "cw") { // 1,2,7,8
                joints_value.dir_sign_ = -1; // vel + -> force - / vel - -> force +
            } else {
                std::cout << "Invalid direction. Plugin won't work." << std::endl;
                return;
            }
            // <<< Propeller direction <<<
            
            // >>> Propeller torque constant >>>
            if (!current_element->HasAttribute("torque_constant")) {
                std::cout << "Missing joint torque constant. Default set to 0" << std::endl;
                joints_value.torque_constant = 0;
            }
            joints_value.torque_constant = current_element->Get<double>("torque_constant");
            // <<< Propeller torque constant <<<

            // >>> Propeller force constant >>>
            if (!current_element->HasAttribute("force_constant")) {
                std::cout << "Missing joint force constant. Default set to 0" << std::endl;
                joints_value.force_constant = 0;
            }
            joints_value.force_constant = current_element->Get<double>("force_constant");
            // <<< Propeller force constant <<<

            // >>> Save config and search for next propeller >>>
            impl_->joints_.emplace_back(joints_value);
            current_element = current_element->GetNextElement("joint");
            std::cout << "get segment: " << joints_value.ptr->GetName() << std::endl;
            // <<< Save config and search for next propeller <<<
        }
        // <<< load propeller parameters<<<
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
