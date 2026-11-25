#pragma once

#include "feedback_control.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

namespace ve33 {

template <size_t Dof>
inline void bind_feedback_for(nanobind::module_& module, const char* cf_name,
                              const char* state_name, const char* ctrl_name) {
    namespace nb = nanobind;
    using CF = ContactFeedbackT<Dof>;
    using CS = ControlStateT<Dof>;
    using FC = FeedbackControlT<Dof>;
    using Ax = AxesT<Dof>;
    nb::class_<CF>(module, cf_name)
        .def(nb::init<Ax, Ax, Ax, double, double, double, double, double, double>())
        .def(nb::init<Ax, Ax, Ax, double, double, double, double, double, double,
                      std::vector<std::array<double, 2>>>())
        .def("set_axis_curve", &CF::set_axis_curve)
        .def("set_friction", &CF::set_friction)
        .def("damping", &CF::damping)
        .def("friction", &CF::friction)
        .def("plan", &CF::plan);
    nb::class_<CS>(module, state_name)
        .def_ro("active", &CS::active)
        .def_ro("rearm_required", &CS::rearm_required)
        .def_ro("output_error", &CS::output_error)
        .def_ro("valid_nominal", &CS::valid_nominal)
        .def_ro("valid_sent", &CS::valid_sent)
        .def_ro("need_send", &CS::need_send)
        .def_ro("need_release", &CS::need_release)
        .def_ro("deadline", &CS::deadline)
        .def_ro("submitted_until", &CS::submitted_until)
        .def_ro("interruption_count", &CS::interruption_count)
        .def_ro("last_interruption", &CS::last_interruption)
        .def_ro("handoff_count", &CS::handoff_count)
        .def_ro("last_handoff_gap_ms", &CS::last_handoff_gap_ms)
        .def_ro("loaded_count", &CS::loaded_count)
        .def_ro("position_count", &CS::position_count)
        .def_ro("message", &CS::message)
        .def_ro("nominal", &CS::nominal)
        .def_ro("motor", &CS::motor)
        .def_ro("motor_kd", &CS::motor_kd)
        .def_ro("friction", &CS::friction)
        .def_ro("sent", &CS::sent)
        .def_ro("position", &CS::position)
        .def_ro("mapped_contact", &CS::mapped_contact)
        .def_ro("frame_weight", &CS::frame_weight)
        .def_ro("gate", &CS::gate)
        .def_ro("dq", &CS::dq);
    nb::class_<FC>(module, ctrl_name)
        .def(nb::init<CF, Ax, double, int64_t, int64_t, double, double, double, double>())
        .def("step", &FC::step, nb::rv_policy::copy)
        .def("missing", &FC::missing, nb::rv_policy::copy)
        .def("stop", &FC::stop, nb::arg("reason") = "力反馈关闭", nb::rv_policy::copy)
        .def("accepted", &FC::accepted)
        .def("failed", &FC::failed)
        .def("released", &FC::released)
        .def("state", &FC::state, nb::rv_policy::copy);
}

inline void bind_feedback(nanobind::module_& module) {
    bind_feedback_for<12>(module, "ContactFeedback", "ControlState", "FeedbackControl");
    module.attr("ContactFeedback12") = module.attr("ContactFeedback");
    module.attr("ControlState12") = module.attr("ControlState");
    module.attr("FeedbackControl12") = module.attr("FeedbackControl");
    bind_feedback_for<16>(module, "ContactFeedback16", "ControlState16", "FeedbackControl16");
}

} // namespace ve33
