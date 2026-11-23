#pragma once

#include "feedback_control.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

namespace ve33 {

inline void bind_feedback(nanobind::module_& module) {
    namespace nb = nanobind;
    nb::class_<ContactFeedback>(module, "ContactFeedback")
        .def(nb::init<Axes, Axes, Axes, double, double, double, double, double, double>())
        .def(nb::init<Axes, Axes, Axes, double, double, double, double, double, double,
                      std::vector<std::array<double, 2>>>())
        .def("set_axis_curve", &ContactFeedback::set_axis_curve)
        .def("set_friction", &ContactFeedback::set_friction)
        .def("damping", &ContactFeedback::damping)
        .def("friction", &ContactFeedback::friction)
        .def("plan", &ContactFeedback::plan);
    nb::class_<ControlState>(module, "ControlState")
        .def_ro("active", &ControlState::active)
        .def_ro("rearm_required", &ControlState::rearm_required)
        .def_ro("output_error", &ControlState::output_error)
        .def_ro("valid_nominal", &ControlState::valid_nominal)
        .def_ro("valid_sent", &ControlState::valid_sent)
        .def_ro("need_send", &ControlState::need_send)
        .def_ro("need_release", &ControlState::need_release)
        .def_ro("deadline", &ControlState::deadline)
        .def_ro("submitted_until", &ControlState::submitted_until)
        .def_ro("interruption_count", &ControlState::interruption_count)
        .def_ro("last_interruption", &ControlState::last_interruption)
        .def_ro("handoff_count", &ControlState::handoff_count)
        .def_ro("last_handoff_gap_ms", &ControlState::last_handoff_gap_ms)
        .def_ro("loaded_count", &ControlState::loaded_count)
        .def_ro("position_count", &ControlState::position_count)
        .def_ro("message", &ControlState::message)
        .def_ro("nominal", &ControlState::nominal)
        .def_ro("motor", &ControlState::motor)
        .def_ro("motor_kd", &ControlState::motor_kd)
        .def_ro("friction", &ControlState::friction)
        .def_ro("sent", &ControlState::sent)
        .def_ro("position", &ControlState::position)
        .def_ro("mapped_contact", &ControlState::mapped_contact)
        .def_ro("frame_weight", &ControlState::frame_weight)
        .def_ro("gate", &ControlState::gate)
        .def_ro("dq", &ControlState::dq);
    nb::class_<FeedbackControl>(module, "FeedbackControl")
        .def(nb::init<ContactFeedback, Axes, double, int64_t, int64_t, double, double, double, double>())
        .def("step", &FeedbackControl::step, nb::rv_policy::copy)
        .def("missing", &FeedbackControl::missing, nb::rv_policy::copy)
        .def("stop", &FeedbackControl::stop, nb::arg("reason") = "力反馈关闭", nb::rv_policy::copy)
        .def("accepted", &FeedbackControl::accepted)
        .def("failed", &FeedbackControl::failed)
        .def("released", &FeedbackControl::released)
        .def("state", &FeedbackControl::state, nb::rv_policy::copy);
}

} // namespace ve33
