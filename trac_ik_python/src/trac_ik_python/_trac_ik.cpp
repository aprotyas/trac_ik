// Copyright (c) 2022, Abrar Rahman Protyasha.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <string>

#include "pybind11/pybind11.h"

#include "trac_ik/trac_ik.hpp"

namespace py = pybind11;

namespace trac_ik_python
{

class TRAC_IK_Wrapper : public TRAC_IK::TRAC_IK
{
  TRAC_IK_Wrapper(
    const std::string & base_link,
    const std::string & tip_link,
    const std::string & urdf_xml = "",
    double _maxtime = 0.005,
    double _eps = 1e-5,
    TRAC_IK::SolveType _type = TRAC_IK::SolveType::Speed)
  : TRAC_IK::TRAC_IK(base_link, tip_link, urdf_xml, _maxtime, _eps, _type)
  {
    // TODO, write a constructor that does more of the work for the user
  }
};

using TRAC_IK = TRAC_IK_Wrapper;

} // namespace trac_ik_python

PYBIND11_MODULE(_trac_ik, m) {
  m.doc() = "Python wrapper of the TRAC_IK API";

  py::enum_<TRAC_IK::SolveType> SolveType("SolveType")
  .value("Speed", TRAC_IK::SolveType::Speed)
  .value("Distance", TRAC_IK::SolveType::Distance)
  .value("Manip1", TRAC_IK::SolveType::Manip1)
  .value("Manip2", TRAC_IK::SolveType::Manip2);
  .export_values();

  py::class_<trac_ik_python::TRAC_IK>(m, "TRAC_IK")
  .def(py::init<
    const std::string &,
    const std::string &,
    const std::string &,
    double,
    double,
    SolveType>(),
    "Constructs the TRAC_IK object",
    py::arg("urdf_xml") = "",
    py::arg("_maxtime") = 0.005,
    py::arg("_eps") = 1e-5,
    py::arg("_type") = "Speed")
  .def_property("solvetype",
    &trac_ik_python::TRAC_IK::GetSolveType,
    &trac_ik_python::TRAC_IK::SetSolveType);
}
