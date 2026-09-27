// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace law90_control_test {
struct PointOperands {double stiffness[8]{},sound[8]{},volume[8]{},length[8]{},rate[8][6]{},modulus[8][5]{};};
TL_SOLID18_HD inline void Capture(const c::Scratch& scratch,const c::History* accepted,PointOperands& output) {
  for(unsigned ip=0;ip<8;++ip) {
    const auto& point=scratch.force.staged.point[ip];
    output.stiffness[ip]=point.material.raw_stiffness_n_m;
    output.sound[ip]=point.material.point.sound_speed_m_s;
    output.volume[ip]=point.current_volume_m3;output.length[ip]=point.characteristic_length_m;
    for(unsigned k=0;k<6;++k)output.rate[ip][k]=scratch.force.kinematics.staged.point[ip].engineering_rate_per_s[k];
    const auto& material=scratch.force.staged.proposed_history.material();
    law::PointHistory history;
    history.unloading_factor=1;history.effective_modulus_pa=material.updated().young_pa;
    if(accepted)history=accepted->native_history().data().point[ip].point;
    law::PointKinematics input;
    for(unsigned k=0;k<6;++k) {
      input.total_b_minus_i_engineering[k]=scratch.force.kinematics.staged.point[ip].selected_left_cauchy_green_minus_identity[k]*(k<3?1.0:2.0);
      input.engineering_rate_s_inverse[k]=output.rate[ip][k];
    }
    law::point_detail::Kinematics kinematics;
    law::point_detail::ResolveKinematics(input,kinematics);
    law::PointResult response;response.history=history;law::point_detail::CurveResponse curve;
    law::point_detail::CurvePass(material,kinematics,response.history,curve);
    const bool unloading=law::point_detail::UpdateLoading(kinematics,curve,response);
    const bool explicit_curve=material.reader().loading_flag==1;
    law::point_detail::CurvePass(material,kinematics,response.history,curve,explicit_curve);
    if(!explicit_curve)law::point_detail::UpdatePath(material,kinematics,unloading,curve,response.history);
    output.modulus[ip][0]=kinematics.strain_norm;output.modulus[ip][1]=curve.stress_norm;
    output.modulus[ip][2]=history.effective_modulus_pa;output.modulus[ip][3]=material.updated().young_pa;
    output.modulus[ip][4]=material.updated().maximum_modulus_pa;
  }
}
template<class Case>
void WriteDiagnostics(const std::vector<Case>& cases,const std::vector<std::array<double,357>>& native,const std::vector<std::array<double,40>>& native_modulus) {
  const char* path=std::getenv("ROBO_LAW90_CONTROL_DIAGNOSTIC");if(!path)return;
  if(std::filesystem::exists(path))throw std::runtime_error("Diagnostic output must be new");
  std::ofstream out(path);out<<std::setprecision(17);
  out<<"case,step,ip,young,max_modulus";
  for(unsigned k=0;k<20;++k)out<<",gpu_h"<<k;
  for(unsigned k=0;k<20;++k)out<<",native_h"<<k;
  out<<",gpu_sti,native_sti,gpu_sound,native_sound,gpu_volume,native_volume,gpu_length,native_length";
  for(unsigned k=0;k<6;++k)out<<",gpu_rate"<<k;
  for(unsigned k=0;k<5;++k)out<<",gpu_modulus"<<k;
  for(unsigned k=0;k<5;++k)out<<",native_modulus"<<k;out<<'\n';
  for(unsigned row=0;row<cases.size();++row)for(unsigned step=0;step<=Case::steps;++step)for(unsigned ip=0;ip<8;++ip) {
    const auto& history=cases[row].output[step].proposed_history.native_history();
    const auto& material=history.material().updated();double values[20];
    law90_force_test::PackHistory(history.data().point[ip],values);
    out<<row<<','<<step<<','<<ip<<','<<material.young_pa<<','<<material.maximum_modulus_pa;
    for(double value:values)out<<','<<value;
    const auto* expected=native[row*(Case::steps+1)+step].data()+40*ip;
    for(unsigned k=0;k<20;++k)out<<','<<expected[k];
    const auto& op=cases[row].operands[step];
    out<<','<<op.stiffness[ip]<<','<<expected[36]<<','<<op.sound[ip]<<','<<expected[26]
       <<','<<op.volume[ip]<<','<<expected[37]<<','<<op.length[ip]<<','<<expected[39];
    for(double rate:op.rate[ip])out<<','<<rate;
    for(double value:op.modulus[ip])out<<','<<value;
    for(unsigned k=0;k<5;++k)out<<','<<native_modulus[row*(Case::steps+1)+step][5*ip+k];out<<'\n';
  }
  out.close();if(!out)throw std::runtime_error("Diagnostic write failed");
}
} // namespace law90_control_test
