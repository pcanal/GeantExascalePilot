//
//===------------------ GeantX --------------------------------------------===//
//
// Geant Exascale Pilot
//
// For the licensing terms see LICENSE file.
// For the list of contributors see CREDITS file.
// Copyright (C) 2019, Geant Exascale Pilot team,  All rights reserved.
//===----------------------------------------------------------------------===//
//
/**
 * @file
 * @brief
 */
//===----------------------------------------------------------------------===//
//

#pragma once

#include "Geant/core/Logger.hpp"
#include "Geant/core/Tuple.hpp"
#include "Geant/particles/Types.hpp"
#include "Geant/processes/Process.hpp"

#include "Geant/proxy/ProxyEmProcess.hpp"
#include "Geant/proxy/ProxyMollerScattering.hpp"

namespace geantx
{
class ProxyIonization : public ProxyEmProcess<ProxyIonization>
{
  friend class ProxyEmProcess<ProxyIonization>;
  ProxyMollerScattering *fModel = nullptr;    
public:
  // Enable/disable GetPhysicalInteractionLength (GPIL) functions
  static constexpr bool EnableAtRestGPIL    = false;
  static constexpr bool EnableAlongStepGPIL = false;
  static constexpr bool EnablePostStepGPIL  = true;
  // Enable/disable DoIt functions
  static constexpr bool EnableAtRestDoIt    = false;
  static constexpr bool EnableAlongStepDoIt = true;
  static constexpr bool EnablePostStepDoIt  = true;

  // for enable_if statements
  template <typename _Tp>
  static constexpr bool IsApplicable = std::is_base_of<Particle, _Tp>::value;
  
  // provide no specializations
  using specialized_types = std::tuple<>;
  
public:
  using this_type = ProxyIonization;
  
  ProxyIonization(){ fModel = new ProxyMollerScattering; }
  ~ProxyIonization() = default;
  
  double MacroscopicXSection(TrackState* _track)
  {  
    GEANT_THIS_TYPE_TESTING_MARKER("");
    //TODO: connectivity to Material
    double Z = 10;

    //TODO: implement MacroscopicCrossSection and CrossSectionPerAtom

    double xsection = fModel->CrossSectionPerAtom(Z, _track->fPhysicsState.fEkin);
    return xsection;
  }
  
  int FinalStateInteraction(TrackState* _track)
  {  
    GEANT_THIS_TYPE_TESTING_MARKER("");

    //update photon state and create an electron 
    int nsecondaries = fModel->SampleSecondaries(_track);

    return nsecondaries;
  }

};
}  // namespace geantx