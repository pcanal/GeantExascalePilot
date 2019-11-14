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

#include "Geant/proxy/ProxyRandom.hpp"

namespace geantx
{
template <typename TEmProcess>
class ProxyEmProcess : public Process
{
protected:
  ProxyRandom *fRng = nullptr;

public:
  // Enable/disable GetPhysicalInteractionLength (GPIL) functions

  static constexpr bool EnableAtRestGPIL    = false;
  static constexpr bool EnableAlongStepGPIL = true;
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
  using this_type = ProxyEmProcess;
  
  ProxyEmProcess(){ fRng = new ProxyRandom; }
  ~ProxyEmProcess() = default;
  
  //mandatory methods
  double MacroscopicXSection(TrackState* _track)
  {
    return static_cast<TEmProcess *>(this)-> MacroscopicXSection(_track);
  }    

  //  int FinalStateInteraction(TrackState* _track)
  //  {
  //  }    

  //auxillary methods
  double MeanFreePath(TrackState* _track)
  {
    double xsection =  MacroscopicXSection(_track);
    return (xsection <= 0.0) ? 0.0 : 1.0/xsection;
  }
  
  // the proposed along step physical interaction length
  double AlongStepGPIL(TrackState* _track)
  {
    GEANT_THIS_TYPE_TESTING_MARKER("");
    return  std::numeric_limits<double>::max();;
  }

  // the proposed post step physical interaction length
  double PostStepGPIL(TrackState* _track);

  // DoIt for the along step
  void AlongStepDoIt(TrackState* _track)
  {
    GEANT_THIS_TYPE_TESTING_MARKER("");
    ThreeVector rand = { get_rand(), get_rand(), get_rand() };
    rand.Normalize();
    _track->fDir = rand;
  }
  
  // DoIt for the post step
  int PostStepDoIt(TrackState* _track);
  
};

template <typename TEmProcess>
double ProxyEmProcess<TEmProcess>::PostStepGPIL(TrackState* track)
{
  GEANT_THIS_TYPE_TESTING_MARKER("");
  double step = std::numeric_limits<double>::max();

  double lambda = MeanFreePath(track);

  //reset or update the number of the interaction length left  
  if ( track->fPhysicsProcessState.fNumOfInteractLengthLeft <= 0.0 ) {
    track->fPhysicsProcessState.fNumOfInteractLengthLeft = -vecCore::math::Log(fRng->uniform());
  }
  else {
    track->fPhysicsProcessState.fNumOfInteractLengthLeft
      -=  track->fPhysicsState.fPstep/track->fPhysicsProcessState.fPhysicsInteractLength;
  }    

  step = lambda * track->fPhysicsProcessState.fNumOfInteractLengthLeft;

  //save lambda and the current step
  track->fPhysicsProcessState.fPhysicsInteractLength = lambda;
  track->fPhysicsState.fPstep = step;

  return step;
}

template <typename TEmProcess>
int ProxyEmProcess<TEmProcess>::PostStepDoIt(TrackState* track)
{
  GEANT_THIS_TYPE_TESTING_MARKER("");
  int nsec = 0;

  //TODO: connective to Material
  int Z = 10;

  nsec = static_cast<TEmProcess *>(this)-> FinalStateInteraction(Z, track->fPhysicsState.fEkin);

  return nsec;
}

}  // namespace geantx