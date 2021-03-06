//===------------------ GeantX --------------------------------------------===//
//
// Geant Exascale Pilot
//
// For the licensing terms see LICENSE file.
// For the list of contributors see CREDITS file.
// Copyright (C) 2019, Geant Exascale Pilot team,  All rights reserved.
//===----------------------------------------------------------------------===//
/**
 * @file Geant/proxy/ProxyPhysicsTable.cuh
 * @brief 
 */
//===----------------------------------------------------------------------===//

#pragma once

#include "Geant/core/Config.hpp"
#include "Geant/proxy/ProxyPhysicsVector.cuh"

namespace geantx {

class ProxyPhysicsTable {

 public:
  GEANT_HOST_DEVICE 
  ProxyPhysicsTable();

  GEANT_HOST_DEVICE 
  ~ProxyPhysicsTable();

  GEANT_HOST
  void Relocate(void *devPtr);

  GEANT_HOST_DEVICE 
  inline size_t SizeOfTable() { return fTableSize; }

  GEANT_HOST_DEVICE 
  inline int  NumberOfVector() { return fNumPhysicsVector; }

  GEANT_HOST_DEVICE 
  double Value(int index, double energy);

  GEANT_HOST_DEVICE 
  void Print();

  GEANT_HOST
  bool RetrievePhysicsTable(const std::string& fileName);

  //for tests
  GEANT_HOST
  ProxyPhysicsVector* GetPhysicsVector(int index) { return fPhysicsVectors[index]; }

  GEANT_HOST
  ProxyPhysicsVector** GetPhysicsVectors() { return fPhysicsVectors; }

private:
  size_t fTableSize;
  int fNumPhysicsVector;
  ProxyPhysicsVector **fPhysicsVectors;
};

} // namespace geantx

