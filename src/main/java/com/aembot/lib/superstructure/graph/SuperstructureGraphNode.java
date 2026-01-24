package com.aembot.lib.superstructure.graph;

import java.util.Objects;

/** A node in the superstructure graph that represents a single state */
public class SuperstructureGraphNode {
  // Name of this state
  public final String kStateName;

  public SuperstructureGraphNode(String stateName) {
    this.kStateName = stateName;
  }

  // Use the state name as the hash
  @Override
  public int hashCode() {
    return Objects.hash(kStateName);
  }

  @Override
  public boolean equals(Object o) {
    // Return true if its the same object in memory
    if (this == o) return true;
    // Return false if its null or a different class type
    if (o == null || getClass() != o.getClass()) return false;

    SuperstructureGraphNode that = (SuperstructureGraphNode) o;
    return Objects.equals(kStateName, that.kStateName);
  }

  // Override the to string function to return the state name
  @Override
  public String toString() {
    return kStateName;
  }
}
