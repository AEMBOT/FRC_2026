package com.aembot.lib.superstructure.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

class Edge<T> {
  private final T target;
  private final double weight;

  public Edge(T target, double weight) {
    this.target = target;
    this.weight = weight;
  }

  public T getTarget() {
    return target;
  }

  public double getWeight() {
    return weight;
  }
}

public class Graph<T> {

  private final Map<T, List<Edge<T>>> adjacencyList = new HashMap<>();

  public void addNode(T node) {
    adjacencyList.putIfAbsent(node, new ArrayList<>());
  }

  public Set<T> getNodes() {
    return adjacencyList.keySet();
  }

  public int getNumNodes() {
    return adjacencyList.size();
  }

  public int getNumEdges(T node) {
    return adjacencyList.get(node).size();
  }

  public void addEdge(T source, T target, double weight) {
    addNode(source);
    addNode(target);
    adjacencyList.get(source).add(new Edge<>(target, weight));
  }

  public List<Edge<T>> getNeighbors(T node) {
    return adjacencyList.getOrDefault(node, Collections.emptyList());
  }
}
