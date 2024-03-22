package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;

public class DefaultTeam {

  // Classe interne pour gérer les points étiquetés
  static class LabeledPoints {
    private final ArrayList<Point> points;  // Liste des points
    private final int[] lab;  // Tableau d'étiquettes

    // Constructeur prenant une liste de points en entrée
    protected LabeledPoints(ArrayList<Point> points) {
      this.points = (ArrayList<Point>) points.clone();  // Clonage de la liste des points pour éviter les modifications externes
      lab = new int[points.size()];  // Initialisation du tableau d'étiquettes avec la taille de la liste des points
      for (int i = 0; i < points.size(); i++)
        lab[i] = i;  // Chaque point est initialement étiqueté avec son indice
    }

    // Méthode pour re-étiqueter les points
    protected void reLab(int j, int k) {
      for (int i = 0; i < lab.length; i++)
        if (lab[i] == j)
          lab[i] = k;  // Remplace toutes les occurrences de l'indice j par k dans le tableau d'étiquettes
    }

    // Méthode pour obtenir l'étiquette d'un point donné
    protected int lab(Point p) {
      for (int i = 0; i < points.size(); i++)
        if (p.equals(points.get(i)))
          return lab[i];  // Retourne l'étiquette du point s'il est trouvé dans la liste
      return -1;  // Retourne -1 si le point n'est pas trouvé
    }
  }

  // Méthode pour calculer les chemins les plus courts entre les points
  public ArrayList<Object> calculateShortestPaths(ArrayList<Point> points, int edgeThreshold) {
    return GraphOps.Singleton().FloydWarshall(points, edgeThreshold);  // Utilise l'algorithme de Floyd-Warshall pour calculer les chemins les plus courts
  }

  // Méthode pour calculer l'arbre de Steiner
  public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
    ArrayList<Object> path_dist = calculateShortestPaths(points, edgeThreshold);  // Calcule les chemins les plus courts
    int[][] paths = (int[][]) path_dist.get(0);  // Récupère les chemins
    double[][] dist = (double[][]) path_dist.get(1);  // Récupère les distances

    HashMap<Point, Integer> pointIndexMap = new HashMap<>();  // Map pour stocker les index des points
    for (int i = 0; i < points.size(); i++) {
      pointIndexMap.put(points.get(i), i);  // Associe chaque point à son index
    }

    ArrayList<Edge> kruskalEdges = GraphOps.Singleton().Kruskal(hitPoints, dist, pointIndexMap);  // Applique l'algorithme de Kruskal
    ArrayList<Edge> bestEdges = GraphOps.Singleton().integrateKruskalIntoGraph(kruskalEdges, paths, pointIndexMap);  // Intègre les arêtes de Kruskal dans le graphe

    return GraphOps.Singleton().constructTreeFromEdges(bestEdges, bestEdges.getFirst().startPoint);  // Transforme les arêtes en arbre
  }

  // Méthode pour calculer l'arbre de Steiner avec restriction budgétaire
  public Tree2D calculSteinerBudget(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
    ArrayList<Object> path_dist = calculateShortestPaths(points, edgeThreshold);  // Calcule les chemins les plus courts
    int[][] paths = (int[][]) path_dist.get(0);  // Récupère les chemins
    double[][] dist = (double[][]) path_dist.get(1);  // Récupère les distances

    HashMap<Point, Integer> mapPointIndice = new HashMap<>();  // Map pour stocker les index des points
    for (int i = 0; i < points.size(); i++) {
      mapPointIndice.put(points.get(i), i);  // Associe chaque point à son index
    }

    ArrayList<Edge> kruskalEdges = GraphOps.Singleton().Kruskal(hitPoints, dist, mapPointIndice);  // Applique l'algorithme de Kruskal
    ArrayList<Edge> bestEdges = GraphOps.Singleton().integrateKruskalIntoGraph(kruskalEdges, paths, mapPointIndice);  // Intègre les arêtes de Kruskal dans le graphe

    // Copie de la liste des points de hitPoints
    @SuppressWarnings("unchecked")
    ArrayList<Point> hitpointsBudget = (ArrayList<Point>) hitPoints.clone();

    // Boucle pour réduire la longueur totale des arêtes jusqu'à respecter le budget
    while (Double.compare(GraphOps.Singleton().calculateTotalEdgeDistances(bestEdges), 1664) > 0) {
      ArrayList<Edge> extrem_edges = GraphOps.Singleton().getExtremeEdges(kruskalEdges, hitPoints);  // Obtient les arêtes extrêmes
      Edge biggest_edge = extrem_edges.getFirst();  // Sélectionne la plus grande arête

      // Boucle pour trouver l'arête la plus grande parmi les arêtes extrêmes
      for (Edge e : extrem_edges) {
        if (Double.compare(biggest_edge.calculateLength(), e.calculateLength()) < 0) {
          biggest_edge = e;
        }
      }

      ArrayList<Edge> big = new ArrayList<>();  // Liste pour stocker l'arête la plus grande
      big.add(biggest_edge);  // Ajoute l'arête la plus grande à la liste

      ArrayList<Point> biggest_edge_pts = GraphOps.Singleton().splitEdges(big);  // Sépare l'arête la plus grande en points
      Point extrem = GraphOps.Singleton().getExtremePoint(kruskalEdges, biggest_edge_pts);  // Obtient le point extrême

      hitpointsBudget.remove(extrem);  // Retire le point extrême de la liste des points

      kruskalEdges = GraphOps.Singleton().Kruskal(hitpointsBudget, dist, mapPointIndice);  // Réapplique Kruskal
      bestEdges = GraphOps.Singleton().integrateKruskalIntoGraph(kruskalEdges, paths, mapPointIndice);  // Réintègre les arêtes de Kruskal dans le graphe
    }

    // Ajoute progressivement les points retirés jusqu'à respecter le budget
    for (Point point : hitPoints) {
      if (!hitpointsBudget.contains(point)) {
        hitpointsBudget.add(point);
        kruskalEdges = GraphOps.Singleton().Kruskal(hitpointsBudget, dist, mapPointIndice);  // Réapplique Kruskal
        bestEdges = GraphOps.Singleton().integrateKruskalIntoGraph(kruskalEdges, paths, mapPointIndice);  // Réintègre les arêtes de Kruskal dans le graphe
        if (Double.compare(GraphOps.Singleton().calculateTotalEdgeDistances(bestEdges), 1664.0) > 0) {
          hitpointsBudget.remove(point);  // Retire le point ajouté s'il dépasse le budget
        } else {
          break;
        }
      }
    }

    kruskalEdges = GraphOps.Singleton().Kruskal(hitpointsBudget, dist, mapPointIndice);  // Réapplique Kruskal
    bestEdges = GraphOps.Singleton().integrateKruskalIntoGraph(kruskalEdges, paths, mapPointIndice);  // Réintègre les arêtes de Kruskal dans le graphe

    return GraphOps.Singleton().constructTreeFromEdges(bestEdges, bestEdges.getFirst().startPoint);  // Transforme les arêtes en arbre
  }
}
