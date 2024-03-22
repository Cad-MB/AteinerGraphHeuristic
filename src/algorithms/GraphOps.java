package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

public class GraphOps {

    private static GraphOps singleInstance = null; // Instance unique de la classe GraphOps

    private GraphOps() {
    }

    // Méthode pour obtenir l'instance unique de GraphOps
    public static GraphOps Singleton() {
        if (singleInstance == null)
            singleInstance = new GraphOps(); // Crée une nouvelle instance si elle n'existe pas déjà
        return singleInstance; // Retourne l'instance unique
    }

    // Méthode pour appliquer l'algorithme de Floyd-Warshall
    public ArrayList<Object> FloydWarshall(ArrayList<Point> points, int edgeThreshold) {
        ArrayList<Object> res = new ArrayList<>();
        int[][] paths = new int[points.size()][points.size()]; // Tableau pour stocker les chemins
        double[][] distances = new double[points.size()][points.size()]; // Tableau pour stocker les distances

        // Initialise les tableaux avec les distances entre les points
        for (int i = 0; i < distances.length; i++) {
            for (int j = 0; j < distances.length; j++) {
                if (points.get(i).distance(points.get(j)) < edgeThreshold) {
                    distances[i][j] = points.get(i).distance(points.get(j)); // Si la distance est inférieure au seuil, la stocke
                    paths[i][j] = j; // Met à jour le chemin
                } else {
                    distances[i][j] = Double.POSITIVE_INFINITY; // Sinon, distance infinie
                    paths[i][j] = -1;
                }
            }
        }

        // Applique l'algorithme de Floyd-Warshall pour trouver les plus courts chemins
        for (int k = 0; k < distances.length; k++) {
            for (int i = 0; i < distances.length; i++) {
                for (int j = 0; j < distances.length; j++) {
                    if (distances[i][j] > distances[i][k] + distances[k][j]) {
                        distances[i][j] = distances[i][k] + distances[k][j]; // Met à jour les distances
                        paths[i][j] = paths[i][k]; // Met à jour les chemins
                    }
                }
            }
        }

        res.add(paths); // Ajoute les chemins à la liste résultante
        res.add(distances); // Ajoute les distances à la liste résultante
        return res; // Retourne la liste résultante
    }

    // Méthode pour appliquer l'algorithme de Kruskal
    public ArrayList<Edge> Kruskal(ArrayList<Point> hitPoints, double[][] dist,
                                   HashMap<Point, Integer> mapPointIndice) {
        ArrayList<Edge> edges = new ArrayList<>();
        for (Point p : hitPoints) {
            for (Point q : hitPoints) {
                if (p.equals(q) || contains(edges, p, q))
                    continue;
                edges.add(new Edge(p, q)); // Ajoute les arêtes entre les points
            }
        }
        edges = sortEdges(edges, dist, mapPointIndice); // Trie les arêtes par distance
        ArrayList<Edge> kruskal = new ArrayList<>();
        Edge current;
        DefaultTeam.LabeledPoints forest = new DefaultTeam.LabeledPoints(hitPoints); // Initialise la structure forestière
        while (!edges.isEmpty()) {
            current = edges.removeFirst(); // Retire la première arête
            if (forest.lab(current.startPoint) != forest.lab(current.endPoint)) {
                kruskal.add(current); // Ajoute l'arête si elle ne forme pas de cycle
                forest.reLab(forest.lab(current.startPoint), forest.lab(current.endPoint)); // Met à jour les étiquettes
            }
        }
        return kruskal; // Retourne les arêtes de l'arbre de Kruskal
    }

    // Méthode pour intégrer les arêtes de Kruskal dans le graphe
    public ArrayList<Edge> integrateKruskalIntoGraph(ArrayList<Edge> kruskhaled, int[][] paths,
                                                     HashMap<Point, Integer> mapPointIndice) {
        ArrayList<Edge> best_edges = new ArrayList<>();
        for (Edge edge : kruskhaled) {
            Point p = edge.startPoint;
            Point q = edge.endPoint;
            ArrayList<Edge> edges_intermediaires = new ArrayList<>();
            int i = mapPointIndice.get(p);
            int j = mapPointIndice.get(q);
            if (paths[i][j] != j) {
                int k = paths[i][j];
                edges_intermediaires.add(new Edge(getKey(mapPointIndice, i),
                        getKey(mapPointIndice, k)));
                while (k != j) {
                    int previousK = k;
                    k = paths[k][j];
                    edges_intermediaires.add(new Edge(getKey(mapPointIndice, previousK),
                            getKey(mapPointIndice, k)));
                }
                edges_intermediaires.add(new Edge(getKey(mapPointIndice, k),
                        getKey(mapPointIndice, j)));
                best_edges.addAll(edges_intermediaires); // Ajoute les arêtes intermédiaires
            } else {
                best_edges.add(edge); // Ajoute l'arête si elle est directe
            }
        }
        return best_edges; // Retourne les meilleures arêtes
    }

    // Méthode pour calculer la longueur totale des arêtes
    public Double calculateTotalEdgeDistances(ArrayList<Edge> best_edges) {
        double totalLength = 0.0;
        for (Edge edge : best_edges) {
            totalLength += edge.calculateLength(); // Calcule la longueur de chaque arête
        }
        return totalLength; // Retourne la longueur totale
    }

    // Méthode pour transformer les arêtes en arbre
    @SuppressWarnings("unchecked")
    public Tree2D constructTreeFromEdges(ArrayList<Edge> edges, Point root) {
        ArrayList<Edge> remainder = new ArrayList<>();
        ArrayList<Point> subTreeRoots = new ArrayList<>();
        Edge current;
        while (!edges.isEmpty()) {
            current = edges.removeFirst(); // Retire la première arête
            if (current.startPoint.equals(root)) {
                subTreeRoots.add(current.endPoint); // Ajoute le point de fin à la liste des racines des sous-arbres
            } else {
                if (current.endPoint.equals(root)) {
                    subTreeRoots.add(current.startPoint); // Ajoute le point de départ à la liste des racines des sous-arbres
                } else {
                    remainder.add(current); // Ajoute l'arête à la liste restante
                }
            }
        }
        ArrayList<Tree2D> subTrees = new ArrayList<>();
        for (Point subTreeRoot : subTreeRoots)
            subTrees.add(constructTreeFromEdges((ArrayList<Edge>) remainder.clone(), subTreeRoot)); // Récursion pour construire les sous-arbres
        return new Tree2D(root, subTrees); // Retourne l'arbre
    }

    // Méthode pour obtenir la clé à partir de la valeur dans une HashMap
    public Point getKey(HashMap<Point, Integer> map, int value) {
        for (Entry<Point, Integer> entry : map.entrySet()) {
            if (entry.getValue() == value)
                return entry.getKey(); // Retourne la clé correspondant à la valeur spécifiée
        }
        return new Point(-1, -1); // Retourne un point par défaut si la valeur n'est pas trouvée
    }

    // Méthode pour obtenir les arêtes extrêmes
    public ArrayList<Edge> getExtremeEdges(ArrayList<Edge> edges, ArrayList<Point> hitPoints) {
        ArrayList<Edge> extremeEdges = new ArrayList<>();
        ArrayList<Point> points = splitEdges(edges); // Sépare les points des arêtes
        for (Edge edge1 : edges) {
            int countP1 = countOccurrences(points, edge1.startPoint);
            int countP2 = countOccurrences(points, edge1.endPoint);
            if (hitPoints.getFirst() != edge1.startPoint && hitPoints.getFirst() != edge1.endPoint) {
                if (countP1 <= 1 || countP2 <= 1) {
                    extremeEdges.add(edge1); // Ajoute l'arête si elle est extrême
                }
            }
        }
        return extremeEdges; // Retourne les arêtes extrêmes
    }

    // Méthode pour obtenir le point extrême
    public Point getExtremePoint(ArrayList<Edge> edges, ArrayList<Point> biggestEdgePoints) {
        Point extreme = null;
        ArrayList<Point> points = splitEdges(edges); // Sépare les points des arêtes
        for (Edge edge1 : edges) {
            int cp1 = countOccurrences(points, edge1.startPoint);
            int cp2 = countOccurrences(points, edge1.endPoint);
            if ((cp1 <= 1) && (biggestEdgePoints.contains(edge1.startPoint))) {
                extreme = edge1.startPoint; // Met à jour le point extrême
            } else if ((cp2 <= 1) && (biggestEdgePoints.contains(edge1.endPoint))) {
                extreme = edge1.endPoint; // Met à jour le point extrême
            }
        }
        return extreme; // Retourne le point extrême
    }

    // Méthode pour séparer les points des arêtes
    public ArrayList<Point> splitEdges(ArrayList<Edge> edgesAlgo) {
        ArrayList<Point> points = new ArrayList<>();
        for (Edge edge : edgesAlgo) {
            points.add(edge.startPoint); // Ajoute le point de départ
            points.add(edge.endPoint); // Ajoute le point de fin
        }
        return points; // Retourne les points séparés
    }

    // Méthode pour compter les occurrences d'un point
    public int countOccurrences(ArrayList<Point> points, Point p) {
        int cpt = 0;
        for (Point point : points) {
            if (point.x == p.x && point.y == p.y) {
                cpt++; // Incrémente le compteur si le point est trouvé
            }
        }
        return cpt; // Retourne le nombre d'occurrences
    }

    // Méthode pour vérifier si une arête existe déjà
    private boolean contains(ArrayList<Edge> edges, Point p, Point q) {
        for (Edge e : edges) {
            if (e.startPoint.equals(p) && e.endPoint.equals(q) || e.startPoint.equals(q) && e.endPoint.equals(p))
                return true; // Retourne vrai si l'arête existe déjà
        }
        return false; // Retourne faux sinon
    }

    // Méthode pour trier les arêtes par distance
    private ArrayList<Edge> sortEdges(ArrayList<Edge> edges, double[][] distances, HashMap<Point, Integer> map) {
        if (edges.size() == 1) {
            return edges; // Retourne les arêtes si la taille est 1
        }
        ArrayList<Edge> left = new ArrayList<>();
        ArrayList<Edge> right = new ArrayList<>();
        int n = edges.size();
        for (int i = 0; i < n / 2; i++) {
            left.add(edges.removeFirst()); // Ajoute la première moitié des arêtes à gauche
        }
        while (!edges.isEmpty()) {
            right.add(edges.removeFirst()); // Ajoute le reste des arêtes à droite
        }
        left = sortEdges(left, distances, map); // Récursion pour trier les arêtes à gauche
        right = sortEdges(right, distances, map); // Récursion pour trier les arêtes à droite
        ArrayList<Edge> result = new ArrayList<>();
        while (!left.isEmpty() || !right.isEmpty()) {
            if (left.isEmpty()) {
                result.add(right.removeFirst()); // Ajoute les arêtes de droite si la liste de gauche est vide
                continue;
            }
            if (right.isEmpty()) {
                result.add(left.removeFirst()); // Ajoute les arêtes de gauche si la liste de droite est vide
                continue;
            }
            if (distances[map.get(left.getFirst().startPoint)][map.get(left.getFirst().endPoint)] < distances[map.get(right.getFirst().startPoint)][map.get(right.getFirst().endPoint)])
                result.add(left.removeFirst()); // Ajoute l'arête de gauche si sa distance est inférieure
            else
                result.add(right.removeFirst()); // Ajoute l'arête de droite sinon
        }
        return result; // Retourne les arêtes triées
    }
}
