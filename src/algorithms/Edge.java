package algorithms;

import java.awt.Point;

public final class Edge {
    final Point startPoint;  // Déclaration de la variable startPoint représentant le point de départ de l'arête.
    final Point endPoint;    // Déclaration de la variable endPoint représentant le point d'arrivée de l'arête.
    private Double cachedLength;  // Cache pour stocker la longueur de l'arête.

    /**
     * Constructeur de la classe Edge.
     * @param startPoint Le point de départ de l'arête.
     * @param endPoint Le point d'arrivée de l'arête.
     */
    Edge(final Point startPoint, final Point endPoint) {
        this.startPoint = startPoint;  // Initialisation du point de départ de l'arête.
        this.endPoint = endPoint;      // Initialisation du point d'arrivée de l'arête.
    }

    /**
     * Méthode pour calculer la longueur de l'arête.
     * @return La longueur de l'arête.
     */
    double calculateLength() {
        if (cachedLength == null) {  // Vérification si la longueur est déjà calculée et mise en cache.
            cachedLength = startPoint.distance(endPoint);  // Calcul de la distance entre les points et stockage dans le cache.
        }
        return cachedLength;  // Retourne la longueur calculée ou mise en cache.
    }
}
