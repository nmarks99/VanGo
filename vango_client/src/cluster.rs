use diff_drive::rigid2d::Vector2D;
use diff_drive::utils::almost_equal;
use diff_drive::utils::distance;

/// Cluster object contains a vector of points (Vector2D)
/// which are points that are all within some linear distance of each other
/// specified by the given tolerance
#[derive(Debug, Clone)]
pub struct Cluster {
    threshold: f64,
    points: Vec<Vector2D<f64>>,
}

impl Cluster {
    /// Creates a new cluster object with the given distance tolerance
    pub fn new(threshold: f64) -> Self {
        Cluster {
            threshold,
            points: vec![],
        }
    }

    /// Creates a new cluster object with the given distance tolerance
    /// and adds the given point to the cluster
    pub fn new_with_point(threshold: f64, point: Vector2D<f64>) -> Self {
        Cluster {
            threshold,
            points: vec![point],
        }
    }

    /// Returns true if the cluster contains the point (within some tolerance)
    /// otherwise returns false. Currently the tolerance is hardcoded for almost_equal
    pub fn contains(&self, point: Vector2D<f64>) -> bool {
        for p in &self.points {
            if almost_equal(p.x, point.x, 1e-6) && almost_equal(p.y, point.y, 1e-6) {
                return true;
            }
        }
        return false;
    }

    /// Adds a point to the cluster without checking first that it
    /// belongs (is within the threshold distance)
    pub fn blind_add(&mut self, point: Vector2D<f64>) -> bool {
        if self.contains(point) {
            return false;
        } else {
            self.points.push(point);
            return true;
        }
    }

    /// Adds a point to the cluster if it is within the
    /// threshold distance of some other point in the cluster
    pub fn add(&mut self, point: Vector2D<f64>) -> bool {
        if self.points.len() == 0 {
            self.points.push(point);
            return true;
        }

        for p in &self.points {
            if !self.contains(point) {
                let d = distance(*p, point);
                if d > 0.0 && d < self.threshold {
                    self.points.push(point);
                    return true;
                }
            } else {
                break;
            }
        }
        return false;
    }

    /// Returns the vector of points in the cluster
    pub fn to_vec(self) -> Vec<Vector2D<f64>> {
        self.points
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // use crate::cluster::Cluster;
    // use diff_drive::rigid2d::Vector2D;
    // use diff_drive::utils::almost_equal;

    #[test]
    fn new() {
        let cluster = Cluster::new(1.0);
        assert!(almost_equal(cluster.threshold, 1.0, 1e-9));
    }

    #[test]
    fn new_from_point() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        assert!(almost_equal(cluster.threshold, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 2.0, 1e-6));
    }

    #[test]
    fn contains() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        assert!(cluster.contains(Vector2D { x: 1.0, y: 2.0 }));
    }

    #[test]
    fn blind_add() {
        let mut cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        let should_be_true = cluster.blind_add(Vector2D { x: 2.0, y: 3.0 });
        assert!(should_be_true);

        let should_be_false = cluster.blind_add(Vector2D { x: 1.0, y: 2.0 });
        assert!(!should_be_false);

        assert_eq!(cluster.points.len(), 2);
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 2.0, 1e-6));
        assert!(almost_equal(cluster.points[1].x, 2.0, 1e-6));
        assert!(almost_equal(cluster.points[1].y, 3.0, 1e-6));
    }

    #[test]
    fn add() {
        let mut cluster = Cluster::new(1.0);
        cluster.add(Vector2D { x: 1.0, y: 0.0 });
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 0.0, 1e-6));

        cluster.add(Vector2D { x: 1.5, y: 0.0 });
        assert!(almost_equal(cluster.points[1].x, 1.5, 1e-6));
        assert!(almost_equal(cluster.points[1].y, 0.0, 1e-6));

        cluster.add(Vector2D { x: 2.6, y: 0.0 });
        assert_eq!(cluster.points.len(), 2);
    }

    #[test]
    fn to_vec() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        let v = cluster.to_vec();
        assert!(almost_equal(v[0].x, 1.0, 1e-6));
        assert!(almost_equal(v[0].y, 2.0, 1e-6));
    }
}
